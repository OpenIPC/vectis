/*
 *
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * vectis.c — Vectis: serial console for the OpenIPC UART 38x38 board.
 *
 * One binary, three roles that run side by side:
 *   - console: a local tty (-u) or an RFC 2217 server (-h/-p), with the
 *     RTS+DTR reset pulse on Ctrl+P and a break on Ctrl+B;
 *   - server:  -l serves the local port over TCP (Telnet/RFC 2217, raw);
 *   - web:     -w serves a power-control page and a small JSON API.
 *
 * vectis-bootrom.c is the original UART bridge (with the HiSilicon
 * BOOTROM-CATCH extension) this program grew out of; see README.md.
 *
 */

#define _GNU_SOURCE
#define _XOPEN_SOURCE 600
#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <poll.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <time.h>
#include <ctype.h>
#include <stdarg.h>
#include <syslog.h>

/* ----- Telnet codes ----- */
#define IAC   255
#define DONT  254
#define DO    253
#define WONT  252
#define WILL  251
#define SB    250
#define SE    240
#define BRK   243  /* Telnet break (IAC BRK, RFC 854 §3.3) */

/* Telnet options */
#define TELOPT_BINARY     0
#define TELOPT_ECHO       1
#define TELOPT_SGA        3
#define TELOPT_COMPORT    44   /* RFC 2217 */

/* Port-control commands (client -> server) */
#define CPC_SET_BAUDRATE        1
#define CPC_SET_DATASIZE        2
#define CPC_SET_PARITY          3
#define CPC_SET_STOPSIZE        4
#define CPC_SET_CONTROL         5
#define CPC_NOTIFY_LINESTATE    6
#define CPC_NOTIFY_MODEMSTATE   7
#define CPC_FLOWCONTROL_SUSP    8
#define CPC_FLOWCONTROL_RESUM   9
#define CPC_SET_LINESTATE_MASK  10
#define CPC_SET_MODEMSTATE_MASK 11
#define CPC_PURGE_DATA          12

/* Values for CPC_SET_CONTROL (RTS/DTR/flow control) */
#define CPC_CTRL_REQ_FLOW   0
#define CPC_CTRL_NO_FLOW    1
#define CPC_CTRL_XON_XOFF   2
#define CPC_CTRL_HW_FLOW    3

#define CPC_CTRL_REQ_DTR    7
#define CPC_CTRL_DTR_ON     8
#define CPC_CTRL_DTR_OFF    9

#define CPC_CTRL_REQ_RTS    10
#define CPC_CTRL_RTS_ON     11
#define CPC_CTRL_RTS_OFF    12

/* Parity values */
#define CPC_PARITY_NONE    1
#define CPC_PARITY_ODD     2
#define CPC_PARITY_EVEN    3
#define CPC_PARITY_MARK    4
#define CPC_PARITY_SPACE   5

/* Stop-bit values */
#define CPC_STOP_1   1
#define CPC_STOP_2   2

#define PROGRAM_VERSION      "1.6.0"
#define PROGRAM_RELEASE_DATE "2026-09-06"

#define RECONNECT_DELAY_S 5

/* Buffer sizes */
#define SB_BUF_SIZE      512   /* Sub-negotiation buffer size */
#define OUTPUT_BUF_SIZE  256   /* Stdout batching buffer size */
#define READ_BUF_SIZE    4096  /* Transport read buffer size */

/* Interactive session control keys — used in the main I/O loop */
#define KEY_RESET  0x10  /* Ctrl+P — RTS+DTR reset pulse */
#define KEY_BREAK  0x02  /* Ctrl+B — send break signal */
#define KEY_EXIT   0x1D  /* Ctrl+] — exit the program */

/* Telnet parser states — declared here so disconnect() can reset them */
enum tn_state {
    TS_DATA,
    TS_IAC,
    TS_NEGOT,    /* waiting for the option byte after DO/DONT/WILL/WONT */
    TS_SB,       /* collecting sub-negotiation data */
    TS_SB_IAC    /* IAC seen inside SB */
};

/* Global state */
static int g_fd = -1;
static struct termios g_old_tio;
static struct termios g_old_dev_tio;
static int g_tio_saved    = 0;
static int g_dev_tio_saved = 0;
static volatile sig_atomic_t g_quit = 0;
static const char *g_progname = "vectis";
static int g_serial_mode  = 0;
static int g_reset_ms     = 200;
static int g_reconnect    = 0;
static int g_no_crlf      = 0;  /* -n: disable LF->CR+LF translation */
static int g_interactive  = 0;  /* stdin is a tty */
static int g_watch_stdin  = 1;  /* poll stdin; cleared once it hits EOF */
static int g_power_on     = 0;  /* last commanded RTS/DTR state, 1 = both asserted */
static int g_dtr_on       = 0;  /* the two lines separately: only the -l server */
static int g_rts_on       = 0;  /* (RFC 2217 SET-CONTROL) can move one at a time */

/* Last power event, whoever caused it, so the web page can show a reset
 * that came from the keyboard or from a TCP client (a 200 ms pulse is
 * invisible to a page that polls every 2 s). */
static uint32_t    g_ev_seq    = 0;
static const char *g_ev_kind   = "";   /* "on", "off", "reset" */
static const char *g_ev_source = "";   /* "web", "keyboard", "tcp", "rfc2217", "link" */
static long long   g_ev_ms     = 0;
static int g_link_quiet   = 0;  /* -w: log only the first link failure of an outage */
static void set_serial_signal_state(int signal_flag, int active);
static void web_shutdown(void);
static void srv_shutdown(void);

/* Telnet parser state */
static enum tn_state g_tn_state = TS_DATA;
static uint8_t       g_negot_cmd = 0;
/* 512 bytes: generous for any RFC 2217 sub-negotiation payload. */
static uint8_t       g_sb_buf[SB_BUF_SIZE];
static size_t        g_sb_len = 0;

/* ---------- Logging ---------- */

static void log_message(int priority, const char *fmt, ...)
{
    char buf[1024];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);

    if (n < 0) {
        /* Encoding error: use a fallback message. */
        syslog(priority, "log_message: encoding error");
        fprintf(stderr, "%s: log_message: encoding error\r\n", g_progname);
        return;
    }

    if ((size_t)n >= sizeof(buf)) {
        /* Message was truncated; issue a separate warning so the
           caller knows the output is incomplete. */
        syslog(LOG_WARNING, "log_message: truncating %d byte message to %zu",
               n, sizeof(buf) - 1);
    }

    syslog(priority, "%s", buf);
    /* Use \r\n: terminal may be in raw mode (OPOST/ONLCR off). */
    fprintf(stderr, "%s: %s\r\n", g_progname, buf);
}

static void log_errno(int priority, const char *fmt, ...)
{
    char buf[1024];
    int saved_errno = errno;
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);

    syslog(priority, "%s: %s", buf, strerror(saved_errno));
    /* Use \r\n: terminal may be in raw mode (OPOST/ONLCR off). */
    fprintf(stderr, "%s: %s: %s\r\n", g_progname, buf, strerror(saved_errno));
}

static void die_errno(const char *msg)
{
    log_errno(LOG_ERR, "%s", msg);
    exit(EXIT_FAILURE);
}

static void sleep_us(unsigned int usec)
{
    while (usleep(usec) != 0 && errno == EINTR)
        ;
}

static long long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static void print_version(const char *prog)
{
    printf("%s %s\n", prog, PROGRAM_VERSION);
    printf("Release date: %s\n", PROGRAM_RELEASE_DATE);
}

/* ---------- Utilities ---------- */

static void restore_terminal(void)
{
    if (g_tio_saved) {
        if (tcsetattr(STDIN_FILENO, TCSANOW, &g_old_tio) != 0)
            log_errno(LOG_WARNING, "restore_terminal: tcsetattr failed");
        g_tio_saved = 0;
    }
}

static void restore_serial(void)
{
    if (g_dev_tio_saved && g_fd >= 0) {
        if (tcsetattr(g_fd, TCSANOW, &g_old_dev_tio) != 0)
            log_errno(LOG_WARNING, "restore_serial: tcsetattr failed");
        g_dev_tio_saved = 0;
    }
}

static void disconnect(void)
{
    if (g_serial_mode && g_fd >= 0) {
        set_serial_signal_state(TIOCM_RTS, 0);
        set_serial_signal_state(TIOCM_DTR, 0);
    }
    restore_serial();
    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
    /* Reset Telnet parser state for potential reconnect. */
    g_tn_state  = TS_DATA;
    g_negot_cmd = 0;
    g_sb_len    = 0;
    g_power_on  = 0;
    g_dtr_on    = 0;
    g_rts_on    = 0;
}

static void cleanup(void)
{
    restore_terminal();
    disconnect();
    srv_shutdown();
    web_shutdown();
}

static void on_signal(int sig)
{
    (void)sig;
    g_quit = 1;
}

static void set_raw_terminal(void)
{
    struct termios tio;
    if (tcgetattr(STDIN_FILENO, &g_old_tio) != 0)
        die_errno("tcgetattr(stdin)");
    tio = g_old_tio;
    /* Full raw mode so every key press (including Ctrl+P) reaches us. */
    cfmakeraw(&tio);
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &tio) != 0)
        die_errno("tcsetattr(stdin)");
    /* Mark saved only after terminal is actually configured (N5). */
    g_tio_saved = 1;
}

static const struct { int baud; speed_t speed; } baud_table[] = {
    { 300, B300 }, { 1200, B1200 }, { 2400, B2400 }, { 4800, B4800 },
    { 9600, B9600 }, { 19200, B19200 }, { 38400, B38400 }, { 57600, B57600 },
    { 115200, B115200 }, { 230400, B230400 }, { 460800, B460800 }, { 921600, B921600 },
};

static int speed_for_baud(int baud, speed_t *out)
{
    for (size_t i = 0; i < sizeof baud_table / sizeof baud_table[0]; i++) {
        if (baud_table[i].baud == baud) {
            *out = baud_table[i].speed;
            return 0;
        }
    }
    return -1;
}

static int baud_from_speed(speed_t speed)
{
    for (size_t i = 0; i < sizeof baud_table / sizeof baud_table[0]; i++)
        if (baud_table[i].speed == speed)
            return baud_table[i].baud;
    return 0;
}

static int configure_serial(int fd, int baud, int data_bits, int stop_bits, char parity)
{
    struct termios tio;
    speed_t speed;

    if (speed_for_baud(baud, &speed) != 0) {
        log_message(LOG_ERR, "Unsupported baud rate: %d", baud);
        return -1;
    }
    if (tcgetattr(fd, &g_old_dev_tio) != 0) {
        log_errno(LOG_ERR, "tcgetattr(serial)");
        return -1;
    }

    tio = g_old_dev_tio;
    if (cfsetispeed(&tio, speed) != 0 || cfsetospeed(&tio, speed) != 0) {
        log_errno(LOG_ERR, "cfsetispeed/cfsetospeed");
        return -1;
    }
    tio.c_cflag |= (CREAD | CLOCAL);
    tio.c_cflag &= ~CSIZE;

    switch (data_bits) {
    case 5: tio.c_cflag |= CS5; break;
    case 6: tio.c_cflag |= CS6; break;
    case 7: tio.c_cflag |= CS7; break;
    case 8: tio.c_cflag |= CS8; break;
    default:
        log_message(LOG_ERR, "Invalid data bits: %d", data_bits);
        return -1;
    }

    switch (parity) {
    case 'N':
        tio.c_cflag &= ~PARENB;
        break;
    case 'E':
        tio.c_cflag |= PARENB;
        tio.c_cflag &= ~PARODD;
        break;
    case 'O':
        tio.c_cflag |= PARENB;
        tio.c_cflag |= PARODD;
        break;
    default:
        log_message(LOG_ERR, "Invalid parity: %c (must be N/E/O)", parity);
        return -1;
    }

    if (stop_bits == 1) {
        tio.c_cflag &= ~CSTOPB;
    } else if (stop_bits == 2) {
        tio.c_cflag |= CSTOPB;
    } else {
        log_message(LOG_ERR, "Stop bits must be 1 or 2");
        return -1;
    }

#ifdef CRTSCTS
    tio.c_cflag &= ~CRTSCTS;
#endif
    tio.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNCR);
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tio.c_oflag &= ~OPOST;
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;

    if (tcflush(fd, TCIFLUSH) != 0) {
        log_errno(LOG_ERR, "tcflush(serial)");
        return -1;
    }
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        log_errno(LOG_ERR, "tcsetattr(serial)");
        return -1;
    }

    g_dev_tio_saved = 1;
    return 0;
}

static void set_serial_signal_state(int signal_flag, int active)
{
    if (g_fd < 0)
        return;

    if (active) {
        if (ioctl(g_fd, TIOCMBIS, &signal_flag) == -1)
            log_errno(LOG_ERR, "Failed to activate serial signal");
    } else {
        if (ioctl(g_fd, TIOCMBIC, &signal_flag) == -1)
            log_errno(LOG_ERR, "Failed to deactivate serial signal");
    }
}

/* Guaranteed write of N bytes to a file descriptor. Returns -1 on error. */
static int write_all(int fd, const void *buf, size_t n)
{
    const uint8_t *p = (const uint8_t *)buf;
    while (n > 0) {
        ssize_t k = write(fd, p, n);
        if (k < 0) {
            if (errno == EINTR)
                continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                /* Wait for the FD to become writable before retrying. */
                struct pollfd pfd = { .fd = fd, .events = POLLOUT };
                poll(&pfd, 1, -1);
                continue;
            }
            return -1;
        }
        if (k == 0)
            return -1;
        p += k;
        n -= (size_t)k;
    }
    return 0;
}

/* ---------- Telnet / RFC 2217 ---------- */

/* Send a simple negotiation command: IAC <cmd> <opt> */
static int telnet_send_negot(uint8_t cmd, uint8_t opt)
{
    uint8_t buf[3] = { IAC, cmd, opt };
    return write_all(g_fd, buf, 3);
}

/*
 * Send sub-negotiation for COM Port Control:
 * IAC SB COM-PORT-OPTION <subcmd> <data...> IAC SE
 *
 * Inside payload data, bytes with value 255 (IAC) are doubled.
 * The entire packet is assembled in a stack buffer and sent in one
 * write_all call to avoid partial-send races and excess syscalls.
 */
static int comport_send(uint8_t subcmd, const uint8_t *data, size_t len)
{
    /* Max payload: 4 bytes × 2 (worst-case IAC escaping) + 4 hdr + 2 tail. */
    uint8_t pkt[32];
    size_t  pos = 0;

    /* Verify payload fits before writing: 4 hdr + worst-case 2×len + 2 tail. */
    if (4 + 2 * len + 2 > sizeof(pkt)) {
        log_message(LOG_ERR, "comport_send: payload too large (%zu bytes)", len);
        return -1;
    }

    pkt[pos++] = IAC;
    pkt[pos++] = SB;
    pkt[pos++] = TELOPT_COMPORT;
    pkt[pos++] = subcmd;

    for (size_t i = 0; i < len; i++) {
        pkt[pos++] = data[i];
        if (data[i] == IAC)
            pkt[pos++] = IAC; /* escape IAC inside payload */
    }

    pkt[pos++] = IAC;
    pkt[pos++] = SE;

    return write_all(g_fd, pkt, pos);
}

/* Set the baud rate (4 bytes, big-endian). */
static int comport_set_baudrate(uint32_t baud)
{
    uint8_t d[4];
    d[0] = (baud >> 24) & 0xFF;
    d[1] = (baud >> 16) & 0xFF;
    d[2] = (baud >> 8)  & 0xFF;
    d[3] = (baud)       & 0xFF;
    return comport_send(CPC_SET_BAUDRATE, d, 4);
}

static int comport_set_datasize(uint8_t bits)  { return comport_send(CPC_SET_DATASIZE, &bits, 1); }
static int comport_set_parity(uint8_t parity)  { return comport_send(CPC_SET_PARITY,   &parity, 1); }
static int comport_set_stop(uint8_t stop)      { return comport_send(CPC_SET_STOPSIZE, &stop,   1); }
static int comport_set_control(uint8_t value)  { return comport_send(CPC_SET_CONTROL,  &value,  1); }

/* ---------- Incoming Telnet command handling ---------- */

static void handle_negot(uint8_t cmd, uint8_t opt)
{
    /* Basic policy: accept binary/SGA/com-port and reject everything else.
       We also tell the server that we want binary/SGA/com-port ourselves. */
    int rc = 0;
    switch (cmd) {
    case DO:
        if (opt == TELOPT_BINARY || opt == TELOPT_SGA || opt == TELOPT_COMPORT)
            rc = telnet_send_negot(WILL, opt);
        else
            rc = telnet_send_negot(WONT, opt);
        break;
    case DONT:
        rc = telnet_send_negot(WONT, opt);
        break;
    case WILL:
        if (opt == TELOPT_BINARY || opt == TELOPT_SGA || opt == TELOPT_ECHO || opt == TELOPT_COMPORT)
            rc = telnet_send_negot(DO, opt);
        else
            rc = telnet_send_negot(DONT, opt);
        break;
    case WONT:
        rc = telnet_send_negot(DONT, opt);
        break;
    default:
        break;
    }
    if (rc < 0)
        log_errno(LOG_WARNING, "handle_negot: failed to send response for opt %u", (unsigned)opt);
}

/* Process COM-PORT sub-negotiation replies from the server. */
static void handle_subneg(const uint8_t *buf, size_t len)
{
    if (len < 2)
        return;
    if (buf[0] != TELOPT_COMPORT)
        return; /* We only care about COM-PORT. */

    uint8_t sub = buf[1];
    /* Server replies use sub-option + 100 (RFC 2217 §4.3). */
    switch (sub) {
    case CPC_SET_BAUDRATE + 100:
        if (len >= 6) {
            uint32_t baud = ((uint32_t)buf[2] << 24) | ((uint32_t)buf[3] << 16)
                          | ((uint32_t)buf[4] <<  8) |  (uint32_t)buf[5];
            log_message(LOG_DEBUG, "[RFC2217] server confirmed baud: %u", baud);
        }
        break;
    case CPC_SET_DATASIZE + 100:
        if (len >= 3)
            log_message(LOG_DEBUG, "[RFC2217] server confirmed data bits: %u", buf[2]);
        break;
    case CPC_SET_PARITY + 100:
        if (len >= 3)
            log_message(LOG_DEBUG, "[RFC2217] server confirmed parity: %u", buf[2]);
        break;
    case CPC_SET_STOPSIZE + 100:
        if (len >= 3)
            log_message(LOG_DEBUG, "[RFC2217] server confirmed stop bits: %u", buf[2]);
        break;
    default:
        break;
    }
}

/* ---------- Incoming stream parsers ---------- */

/* Flush the local output buffer to stdout. Returns 0 on success, -1 on error. */
static int flush_stdout_buf(uint8_t *out, size_t *pos)
{
    if (*pos == 0)
        return 0;
    int rc = write_all(STDOUT_FILENO, out, *pos);
    *pos = 0;
    return rc;
}

/* Add a byte to output buffer with optional LF→CR+LF translation.
 * Flushes if buffer is full. Returns 0 on success, -1 on flush error. */
static int add_to_output_buf(uint8_t *out, size_t *out_pos, size_t out_size, uint8_t b)
{
    /* Reserve space for up to 2 bytes (LF expands to CR+LF). */
    if (*out_pos + 2 > out_size && flush_stdout_buf(out, out_pos) != 0)
        return -1;
    if (b == '\n' && !g_no_crlf)
        out[(*out_pos)++] = '\r';
    out[(*out_pos)++] = b;
    return 0;
}

/* Serial mode: copy data to stdout with optional LF→CR+LF translation.
 * Translation is needed in raw terminal mode (OPOST off) so that LF moves
 * the cursor to column 0. Use -n to disable it for devices that send \r\n. */
static int process_serial_incoming(const uint8_t *buf, size_t n)
{
    uint8_t out[256];
    size_t  out_pos = 0;

    for (size_t i = 0; i < n; i++) {
        if (add_to_output_buf(out, &out_pos, sizeof(out), buf[i]) != 0)
            return -1;
    }
    return flush_stdout_buf(out, &out_pos);
}

/* Telnet mode: parse and strip Telnet commands; optional LF→CR+LF translation.
 * Data bytes are batched into a stack buffer to minimise write(2) calls.
 * Returns 0 on success, -1 if a write to stdout fails. */
static int process_incoming(const uint8_t *buf, size_t n)
{
    uint8_t out[256];
    size_t  out_pos = 0;

    for (size_t i = 0; i < n; i++) {
        uint8_t b = buf[i];
        switch (g_tn_state) {
        case TS_DATA:
            if (b == IAC) {
                if (flush_stdout_buf(out, &out_pos) != 0)
                    return -1;
                g_tn_state = TS_IAC;
            } else {
                if (add_to_output_buf(out, &out_pos, sizeof(out), b) != 0)
                    return -1;
            }
            break;

        case TS_IAC:
            if (b == IAC) {
                /* Escaped 0xFF becomes data. */
                if (add_to_output_buf(out, &out_pos, sizeof(out), b) != 0)
                    return -1;
                g_tn_state = TS_DATA;
            } else if (b == DO || b == DONT || b == WILL || b == WONT) {
                g_negot_cmd = b;
                g_tn_state = TS_NEGOT;
            } else if (b == SB) {
                g_sb_len = 0;
                g_tn_state = TS_SB;
            } else {
                /* Ignore other commands (NOP, BRK echo, etc.). */
                g_tn_state = TS_DATA;
            }
            break;

        case TS_NEGOT:
            handle_negot(g_negot_cmd, b);
            g_tn_state = TS_DATA;
            break;

        case TS_SB:
            if (b == IAC) {
                g_tn_state = TS_SB_IAC;
            } else {
                if (g_sb_len < sizeof(g_sb_buf)) {
                    g_sb_buf[g_sb_len++] = b;
                } else if (g_sb_len == sizeof(g_sb_buf)) {
                    /* Log once at the overflow boundary; increment so we don't repeat. */
                    log_message(LOG_WARNING, "SB payload exceeds %zu bytes, truncating",
                                sizeof(g_sb_buf));
                    g_sb_len++;
                }
                /* After overflow: continue silently discarding bytes (don't increment again). */
            }
            break;

        case TS_SB_IAC:
            if (b == SE) {
                /* Pass only the valid (non-overflowed) length (N1). */
                size_t valid_len = g_sb_len < sizeof(g_sb_buf) ? g_sb_len : sizeof(g_sb_buf);
                handle_subneg(g_sb_buf, valid_len);
                g_sb_len = 0;
                g_tn_state = TS_DATA;
            } else if (b == IAC) {
                /* Escaped IAC inside SB. */
                if (g_sb_len < sizeof(g_sb_buf))
                    g_sb_buf[g_sb_len++] = IAC;
                g_tn_state = TS_SB;
            } else {
                /* Non-standard sequence: leave SB. */
                g_tn_state = TS_DATA;
            }
            break;
        }
    }
    return flush_stdout_buf(out, &out_pos);
}

/* ---------- Power control (RTS/DTR) ---------- */

/* Record a change of the pair state.  An "off" followed within 5 s by an
 * "on" from the same source is what a reset pulse looks like from here,
 * whether it came from Ctrl+P, a raw TCP client or an RFC 2217 client's
 * SET-CONTROL sequence, so it is reported as one "reset" event. */
static void power_event(int on, const char *source)
{
    long long now = now_ms();
    const char *kind = on ? "on" : "off";

    if (on && strcmp(g_ev_kind, "off") == 0 && strcmp(g_ev_source, source) == 0 &&
        now - g_ev_ms < 5000)
        kind = "reset";
    g_ev_seq++;
    g_ev_kind   = kind;
    g_ev_source = source;
    g_ev_ms     = now;
}

/* Drive RTS and DTR together: on = asserted (device powered), off =
 * deasserted.  Shared by the connect path, Ctrl+P and the -w web
 * buttons, so every caller keeps g_power_on in step with the device.
 * `source` names who asked, for the event log on the web page. */
static int power_set(int on, const char *source)
{
    int was = g_power_on;

    if (g_fd < 0)
        return -1;
    if (g_serial_mode) {
        set_serial_signal_state(TIOCM_DTR, on);
        set_serial_signal_state(TIOCM_RTS, on);
    } else {
        if (comport_set_control(on ? CPC_CTRL_DTR_ON : CPC_CTRL_DTR_OFF) < 0 ||
            comport_set_control(on ? CPC_CTRL_RTS_ON : CPC_CTRL_RTS_OFF) < 0)
            return -1;
    }
    g_dtr_on = g_rts_on = g_power_on = on;
    if (was != on)
        power_event(on, source);
    return 0;
}

/* ---------- Break signal ---------- */

/* Send a serial break: tcsendbreak in serial mode, IAC BRK in Telnet mode.
 * Useful for stopping U-Boot autoboot and other bootloader interactions. */
static void send_break(void)
{
    if (g_fd < 0)
        return;
    if (g_serial_mode) {
        log_message(LOG_INFO, "[break] sending serial break");
        if (tcsendbreak(g_fd, 0) != 0)
            log_errno(LOG_WARNING, "tcsendbreak");
    } else {
        log_message(LOG_INFO, "[break] sending IAC BRK");
        uint8_t brk[2] = { IAC, BRK };
        if (write_all(g_fd, brk, 2) < 0)
            log_errno(LOG_WARNING, "send break: write failed");
    }
}

/* ---------- Reset pulse (RTS+DTR released for g_reset_ms) ---------- */

static void send_reset_pulse(const char *source)
{
    if (g_fd < 0)
        return;
    fputs("\r\n", stderr);
    log_message(LOG_INFO, "[reset] RTS+DTR off for %d ms", g_reset_ms);
    (void)power_set(0, source);
    sleep_us((unsigned int)g_reset_ms * 1000);
    (void)power_set(1, source);
    log_message(LOG_INFO, "[reset] done");
}

/* ---------- stdin EOF ---------- */

/* Called when stdin reports EOF or a hangup.  Returns 1 when the session
 * should end, 0 when the I/O loop should carry on without stdin.
 *
 * A drained pipe whose writer has exited (`echo cmd | vectis ...`)
 * reports POLLHUP on every poll() call; unless the fd is taken out of
 * the poll set the loop spins at 100% CPU.  This is the same bug class
 * as the stdin/UART EOF fixes in vectis-bootrom.c, and the remedy is the same:
 * stop polling the dead descriptor. */
static int on_stdin_eof(void)
{
    if (g_interactive) {
        /* The controlling terminal went away — nobody is left to drive
         * the session, so end it (and any -r reconnect loop). */
        log_message(LOG_INFO, "stdin closed, exiting");
        g_quit = 1;
        return 1;
    }
    /* Scripted use: the input side is done, but the device's reply may
     * still be on its way.  Keep forwarding device output until the
     * transport closes or a signal arrives. */
    g_watch_stdin = 0;
    log_message(LOG_INFO, "stdin closed, continuing to forward device output");
    return 0;
}

/* ---------- Built-in web server (-w) ---------- */

/* A deliberately small HTTP/1.1 server that lives in the main poll loop:
 * one static page (GET /) plus the JSON API the page calls.
 *
 *   GET  /               -> the page
 *   GET  /status         -> {"link":..,"power":..,"rts":..,"dtr":..,...}
 *   GET  /power/enable   -> assert RTS+DTR   (what the connect path does)
 *   GET  /power/disable  -> deassert RTS+DTR
 *   GET  /power/reset    -> RTS+DTR pulse    (what Ctrl+P does)
 *
 * The three /power/ commands accept GET and POST alike (curl, bookmarks,
 * the page's own fetch) and always answer with the full status JSON,
 * whoever the client is.  The page itself is served only for a bare "/":
 * any other path, and "/" with a query string, is not the page.
 *
 * The buttons act on the very same transport the terminal session uses,
 * so the web page and the keyboard drive the device side by side.
 * Requests are answered synchronously and the connection is closed after
 * each reply; a client that stalls is dropped after WEB_CLIENT_TIMEOUT_MS.
 * There is no authentication: treat the port like the -l listener and
 * bind it to a trusted interface (-w 127.0.0.1:PORT). */

#define WEB_MAX_CLIENTS        8
#define WEB_NPOLL              (1 + WEB_MAX_CLIENTS)  /* poll slots: listener + clients */
#define WEB_REQ_MAX            4096   /* request (headers + body) we are willing to buffer */
#define WEB_CLIENT_TIMEOUT_MS  5000   /* drop a client that never completes its request */
#define WEB_SEND_TIMEOUT_MS    3000   /* give up on a client that will not read the reply */

struct web_client {
    int       fd;
    size_t    len;
    long long started_ms;
    char      peer[INET_ADDRSTRLEN];
    char      buf[WEB_REQ_MAX];
};

static int g_web_listen_fd = -1;
static int  g_srv_fd;               /* -l client socket, defined with the listener below */
static char g_srv_peer[INET_ADDRSTRLEN];
static struct web_client g_web_clients[WEB_MAX_CLIENTS];
static char g_web_target[300] = "";  /* device path or host:port shown on the page */

#define WEB_TEXT(s) (s), (sizeof(s) - 1)

/* The page: HTML, CSS and JS in one document, written with single quotes
 * only so it embeds as plain string lines (one source line per HTML line,
 * "\n" terminated).  Keep the heading "Vectis Web Server" at the top. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverlength-strings"  /* one literal > 4095 bytes; fine for GCC and Clang */
static const char web_page[] =
    "<!doctype html>\n"
    "<html lang='en'>\n"
    "<head>\n"
    "<meta charset='utf-8'>\n"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>\n"
    "<title>Vectis Web Server</title>\n"
    "<style>\n"
    ":root{--panel:#2c2119;--panel2:#3d2d22;--brass:#c9a24d;--brass2:#8c6a2a;--amber:#ffb347;--ember:#ff6a1a;--ink:#e8d9b8;--dim:#8a7455;--red:#c8452c;--green:#5ad38f}\n"
    "*{box-sizing:border-box}\n"
    "body{margin:0;min-height:100vh;background:radial-gradient(ellipse at 50% 0,#3b2c22 0,#1a1512 55%,#0f0b09 100%);color:var(--ink);font-family:Georgia,'Times New Roman',serif;display:flex;flex-direction:column;align-items:center;padding:28px 12px}\n"
    "h1{font-size:clamp(22px,4.5vw,40px);letter-spacing:.2em;text-transform:uppercase;margin:0 0 6px;text-align:center;color:var(--brass);text-shadow:0 1px 0 #f3d78a,0 -1px 0 #3d2a0c,0 0 22px rgba(201,162,77,.35)}\n"
    ".sub{font-family:'Courier New',monospace;font-size:12px;color:var(--dim);letter-spacing:.3em;text-transform:uppercase;margin-bottom:24px;text-align:center}\n"
    ".panel{position:relative;width:min(760px,100%);background:linear-gradient(180deg,var(--panel2),var(--panel));border:3px solid var(--brass2);border-radius:16px;box-shadow:inset 0 0 0 2px #1a1210,inset 0 0 50px rgba(0,0,0,.6),0 24px 48px rgba(0,0,0,.65);padding:30px 24px 24px}\n"
    ".rivet{position:absolute;width:14px;height:14px;border-radius:50%;background:radial-gradient(circle at 35% 35%,#f1d38b,#8c6a2a 60%,#3a2a10);box-shadow:0 1px 2px #000,inset 0 -1px 1px rgba(0,0,0,.5)}\n"
    ".rivet.tl{top:8px;left:8px}.rivet.tr{top:8px;right:8px}.rivet.bl{bottom:8px;left:8px}.rivet.br{bottom:8px;right:8px}\n"
    ".plate{font-family:'Courier New',monospace;font-size:12px;letter-spacing:.2em;text-transform:uppercase;color:#2a1c10;background:linear-gradient(180deg,#e2c377,#a88237);border:1px solid #5b4416;border-radius:4px;padding:4px 10px;display:inline-block;box-shadow:inset 0 1px 0 #fbe6a8,0 1px 2px #000}\n"
    ".top{display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:10px}\n"
    ".lamps{display:flex;gap:18px;flex-wrap:wrap;margin-top:20px;padding:14px 16px;border:1px solid #574123;border-radius:10px;background:rgba(0,0,0,.25);box-shadow:inset 0 2px 8px rgba(0,0,0,.6)}\n"
    ".lamp{display:flex;align-items:center;gap:8px;font-family:'Courier New',monospace;font-size:12px;letter-spacing:.15em;text-transform:uppercase;color:var(--dim)}\n"
    ".bulb{width:18px;height:18px;border-radius:50%;border:2px solid #6e5320;background:#2a2118;box-shadow:inset 0 0 6px #000;transition:background .3s,box-shadow .3s}\n"
    ".bulb.on{background:radial-gradient(circle at 40% 35%,#fff3c4,var(--amber) 45%,#a24d0a);box-shadow:0 0 14px var(--amber),0 0 30px rgba(255,179,71,.45)}\n"
    ".bulb.ok{background:radial-gradient(circle at 40% 35%,#d8ffe8,var(--green) 45%,#155b34);box-shadow:0 0 14px var(--green)}\n"
    ".bulb.bad{background:radial-gradient(circle at 40% 35%,#ffd0c4,var(--red) 45%,#5a1a0e);box-shadow:0 0 14px var(--red);animation:blink 1.2s infinite}\n"
    "@keyframes blink{50%{box-shadow:0 0 2px var(--red)}}\n"
    ".mid{display:flex;gap:22px;align-items:center;justify-content:space-between;flex-wrap:wrap;margin-top:22px}\n"
    ".gauge{position:relative;width:220px;height:118px;overflow:hidden;flex:none;margin:0 auto}\n"
    ".dial{position:absolute;left:0;top:0;width:220px;height:220px;border-radius:50%;border:6px solid var(--brass2);background:radial-gradient(circle at 50% 50%,#f4e6c3 0,#e6d3a3 55%,#c9b27a 100%);box-shadow:inset 0 0 18px rgba(0,0,0,.5)}\n"
    ".tick{position:absolute;left:50%;top:110px;width:2px;height:104px;margin-left:-1px;margin-top:-104px;transform-origin:50% 100%}\n"
    ".tick:after{content:'';position:absolute;top:8px;left:0;width:2px;height:12px;background:#3a2a10}\n"
    ".needle{position:absolute;left:50%;top:110px;width:4px;height:98px;margin-left:-2px;margin-top:-98px;background:linear-gradient(180deg,#d33 0,#7a1a10 70%,#2a1c10);transform-origin:50% 100%;transform:rotate(-88deg);transition:transform 1.1s cubic-bezier(.3,1.6,.4,1);border-radius:2px;box-shadow:0 0 4px rgba(0,0,0,.6)}\n"
    ".hub{position:absolute;left:50%;top:110px;width:26px;height:26px;margin:-13px 0 0 -13px;border-radius:50%;background:radial-gradient(circle at 35% 35%,#f1d38b,#8c6a2a 60%,#3a2a10);box-shadow:0 2px 4px #000}\n"
    ".lbl{position:absolute;bottom:8px;font-family:'Courier New',monospace;font-size:11px;letter-spacing:.2em;color:#3a2a10;font-weight:bold}\n"
    ".lbl.l{left:28px}.lbl.r{right:28px}\n"
    ".readout{flex:1;min-width:230px;font-family:'Courier New',monospace}\n"
    ".state{font-size:clamp(18px,3vw,26px);letter-spacing:.18em;text-transform:uppercase;color:var(--amber);text-shadow:0 0 12px rgba(255,179,71,.5);padding:14px 16px;border:2px solid #574123;border-radius:8px;background:#120d0a;box-shadow:inset 0 0 20px rgba(0,0,0,.8);transition:color .3s}\n"
    ".state.off{color:var(--dim);text-shadow:none}\n"
    ".state.bad{color:var(--red);text-shadow:0 0 12px rgba(200,69,44,.5)}\n"
    ".meta{margin-top:10px;font-size:12px;color:var(--dim);letter-spacing:.08em;line-height:1.7;word-break:break-all}\n"
    ".btns{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;margin-top:24px}\n"
    ".btn{position:relative;border:0;cursor:pointer;font-family:Georgia,'Times New Roman',serif;font-size:15px;letter-spacing:.22em;text-transform:uppercase;color:#2a1c10;padding:18px 8px 14px;border-radius:12px;background:linear-gradient(180deg,#e9cd84,#b08b3c 55%,#7c5c1f);box-shadow:0 7px 0 #4a3510,0 12px 20px rgba(0,0,0,.6),inset 0 1px 0 #fff1c0;transition:transform .1s,box-shadow .1s,filter .2s}\n"
    ".btn:hover{filter:brightness(1.07)}\n"
    ".btn.pressed{transform:translateY(6px);box-shadow:0 1px 0 #4a3510,0 3px 6px rgba(0,0,0,.6),inset 0 1px 0 #fff1c0}\n"
    ".btn .ind{display:block;margin:10px auto 0;width:12px;height:12px;border-radius:50%;background:#3b2a12;border:1px solid #241806;box-shadow:inset 0 0 4px #000;transition:background .3s,box-shadow .3s}\n"
    ".btn.engaged{background:linear-gradient(180deg,#f6dc95,#c99d45 55%,#8a6621);box-shadow:0 2px 0 #4a3510,0 4px 8px rgba(0,0,0,.6),inset 0 0 14px rgba(255,180,60,.55);transform:translateY(4px)}\n"
    ".btn.engaged .ind{background:radial-gradient(circle at 40% 35%,#fff3c4,var(--amber) 45%,#a24d0a);box-shadow:0 0 10px var(--amber)}\n"
    ".btn.reset.engaged .ind{background:radial-gradient(circle at 40% 35%,#ffe1d0,var(--ember) 45%,#7a2a05);box-shadow:0 0 10px var(--ember);animation:flick .35s infinite}\n"
    "@keyframes flick{50%{opacity:.25}}\n"
    ".btn:disabled{cursor:not-allowed;filter:grayscale(.7) brightness(.6)}\n"
    ".btn .gear{position:absolute;right:10px;top:8px;font-size:16px;line-height:1;opacity:0;transition:opacity .2s}\n"
    ".btn.reset.engaged .gear{opacity:1;animation:spin 1s linear infinite}\n"
    "@keyframes spin{to{transform:rotate(360deg)}}\n"
    ".log{font-family:'Courier New',monospace;font-size:12px;color:var(--dim);margin-top:18px;min-height:16px;letter-spacing:.08em}\n"
    ".foot{margin-top:18px;font-family:'Courier New',monospace;font-size:11px;color:#5e4d36;letter-spacing:.2em;text-transform:uppercase;text-align:center}\n"
    "</style>\n"
    "</head>\n"
    "<body>\n"
    "<h1>Vectis Web Server</h1>\n"
    "<div class='sub'>RTS / DTR power control &middot; OpenIPC UART 38x38</div>\n"
    "<div class='panel'>\n"
    "<span class='rivet tl'></span><span class='rivet tr'></span><span class='rivet bl'></span><span class='rivet br'></span>\n"
    "<div class='top'><span class='plate'>Power console</span><span class='plate' id='ver'>vectis</span></div>\n"
    "<div class='lamps'>\n"
    "<div class='lamp'><span class='bulb' id='l-link'></span><span id='t-link'>Link</span></div>\n"
    "<div class='lamp'><span class='bulb' id='l-lines'></span>RTS+DTR</div>\n"
    "</div>\n"
    "<div class='mid'>\n"
    "<div class='gauge'><div class='dial'></div>\n"
    "<div class='tick' style='transform:rotate(-80deg)'></div><div class='tick' style='transform:rotate(-40deg)'></div><div class='tick'></div><div class='tick' style='transform:rotate(40deg)'></div><div class='tick' style='transform:rotate(80deg)'></div>\n"
    "<span class='lbl l'>OFF</span><span class='lbl r'>ON</span>\n"
    "<div class='needle' id='needle'></div><div class='hub'></div></div>\n"
    "<div class='readout'><div class='state off' id='state'>Stand by</div><div class='meta' id='meta'>Reading the console&hellip;</div></div>\n"
    "</div>\n"
    "<div class='btns'>\n"
    "<button class='btn' id='b-on' onclick='act(this)' disabled>Enable<span class='ind'></span></button>\n"
    "<button class='btn' id='b-off' onclick='act(this)' disabled>Disable<span class='ind'></span></button>\n"
    "<button class='btn reset' id='b-reset' onclick='act(this)' disabled>Reset<span class='gear'>&#9881;</span><span class='ind'></span></button>\n"
    "</div>\n"
    "<div class='log' id='log'>&nbsp;</div>\n"
    "</div>\n"
    "<div class='foot'>Vectis &middot; OpenIPC &middot; MIT License</div>\n"
    "<script>\n"
    "var busy=false,lastSeq=-1;\n"
    "function $(i){return document.getElementById(i)}\n"
    "function bulb(id,c){$(id).className='bulb'+(c?' '+c:'')}\n"
    "function esc(t){return String(t).replace(/[&<>]/g,function(c){return {'&':'&amp;','<':'&lt;','>':'&gt;'}[c]})}\n"
    "function render(s){\n"
    " var online=s.link==='online',on=s.power==='on';\n"
    " bulb('l-link',online?'ok':'bad');$('t-link').textContent=online?'Link':'No link';\n"
    " bulb('l-lines',online&&(s.rts<0||s.dtr<0?on:s.rts===1&&s.dtr===1)?'on':'');\n"
    " var st=$('state');st.textContent=online?(on?'Power enabled':'Power disabled'):'Link offline';st.className='state'+(online?(on?'':' off'):' bad');\n"
    " $('meta').innerHTML='TARGET&nbsp; '+esc(s.target)+' ['+esc(s.mode)+']<br>PULSE&nbsp;&nbsp; '+esc(s.reset_ms)+' ms &middot; RTS+DTR'+(s.tcp_client?'<br>CLIENT&nbsp; '+esc(s.tcp_client):'');\n"
    " $('ver').textContent='vectis '+s.version;\n"
    " $('needle').style.transform='rotate('+(online?(on?80:-80):-88)+'deg)';\n"
    " $('b-on').classList.toggle('engaged',online&&on);$('b-off').classList.toggle('engaged',online&&!on);\n"
    " var d=!online||busy;$('b-on').disabled=d;$('b-off').disabled=d;$('b-reset').disabled=d;\n"
    " remote(s.event);\n"
    "}\n"
    "function remote(e){\n"
    " if(!e){return}\n"
    " if(lastSeq<0){lastSeq=e.seq;return}\n"
    " if(e.seq===lastSeq){return}\n"
    " lastSeq=e.seq;\n"
    " if(e.ago_ms>15000||busy){return}\n"
    " var who=e.source==='web'?'another browser':e.source;\n"
    " $('log').textContent='> power '+e.kind+' by '+who;\n"
    " if(e.kind==='reset'){flashReset()}\n"
    "}\n"
    "function flashReset(){\n"
    " var b=$('b-reset');b.classList.add('engaged');$('needle').style.transform='rotate(-80deg)';\n"
    " $('state').textContent='Resetting';$('state').className='state';\n"
    " setTimeout(function(){b.classList.remove('engaged');if(!busy)poll()},900);\n"
    "}\n"
    "function offline(){render({link:'offline',power:'off',rts:-1,dtr:-1,target:'?',mode:'?',reset_ms:'?',version:'?'})}\n"
    "function poll(){if(busy)return;fetch('/status',{cache:'no-store'}).then(function(r){return r.json()}).then(render).catch(offline)}\n"
    "function finish(btn){busy=false;setTimeout(function(){btn.classList.remove('pressed')},180);if(btn.id==='b-reset')btn.classList.remove('engaged')}\n"
    "function act(btn){\n"
    " if(busy)return;busy=true;\n"
    " var id=btn.id,path=id==='b-on'?'/power/enable':id==='b-off'?'/power/disable':'/power/reset';\n"
    " var label=id==='b-on'?'Enable':id==='b-off'?'Disable':'Reset';\n"
    " btn.classList.add('pressed');\n"
    " ['b-on','b-off','b-reset'].forEach(function(i){$(i).disabled=true});\n"
    " if(id==='b-reset'){btn.classList.add('engaged');$('needle').style.transform='rotate(-80deg)';$('state').textContent='Resetting';$('state').className='state'}\n"
    " $('log').textContent='> '+label+' ... engaging';\n"
    " var t0=Date.now(),hold=function(f){setTimeout(f,id==='b-reset'?Math.max(0,900-(Date.now()-t0)):0)};\n"
    " fetch(path,{method:'POST'}).then(function(r){return r.json().then(function(j){return {ok:r.ok,j:j}})}).then(function(x){hold(function(){\n"
    "  finish(btn);\n"
    "  if(x.ok){if(x.j.event){lastSeq=x.j.event.seq}render(x.j);$('log').textContent='> '+label+' ... done'}else{$('log').textContent='> '+label+' ... refused: '+(x.j.error||'error');poll()}\n"
    " })}).catch(function(){hold(function(){finish(btn);$('log').textContent='> '+label+' ... no answer from server';offline()})});\n"
    "}\n"
    "setInterval(poll,2000);poll();\n"
    "</script>\n"
    "</body>\n"
    "</html>\n";
#pragma GCC diagnostic pop

static void web_client_close(struct web_client *c)
{
    if (c->fd >= 0) {
        close(c->fd);
        c->fd = -1;
    }
    c->len = 0;
}

static void web_shutdown(void)
{
    for (int i = 0; i < WEB_MAX_CLIENTS; i++)
        web_client_close(&g_web_clients[i]);
    if (g_web_listen_fd >= 0) {
        close(g_web_listen_fd);
        g_web_listen_fd = -1;
    }
}

/* Parse a "PORT" or "ADDR:PORT" argument (-w, -l).  Returns 0 on success. */
static int parse_bind(const char *arg, char *addr, size_t addr_size, int *port)
{
    const char *colon = strrchr(arg, ':');
    const char *pstr = arg;
    char *endp;
    long v;

    addr[0] = '\0';
    if (colon) {
        size_t alen = (size_t)(colon - arg);
        if (alen == 0 || alen >= addr_size)
            return -1;
        memcpy(addr, arg, alen);
        addr[alen] = '\0';
        pstr = colon + 1;
    }
    errno = 0;
    v = strtol(pstr, &endp, 10);
    if (errno != 0 || endp == pstr || *endp != '\0' || v < 1 || v > 65535)
        return -1;
    *port = (int)v;
    return 0;
}

/* Open a non-blocking IPv4 listening socket; `what` names it in logs. */
static int tcp_listen(const char *addr, int port, const char *what)
{
    struct sockaddr_in sa;
    int one = 1;
    int fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);

    if (fd < 0) {
        log_errno(LOG_ERR, "socket(%s)", what);
        return -1;
    }
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one) != 0)
        log_errno(LOG_WARNING, "setsockopt(%s SO_REUSEADDR)", what);

    memset(&sa, 0, sizeof sa);
    sa.sin_family = AF_INET;
    sa.sin_port   = htons((uint16_t)port);
    if (addr[0] == '\0') {
        sa.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (inet_pton(AF_INET, addr, &sa.sin_addr) != 1) {
        log_message(LOG_ERR, "Invalid %s bind address: %s (IPv4 dotted quad expected)", what, addr);
        close(fd);
        return -1;
    }
    if (bind(fd, (struct sockaddr *)&sa, sizeof sa) != 0) {
        log_errno(LOG_ERR, "bind(%s %s:%d)", what, addr[0] ? addr : "0.0.0.0", port);
        close(fd);
        return -1;
    }
    if (listen(fd, 8) != 0) {
        log_errno(LOG_ERR, "listen(%s)", what);
        close(fd);
        return -1;
    }
    return fd;
}

static int web_listen(const char *addr, int port)
{
    int fd = tcp_listen(addr, port, "web");

    if (fd < 0)
        return -1;
    for (int i = 0; i < WEB_MAX_CLIENTS; i++) {
        g_web_clients[i].fd  = -1;
        g_web_clients[i].len = 0;
    }
    g_web_listen_fd = fd;
    log_message(LOG_INFO, "Web server ready on http://%s:%d/", addr[0] ? addr : "0.0.0.0", port);
    return 0;
}

/* Write everything to a non-blocking socket, bounded by WEB_SEND_TIMEOUT_MS
 * so a stalled peer cannot hold up the poll loop.  Shared by the web
 * server and the -l listener. */
static int tcp_send_all(int fd, const char *data, size_t len)
{
    long long deadline = now_ms() + WEB_SEND_TIMEOUT_MS;

    while (len > 0) {
        ssize_t k = send(fd, data, len, MSG_NOSIGNAL);
        if (k < 0) {
            if (errno == EINTR)
                continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                long long left = deadline - now_ms();
                struct pollfd p = { .fd = fd, .events = POLLOUT, .revents = 0 };
                if (left <= 0)
                    return -1;
                if (poll(&p, 1, (int)left) < 0 && errno != EINTR)
                    return -1;
                continue;
            }
            return -1;
        }
        data += k;
        len  -= (size_t)k;
    }
    return 0;
}

static void web_reply(struct web_client *c, int code, const char *reason,
                      const char *ctype, const char *body, size_t body_len,
                      int head_only)
{
    static char out[sizeof(web_page) + 1024];
    int hl = snprintf(out, sizeof out,
                      "HTTP/1.1 %d %s\r\n"
                      "Content-Type: %s\r\n"
                      "Content-Length: %zu\r\n"
                      "Cache-Control: no-store\r\n"
                      "Connection: close\r\n"
                      "\r\n",
                      code, reason, ctype, body_len);
    size_t total;

    if (hl < 0 || (size_t)hl >= sizeof out)
        return;
    total = (size_t)hl;
    if (!head_only && body_len > 0) {
        if (body_len > sizeof out - total)
            body_len = sizeof out - total;
        memcpy(out + total, body, body_len);
        total += body_len;
    }
    (void)tcp_send_all(c->fd, out, total);
}

static void json_escape(const char *in, char *out, size_t cap)
{
    size_t o = 0;

    for (; *in != '\0' && o + 7 < cap; in++) {
        unsigned char ch = (unsigned char)*in;
        if (ch == '"' || ch == '\\') {
            out[o++] = '\\';
            out[o++] = (char)ch;
        } else if (ch < 0x20) {
            o += (size_t)snprintf(out + o, cap - o, "\\u%04x", ch);
        } else {
            out[o++] = (char)ch;
        }
    }
    out[o] = '\0';
}

/* Render the state the page shows.  rts/dtr are the real modem bits in
 * -u mode (-1 when the ioctl is unavailable or in RFC 2217 mode);
 * "power" is the last state this process commanded. */
static size_t web_status_json(char *out, size_t cap)
{
    char target[sizeof(g_web_target) * 6];
    int rts = -1, dtr = -1;
    int n;

    json_escape(g_web_target, target, sizeof target);
    if (g_serial_mode && g_fd >= 0) {
        int st = 0;
        if (ioctl(g_fd, TIOCMGET, &st) == 0) {
            rts = (st & TIOCM_RTS) ? 1 : 0;
            dtr = (st & TIOCM_DTR) ? 1 : 0;
        }
    }
    char event[128];

    if (g_ev_seq == 0)
        snprintf(event, sizeof event, "null");
    else
        snprintf(event, sizeof event,
                 "{\"seq\":%u,\"kind\":\"%s\",\"source\":\"%s\",\"ago_ms\":%lld}",
                 (unsigned)g_ev_seq, g_ev_kind, g_ev_source, now_ms() - g_ev_ms);
    n = snprintf(out, cap,
                 "{\"link\":\"%s\",\"mode\":\"%s\",\"target\":\"%s\","
                 "\"power\":\"%s\",\"rts\":%d,\"dtr\":%d,"
                 "\"reset_ms\":%d,\"tcp_client\":%s%s%s,\"event\":%s,\"version\":\"%s\"}",
                 g_fd >= 0 ? "online" : "offline",
                 g_serial_mode ? "serial" : "rfc2217",
                 target, g_power_on ? "on" : "off", rts, dtr,
                 g_reset_ms,
                 g_srv_fd >= 0 ? "\"" : "null", g_srv_fd >= 0 ? g_srv_peer : "",
                 g_srv_fd >= 0 ? "\"" : "",
                 event, PROGRAM_VERSION);
    if (n < 0)
        return 0;
    return (size_t)n < cap ? (size_t)n : cap - 1;
}

/* Perform the /power/<what> command: enable, disable or reset, and reply
 * with the full status JSON. */
static void web_power_action(struct web_client *c, const char *what)
{
    const char *action;
    char json[2304];
    size_t n;
    int rc = 0;

    if (strcmp(what, "enable") == 0)
        action = "on";
    else if (strcmp(what, "disable") == 0)
        action = "off";
    else if (strcmp(what, "reset") == 0)
        action = "reset";
    else {
        web_reply(c, 404, "Not Found", "application/json",
                  WEB_TEXT("{\"error\":\"unknown command; use /power/enable, /power/disable or /power/reset\"}"), 0);
        return;
    }
    if (g_fd < 0) {
        web_reply(c, 503, "Service Unavailable", "application/json",
                  WEB_TEXT("{\"error\":\"link offline\"}"), 0);
        return;
    }
    log_message(LOG_INFO, "[web] %s: power %s", c->peer, action);
    if (action[0] == 'r')
        send_reset_pulse("web");
    else
        rc = power_set(action[1] == 'n', "web");
    if (rc != 0) {
        web_reply(c, 500, "Internal Server Error", "application/json",
                  WEB_TEXT("{\"error\":\"control command failed\"}"), 0);
        return;
    }
    n = web_status_json(json, sizeof json);
    web_reply(c, 200, "OK", "application/json", json, n, 0);
}

static void web_process_request(struct web_client *c)
{
    char method[8], path[256], json[2304];
    char *q;
    int is_get, is_head, is_post;
    size_t n;

    c->buf[c->len] = '\0';
    if (sscanf(c->buf, "%7s %255s", method, path) != 2) {
        web_reply(c, 400, "Bad Request", "text/plain", WEB_TEXT("bad request\n"), 0);
        return;
    }
    q = strchr(path, '?');   /* query strings are ignored on the command paths */
    if (q)
        *q = '\0';
    is_get  = strcmp(method, "GET") == 0;
    is_head = strcmp(method, "HEAD") == 0;
    is_post = strcmp(method, "POST") == 0;

    if (strcmp(path, "/") == 0 && q == NULL) {
        /* The page: a bare "/" only, nothing appended. */
        if (!is_get && !is_head) {
            web_reply(c, 405, "Method Not Allowed", "text/plain", WEB_TEXT("GET only\n"), 0);
            return;
        }
        web_reply(c, 200, "OK", "text/html; charset=utf-8", web_page, sizeof(web_page) - 1, is_head);
        return;
    }
    if (strcmp(path, "/status") == 0) {
        if (!is_get && !is_head) {
            web_reply(c, 405, "Method Not Allowed", "application/json",
                      WEB_TEXT("{\"error\":\"GET only\"}"), 0);
            return;
        }
        n = web_status_json(json, sizeof json);
        web_reply(c, 200, "OK", "application/json", json, n, is_head);
        return;
    }
    if (strncmp(path, "/power/", 7) == 0) {
        if (!is_get && !is_post) {
            web_reply(c, 405, "Method Not Allowed", "application/json",
                      WEB_TEXT("{\"error\":\"GET or POST\"}"), 0);
            return;
        }
        web_power_action(c, path + 7);
        return;
    }
    web_reply(c, 404, "Not Found", "text/plain", WEB_TEXT("not found\n"), 0);
}

static void web_client_read(struct web_client *c)
{
    ssize_t n;
    char *hdr_end;
    size_t hdr_len, body_len = 0;
    const char *cl;
    char saved;

    n = read(c->fd, c->buf + c->len, WEB_REQ_MAX - 1 - c->len);
    if (n < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
            return;
        web_client_close(c);
        return;
    }
    if (n == 0) {
        web_client_close(c);
        return;
    }
    c->len += (size_t)n;
    c->buf[c->len] = '\0';

    hdr_end = strstr(c->buf, "\r\n\r\n");
    if (hdr_end != NULL) {
        hdr_len = (size_t)(hdr_end - c->buf) + 4;
    } else if ((hdr_end = strstr(c->buf, "\n\n")) != NULL) {
        hdr_len = (size_t)(hdr_end - c->buf) + 2;
    } else {
        if (c->len >= WEB_REQ_MAX - 1) {
            web_reply(c, 431, "Request Header Fields Too Large", "text/plain",
                      WEB_TEXT("headers too large\n"), 0);
            web_client_close(c);
        }
        return;  /* wait for the rest of the headers */
    }

    /* Accept a body (browsers send Content-Length: 0 on POST) but only
     * as much as fits in the buffer; anything larger is refused. */
    saved = c->buf[hdr_len];
    c->buf[hdr_len] = '\0';
    cl = strcasestr(c->buf, "\ncontent-length:");
    c->buf[hdr_len] = saved;
    if (cl != NULL) {
        long v = strtol(cl + 16, NULL, 10);
        if (v < 0 || (size_t)v > WEB_REQ_MAX - 1 - hdr_len) {
            web_reply(c, 413, "Payload Too Large", "text/plain",
                      WEB_TEXT("body too large\n"), 0);
            web_client_close(c);
            return;
        }
        body_len = (size_t)v;
    }
    if (c->len < hdr_len + body_len)
        return;  /* wait for the body */

    web_process_request(c);
    web_client_close(c);
}

static void web_accept(void)
{
    struct sockaddr_in peer;
    socklen_t pl = sizeof peer;
    struct web_client *c = NULL;
    int cfd = accept4(g_web_listen_fd, (struct sockaddr *)&peer, &pl,
                      SOCK_NONBLOCK | SOCK_CLOEXEC);

    if (cfd < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR &&
            errno != ECONNABORTED)
            log_errno(LOG_WARNING, "accept(web)");
        return;
    }
    for (int i = 0; i < WEB_MAX_CLIENTS; i++) {
        if (g_web_clients[i].fd < 0) {
            c = &g_web_clients[i];
            break;
        }
    }
    if (c == NULL) {
        static const char busy[] =
            "HTTP/1.1 503 Service Unavailable\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
        (void)send(cfd, busy, sizeof busy - 1, MSG_NOSIGNAL | MSG_DONTWAIT);
        close(cfd);
        return;
    }
    c->fd         = cfd;
    c->len        = 0;
    c->started_ms = now_ms();
    if (inet_ntop(AF_INET, &peer.sin_addr, c->peer, sizeof c->peer) == NULL)
        snprintf(c->peer, sizeof c->peer, "?");
}

/* Fill poll slots for the listener and every client slot (fd = -1 where
 * unused, which poll() skips).  Returns the number of slots filled: 0
 * when -w is off, WEB_NPOLL otherwise. */
static int web_fill_pollfds(struct pollfd *p)
{
    if (g_web_listen_fd < 0)
        return 0;
    p[0].fd      = g_web_listen_fd;
    p[0].events  = POLLIN;
    p[0].revents = 0;
    for (int i = 0; i < WEB_MAX_CLIENTS; i++) {
        p[1 + i].fd      = g_web_clients[i].fd;
        p[1 + i].events  = POLLIN;
        p[1 + i].revents = 0;
    }
    return WEB_NPOLL;
}

/* Service the slots filled by web_fill_pollfds() once poll() returned,
 * and drop clients that have been idle for too long. */
static void web_handle_pollfds(const struct pollfd *p, int n)
{
    long long now;

    if (n <= 0)
        return;
    if (p[0].revents & POLLIN)
        web_accept();
    now = now_ms();
    for (int i = 0; i < WEB_MAX_CLIENTS; i++) {
        struct web_client *c = &g_web_clients[i];
        if (c->fd < 0 || p[1 + i].fd != c->fd)
            continue;  /* empty slot, or accepted only just now */
        if (p[1 + i].revents & POLLNVAL)
            web_client_close(c);
        else if (p[1 + i].revents & (POLLIN | POLLHUP | POLLERR))
            web_client_read(c);
        if (c->fd >= 0 && now - c->started_ms > WEB_CLIENT_TIMEOUT_MS)
            web_client_close(c);
    }
}

/* ---------- TCP listener (-l): serve the local UART to one client ---------- */

/* With -l the direct-serial session also acts as a small RFC 2217 server,
 * so that telnet, another vectis (-h HOST -p PORT), pyserial's rfc2217://
 * transport or plain nc/socat reach the same UART while the local
 * terminal and the -w web console keep working.  The design follows the
 * original vectis-bootrom server: one client at a time (a new connection replaces the
 * current one); a connection starts in legacy raw mode (bytes pass
 * through, LF becomes CR, Ctrl+P fires the reset pulse) and flips into
 * Telnet/RFC 2217 mode for the rest of the connection as soon as the
 * client sends an IAC byte.  In Telnet mode the data path is binary safe
 * (IAC escaped both ways) and the client can set the baud rate, data and
 * stop bits and parity, drive DTR/RTS, purge the queues and send a break;
 * everything is applied to the local port.  After a link loss the port
 * is reopened with the command-line settings again. */

#define SRV_NPOLL         2     /* poll slots: listener + client */
#define SRV_SB_MAX        64    /* longest sub-negotiation we keep */
#define CPC_SIGNATURE     0
#define CPC_SERVER_OFFSET 100   /* server replies use sub-option + 100 */

enum srv_state { SS_DATA, SS_IAC, SS_NEG, SS_SB_OPT, SS_SB_DATA, SS_SB_IAC };

static int  g_srv_listen_fd = -1;
static int  g_srv_fd        = -1;   /* the connected client, or -1 */
static char g_srv_peer[INET_ADDRSTRLEN] = "";   /* tentative definitions above */
static struct {
    int telnet;                     /* 0 = legacy raw, 1 = Telnet/RFC 2217 */
    enum srv_state state;
    uint8_t neg_cmd;
    uint8_t sb_opt;
    uint8_t sb_buf[SRV_SB_MAX];
    size_t  sb_len;
    int binary_local, binary_remote;   /* WILL state on our / the client's side */
    int sga_local, sga_remote;
    int comport_local, comport_remote;
} g_srv;

static void srv_close_client(const char *why)
{
    if (g_srv_fd >= 0) {
        close(g_srv_fd);
        g_srv_fd = -1;
        log_message(LOG_INFO, "[tcp] %s: %s", g_srv_peer, why);
    }
    memset(&g_srv, 0, sizeof g_srv);
}

static void srv_shutdown(void)
{
    srv_close_client("closed");
    if (g_srv_listen_fd >= 0) {
        close(g_srv_listen_fd);
        g_srv_listen_fd = -1;
    }
}

static void srv_send(const uint8_t *data, size_t len)
{
    if (g_srv_fd < 0 || len == 0)
        return;
    if (tcp_send_all(g_srv_fd, (const char *)data, len) != 0)
        srv_close_client("write failed, client dropped");
}

static void srv_send_neg(uint8_t cmd, uint8_t opt)
{
    uint8_t b[3] = { IAC, cmd, opt };
    srv_send(b, sizeof b);
}

/* IAC SB COM-PORT-OPTION (sub+100) <payload, IAC doubled> IAC SE */
static void srv_send_subneg(uint8_t sub, const uint8_t *payload, size_t len)
{
    uint8_t pkt[6 + 2 * SRV_SB_MAX];
    size_t pos = 0;

    if (len > SRV_SB_MAX)
        len = SRV_SB_MAX;
    pkt[pos++] = IAC;
    pkt[pos++] = SB;
    pkt[pos++] = TELOPT_COMPORT;
    pkt[pos++] = (uint8_t)(sub + CPC_SERVER_OFFSET);
    for (size_t i = 0; i < len; i++) {
        pkt[pos++] = payload[i];
        if (payload[i] == IAC)
            pkt[pos++] = IAC;
    }
    pkt[pos++] = IAC;
    pkt[pos++] = SE;
    srv_send(pkt, pos);
}

static void srv_send_subneg_byte(uint8_t sub, uint8_t value)
{
    srv_send_subneg(sub, &value, 1);
}

static void srv_send_subneg_u32(uint8_t sub, uint32_t v)
{
    uint8_t d[4] = { (uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v };
    srv_send_subneg(sub, d, 4);
}

static void srv_enter_telnet(void)
{
    g_srv.telnet = 1;
    srv_send_neg(WILL, TELOPT_BINARY);
    srv_send_neg(DO,   TELOPT_BINARY);
    srv_send_neg(WILL, TELOPT_SGA);
    srv_send_neg(DO,   TELOPT_SGA);
    srv_send_neg(WILL, TELOPT_COMPORT);
    srv_send_neg(DO,   TELOPT_COMPORT);
    log_message(LOG_INFO, "[tcp] %s: Telnet/RFC 2217 mode", g_srv_peer);
}

static void srv_handle_neg(uint8_t cmd, uint8_t opt)
{
    int *local, *remote;

    switch (opt) {
    case TELOPT_BINARY:  local = &g_srv.binary_local;  remote = &g_srv.binary_remote;  break;
    case TELOPT_SGA:     local = &g_srv.sga_local;     remote = &g_srv.sga_remote;     break;
    case TELOPT_COMPORT: local = &g_srv.comport_local; remote = &g_srv.comport_remote; break;
    default:
        /* Refuse anything else, once, so nothing loops. */
        if (cmd == WILL)
            srv_send_neg(DONT, opt);
        else if (cmd == DO)
            srv_send_neg(WONT, opt);
        return;
    }
    switch (cmd) {
    case WILL: if (!*remote) { *remote = 1; srv_send_neg(DO,   opt); } break;
    case WONT: if (*remote)  { *remote = 0; srv_send_neg(DONT, opt); } break;
    case DO:   if (!*local)  { *local  = 1; srv_send_neg(WILL, opt); } break;
    case DONT: if (*local)   { *local  = 0; srv_send_neg(WONT, opt); } break;
    default: break;
    }
}

/* --- the local port's parameters, read and set as RFC 2217 values --- */

static int srv_port_get(struct termios *t)
{
    return g_fd >= 0 && tcgetattr(g_fd, t) == 0;
}

static uint32_t srv_current_baud(void)
{
    struct termios t;
    return srv_port_get(&t) ? (uint32_t)baud_from_speed(cfgetospeed(&t)) : 0;
}

static int srv_set_baud(uint32_t baud)
{
    struct termios t;
    speed_t s;

    if (speed_for_baud((int)baud, &s) != 0 || !srv_port_get(&t))
        return -1;
    cfsetispeed(&t, s);
    cfsetospeed(&t, s);
    return tcsetattr(g_fd, TCSANOW, &t);
}

static uint8_t srv_current_datasize(void)
{
    struct termios t;

    if (!srv_port_get(&t))
        return 0;
    switch (t.c_cflag & CSIZE) {
    case CS5: return 5;
    case CS6: return 6;
    case CS7: return 7;
    default:  return 8;
    }
}

static int srv_set_datasize(uint8_t bits)
{
    struct termios t;
    tcflag_t cs;

    switch (bits) {
    case 5: cs = CS5; break;
    case 6: cs = CS6; break;
    case 7: cs = CS7; break;
    case 8: cs = CS8; break;
    default: return -1;
    }
    if (!srv_port_get(&t))
        return -1;
    t.c_cflag = (t.c_cflag & ~(tcflag_t)CSIZE) | cs;
    return tcsetattr(g_fd, TCSANOW, &t);
}

static uint8_t srv_current_parity(void)
{
    struct termios t;

    if (!srv_port_get(&t))
        return 0;
    if (!(t.c_cflag & PARENB))
        return CPC_PARITY_NONE;
    return (t.c_cflag & PARODD) ? CPC_PARITY_ODD : CPC_PARITY_EVEN;
}

static int srv_set_parity(uint8_t p)
{
    struct termios t;

    if (!srv_port_get(&t))
        return -1;
    switch (p) {
    case CPC_PARITY_NONE: t.c_cflag &= ~(tcflag_t)PARENB; break;
    case CPC_PARITY_ODD:  t.c_cflag |= PARENB | PARODD; break;
    case CPC_PARITY_EVEN: t.c_cflag |= PARENB; t.c_cflag &= ~(tcflag_t)PARODD; break;
    default: return -1;   /* mark/space: not supported */
    }
    return tcsetattr(g_fd, TCSANOW, &t);
}

static uint8_t srv_current_stopsize(void)
{
    struct termios t;

    if (!srv_port_get(&t))
        return 0;
    return (t.c_cflag & CSTOPB) ? CPC_STOP_2 : CPC_STOP_1;
}

static int srv_set_stopsize(uint8_t s)
{
    struct termios t;

    if (!srv_port_get(&t))
        return -1;
    if (s == CPC_STOP_1)
        t.c_cflag &= ~(tcflag_t)CSTOPB;
    else if (s == CPC_STOP_2)
        t.c_cflag |= CSTOPB;
    else
        return -1;    /* 1.5 stop bits: not supported */
    return tcsetattr(g_fd, TCSANOW, &t);
}

/* One modem line on the client's request.  DTR and RTS are driven as a
 * pair everywhere else in this program; the protocol sets them one at a
 * time, so the pair state (g_power_on) is derived from the two lines. */
static void srv_set_line(int flag, int on)
{
    int was = g_power_on;

    set_serial_signal_state(flag, on);
    if (flag == TIOCM_DTR)
        g_dtr_on = on;
    else
        g_rts_on = on;
    g_power_on = g_dtr_on && g_rts_on;
    if (was != g_power_on)
        power_event(g_power_on, "rfc2217");
}

static void srv_handle_subneg(void)
{
    const uint8_t *d = g_srv.sb_buf;
    size_t len = g_srv.sb_len;
    uint8_t sub, v, now;

    if (g_srv.sb_opt != TELOPT_COMPORT || len < 1)
        return;
    sub = d[0];
    switch (sub) {
    case CPC_SIGNATURE: {
        static const char sig[] = "vectis " PROGRAM_VERSION;
        srv_send_subneg(sub, (const uint8_t *)sig, sizeof sig - 1);
        break;
    }
    case CPC_SET_BAUDRATE: {
        uint32_t baud, cur;
        if (len < 5)
            return;
        baud = ((uint32_t)d[1] << 24) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 8) | d[4];
        if (baud != 0) {
            if (srv_set_baud(baud) == 0)
                log_message(LOG_INFO, "[tcp] %s: baud rate %u", g_srv_peer, baud);
            else
                log_message(LOG_WARNING, "[tcp] %s: baud rate %u not applied", g_srv_peer, baud);
        }
        cur = srv_current_baud();
        srv_send_subneg_u32(sub, cur ? cur : baud);
        break;
    }
    case CPC_SET_DATASIZE:
        if (len < 2)
            return;
        v = d[1];
        if (v != 0 && srv_set_datasize(v) != 0)
            log_message(LOG_WARNING, "[tcp] %s: data size %u not applied", g_srv_peer, v);
        now = srv_current_datasize();
        srv_send_subneg_byte(sub, now ? now : v);
        break;
    case CPC_SET_PARITY:
        if (len < 2)
            return;
        v = d[1];
        if (v != 0 && srv_set_parity(v) != 0)
            log_message(LOG_WARNING, "[tcp] %s: parity %u not applied", g_srv_peer, v);
        now = srv_current_parity();
        srv_send_subneg_byte(sub, now ? now : v);
        break;
    case CPC_SET_STOPSIZE:
        if (len < 2)
            return;
        v = d[1];
        if (v != 0 && srv_set_stopsize(v) != 0)
            log_message(LOG_WARNING, "[tcp] %s: stop size %u not applied", g_srv_peer, v);
        now = srv_current_stopsize();
        srv_send_subneg_byte(sub, now ? now : v);
        break;
    case CPC_SET_CONTROL: {
        uint8_t reply;
        if (len < 2)
            return;
        v = d[1];
        reply = v;
        switch (v) {
        case CPC_CTRL_REQ_FLOW: reply = CPC_CTRL_NO_FLOW; break;   /* no flow control here */
        case CPC_CTRL_REQ_DTR:  reply = g_dtr_on ? CPC_CTRL_DTR_ON : CPC_CTRL_DTR_OFF; break;
        case CPC_CTRL_DTR_ON:   srv_set_line(TIOCM_DTR, 1); break;
        case CPC_CTRL_DTR_OFF:  srv_set_line(TIOCM_DTR, 0); break;
        case CPC_CTRL_REQ_RTS:  reply = g_rts_on ? CPC_CTRL_RTS_ON : CPC_CTRL_RTS_OFF; break;
        case CPC_CTRL_RTS_ON:   srv_set_line(TIOCM_RTS, 1); break;
        case CPC_CTRL_RTS_OFF:  srv_set_line(TIOCM_RTS, 0); break;
        default: break;   /* flow-control settings: acknowledged as received */
        }
        srv_send_subneg_byte(sub, reply);
        break;
    }
    case CPC_PURGE_DATA: {
        int how;
        if (len < 2)
            return;
        how = d[1] == 1 ? TCIFLUSH : d[1] == 2 ? TCOFLUSH : d[1] == 3 ? TCIOFLUSH : -1;
        if (how != -1 && g_fd >= 0)
            tcflush(g_fd, how);
        srv_send_subneg_byte(sub, d[1]);
        break;
    }
    case CPC_FLOWCONTROL_SUSP:
    case CPC_FLOWCONTROL_RESUM:
    case CPC_SET_LINESTATE_MASK:
    case CPC_SET_MODEMSTATE_MASK:
        /* Acknowledged so the client never waits; nothing to apply. */
        if (len >= 2)
            srv_send_subneg_byte(sub, d[1]);
        else
            srv_send_subneg(sub, NULL, 0);
        break;
    default:
        break;   /* unknown sub-option: ignored */
    }
}

/* Client bytes for the UART.  Silently dropped while the link is down. */
static void srv_to_uart(const uint8_t *data, size_t len)
{
    if (len == 0 || g_fd < 0)
        return;
    if (write_all(g_fd, data, len) < 0)
        log_errno(LOG_WARNING, "[tcp] write(serial)");
}

/* Telnet mode: strip and act on commands, unescape IAC IAC, forward the
 * payload to the UART. */
static void srv_telnet_process(const uint8_t *in, size_t n)
{
    static uint8_t out[READ_BUF_SIZE];
    size_t o = 0;

    for (size_t i = 0; i < n; i++) {
        uint8_t b = in[i];
        switch (g_srv.state) {
        case SS_DATA:
            if (b == IAC)
                g_srv.state = SS_IAC;
            else
                out[o++] = b;
            break;
        case SS_IAC:
            if (b == IAC) {
                out[o++] = IAC;
                g_srv.state = SS_DATA;
            } else if (b == WILL || b == WONT || b == DO || b == DONT) {
                g_srv.neg_cmd = b;
                g_srv.state = SS_NEG;
            } else if (b == SB) {
                g_srv.state = SS_SB_OPT;
            } else {
                if (b == BRK) {
                    /* Ctrl+B on a vectis client, "send brk" in telnet:
                     * flush what came before it, then break the line. */
                    srv_to_uart(out, o);
                    o = 0;
                    log_message(LOG_INFO, "[tcp] %s: break", g_srv_peer);
                    if (g_fd >= 0 && tcsendbreak(g_fd, 0) != 0)
                        log_errno(LOG_WARNING, "tcsendbreak");
                }
                g_srv.state = SS_DATA;   /* NOP, AYT, ...: ignored */
            }
            break;
        case SS_NEG:
            srv_handle_neg(g_srv.neg_cmd, b);
            g_srv.state = SS_DATA;
            break;
        case SS_SB_OPT:
            g_srv.sb_opt = b;
            g_srv.sb_len = 0;
            g_srv.state = SS_SB_DATA;
            break;
        case SS_SB_DATA:
            if (b == IAC)
                g_srv.state = SS_SB_IAC;
            else if (g_srv.sb_len < SRV_SB_MAX)
                g_srv.sb_buf[g_srv.sb_len++] = b;
            break;
        case SS_SB_IAC:
            if (b == SE) {
                srv_handle_subneg();
                g_srv.state = SS_DATA;
            } else if (b == IAC) {
                if (g_srv.sb_len < SRV_SB_MAX)
                    g_srv.sb_buf[g_srv.sb_len++] = IAC;
                g_srv.state = SS_SB_DATA;
            } else {
                g_srv.state = SS_DATA;   /* malformed escape: leave SB */
            }
            break;
        }
    }
    srv_to_uart(out, o);
}

/* Legacy raw mode (nc, socat): LF and CRLF become CR so Enter reaches
 * U-Boot as it expects, and Ctrl+P fires the reset pulse, as on the
 * original vectis-bootrom server. */
static void srv_legacy_process(const uint8_t *in, size_t n)
{
    static uint8_t out[READ_BUF_SIZE];
    size_t o = 0;

    for (size_t i = 0; i < n; i++) {
        uint8_t b = in[i];
        if (b == KEY_RESET) {
            srv_to_uart(out, o);
            o = 0;
            log_message(LOG_INFO, "[tcp] %s: reset request", g_srv_peer);
            send_reset_pulse("tcp");
            continue;
        }
        if (b == '\r') {
            out[o++] = '\r';
            if (i + 1 < n && in[i + 1] == '\n')
                i++;
        } else if (b == '\n') {
            out[o++] = '\r';
        } else {
            out[o++] = b;
        }
    }
    srv_to_uart(out, o);
}

static void srv_handle_client_input(void)
{
    static uint8_t in[READ_BUF_SIZE];
    ssize_t n = read(g_srv_fd, in, sizeof in);

    if (n < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
            return;
        srv_close_client("read failed, client dropped");
        return;
    }
    if (n == 0) {
        srv_close_client("disconnected");
        return;
    }
    if (!g_srv.telnet && memchr(in, IAC, (size_t)n) != NULL)
        srv_enter_telnet();
    if (g_srv.telnet)
        srv_telnet_process(in, (size_t)n);
    else
        srv_legacy_process(in, (size_t)n);
}

/* UART bytes for the client: raw in legacy mode, IAC-escaped in Telnet mode. */
static void srv_send_uart(const uint8_t *data, size_t n)
{
    static uint8_t chunk[READ_BUF_SIZE];
    size_t o = 0;

    if (g_srv_fd < 0 || n == 0)
        return;
    if (!g_srv.telnet) {
        srv_send(data, n);
        return;
    }
    for (size_t i = 0; i < n; i++) {
        if (o + 2 > sizeof chunk) {
            srv_send(chunk, o);
            o = 0;
            if (g_srv_fd < 0)
                return;
        }
        if (data[i] == IAC) {
            chunk[o++] = IAC;
            chunk[o++] = IAC;
        } else {
            chunk[o++] = data[i];
        }
    }
    srv_send(chunk, o);
}

static void srv_accept(void)
{
    struct sockaddr_in peer;
    socklen_t pl = sizeof peer;
    int one = 1;
    int cfd = accept4(g_srv_listen_fd, (struct sockaddr *)&peer, &pl,
                      SOCK_NONBLOCK | SOCK_CLOEXEC);

    if (cfd < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR &&
            errno != ECONNABORTED)
            log_errno(LOG_WARNING, "accept(tcp)");
        return;
    }
    if (g_srv_fd >= 0)
        srv_close_client("replaced by a new connection");
    memset(&g_srv, 0, sizeof g_srv);
    g_srv_fd = cfd;
    if (inet_ntop(AF_INET, &peer.sin_addr, g_srv_peer, sizeof g_srv_peer) == NULL)
        snprintf(g_srv_peer, sizeof g_srv_peer, "?");
    if (setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one) != 0)
        log_errno(LOG_WARNING, "setsockopt(TCP_NODELAY)");
    log_message(LOG_INFO, "[tcp] %s:%u connected%s", g_srv_peer,
                (unsigned)ntohs(peer.sin_port), g_fd < 0 ? " (link offline)" : "");
}

static int srv_listen(const char *addr, int port)
{
    int fd = tcp_listen(addr, port, "tcp");

    if (fd < 0)
        return -1;
    g_srv_listen_fd = fd;
    log_message(LOG_INFO, "TCP listener ready on %s:%d (telnet, RFC 2217, raw)",
                addr[0] ? addr : "0.0.0.0", port);
    return 0;
}

/* Poll slots for the listener and the client; 0 when -l is off. */
static int srv_fill_pollfds(struct pollfd *p)
{
    if (g_srv_listen_fd < 0)
        return 0;
    p[0].fd = g_srv_listen_fd; p[0].events = POLLIN; p[0].revents = 0;
    p[1].fd = g_srv_fd;        p[1].events = POLLIN; p[1].revents = 0;
    return SRV_NPOLL;
}

static void srv_handle_pollfds(const struct pollfd *p, int n)
{
    if (n <= 0)
        return;
    if (p[0].revents & POLLIN)
        srv_accept();
    if (g_srv_fd >= 0 && p[1].fd == g_srv_fd) {
        if (p[1].revents & POLLNVAL)
            srv_close_client("socket invalid");
        else if (p[1].revents & (POLLIN | POLLHUP | POLLERR))
            srv_handle_client_input();
    }
}

/* ---------- Connection ---------- */

static int connect_to(const char *host, const char *port)
{
    struct addrinfo hints, *res = NULL, *ai;
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    int rc = getaddrinfo(host, port, &hints, &res);
    if (rc != 0) {
        if (!g_link_quiet)
            log_message(LOG_ERR, "getaddrinfo: %s", gai_strerror(rc));
        return -1;
    }

    int s = -1;
    for (ai = res; ai; ai = ai->ai_next) {
        s = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (s < 0)
            continue;
        /* 10-second connect timeout so the user is not stuck indefinitely. */
        struct timeval tv = { .tv_sec = 10, .tv_usec = 0 };
        if (setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv) != 0)
            log_errno(LOG_WARNING, "setsockopt(SO_SNDTIMEO)");
        if (connect(s, ai->ai_addr, ai->ai_addrlen) == 0) {
            /* Log the resolved IP so multi-address hostnames are traceable. */
            char ipbuf[INET6_ADDRSTRLEN] = "";
            void *sin_addr = (ai->ai_family == AF_INET)
                ? (void *)&((struct sockaddr_in  *)ai->ai_addr)->sin_addr
                : (void *)&((struct sockaddr_in6 *)ai->ai_addr)->sin6_addr;
            inet_ntop(ai->ai_family, sin_addr, ipbuf, sizeof ipbuf);
            log_message(LOG_INFO, "Resolved %s -> %s", host, ipbuf);

            /* Clear connect timeout: SO_SNDTIMEO affects all writes, not just connect(). */
            struct timeval zero = { 0, 0 };
            (void)setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &zero, sizeof zero);
            /* Disable Nagle algorithm so keystrokes are sent immediately. */
            int one = 1;
            if (setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one) != 0)
                log_errno(LOG_WARNING, "setsockopt(TCP_NODELAY)");
            /* Enable keep-alive probes so a silent TCP drop is eventually detected. */
            if (setsockopt(s, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof one) != 0)
                log_errno(LOG_WARNING, "setsockopt(SO_KEEPALIVE)");
            /* Tune keepalive: detect dead connections within ~20 s. */
            int idle = 10, intvl = 5, cnt = 3;
            if (setsockopt(s, IPPROTO_TCP, TCP_KEEPIDLE,  &idle,  sizeof idle) != 0)
                log_errno(LOG_WARNING, "setsockopt(TCP_KEEPIDLE)");
            if (setsockopt(s, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof intvl) != 0)
                log_errno(LOG_WARNING, "setsockopt(TCP_KEEPINTVL)");
            if (setsockopt(s, IPPROTO_TCP, TCP_KEEPCNT,   &cnt,   sizeof cnt) != 0)
                log_errno(LOG_WARNING, "setsockopt(TCP_KEEPCNT)");
            break;
        }
        close(s);
        s = -1;
    }
    freeaddrinfo(res);

    if (s < 0 && !g_link_quiet)
        log_errno(LOG_ERR, "Unable to connect to %s:%s", host, port);
    return s;
}

/* -w or -l: the process is a daemon whose listeners must outlive the link. */
static int daemon_mode(void)
{
    return g_web_listen_fd >= 0 || g_srv_listen_fd >= 0;
}

/* Interruptible pause between link attempts (1-second ticks).  Keeps
 * servicing the -w web server and the -l listener meanwhile, so the page
 * stays reachable (reporting the link as offline) and TCP clients can
 * connect and negotiate while the device is away. */
static void sleep_reconnect(unsigned int seconds)
{
    long long until = now_ms() + (long long)seconds * 1000;

    while (!g_quit) {
        long long left = until - now_ms();
        struct pollfd pfds[SRV_NPOLL + WEB_NPOLL];
        int slice, nsrv, nweb;

        if (left <= 0)
            break;
        slice = left > 1000 ? 1000 : (int)left;
        nsrv = srv_fill_pollfds(pfds);
        nweb = web_fill_pollfds(pfds + nsrv);
        if (nsrv + nweb == 0) {
            struct pollfd dummy = { .fd = -1, .events = 0, .revents = 0 };
            poll(&dummy, 1, slice);
            continue;
        }
        if (poll(pfds, nsrv + nweb, slice) < 0 && errno != EINTR)
            break;
        srv_handle_pollfds(pfds, nsrv);
        web_handle_pollfds(pfds + nsrv, nweb);
    }
}

/* True when the session should be re-established after a failure: with
 * -r in RFC 2217 mode (unchanged behaviour), and always in daemon mode
 * (-w, -l), where the listeners must stay up whatever the link does. */
static int session_retry(void)
{
    return !g_quit && (daemon_mode() || (g_reconnect && !g_serial_mode));
}

/* Wait before the next link attempt.  In daemon mode only the first
 * failure of an outage is logged, so an unplugged device does not add a
 * line to syslog every RECONNECT_DELAY_S seconds. */
static void retry_pause(void)
{
    if (!g_link_quiet)
        log_message(LOG_INFO, "Retrying in %d seconds...", RECONNECT_DELAY_S);
    if (daemon_mode())
        g_link_quiet = 1;
    sleep_reconnect(RECONNECT_DELAY_S);
}

/* ---------- CLI parser ---------- */

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s -h <host> -p <port> [options]\n"
        "   or: %s -u <device> [options]\n"
        "\n"
        "RFC 2217/Telnet mode:\n"
        "  -h HOST         RFC 2217 server address\n"
        "  -p PORT         TCP port\n"
        "\n"
        "Direct serial mode:\n"
        "  -u DEVICE       local tty device path, for example /dev/ttyUSB0\n"
        "\n"
        "Port settings (default 115200 8N1):\n"
        "  -b BAUD         baud rate (default 115200, max 921600)\n"
        "  -d 5|6|7|8      data bits (default 8)\n"
        "  -s 1|2          stop bits (default 1)\n"
        "  -y N|E|O        parity: None/Even/Odd (default N)\n"
        "\n"
        "Connection options:\n"
        "  -r              reconnect automatically on disconnect (RFC 2217 mode only)\n"
        "  -t MS           reset pulse duration in ms (default 200)\n"
        "  -n              disable LF->CR+LF translation (for devices sending \\r\\n)\n"
        "  -l [ADDR:]PORT  serve the local port (-u) to one TCP client at a time:\n"
        "                  telnet, another vectis -h HOST -p PORT, pyserial\n"
        "                  rfc2217://, or raw nc/socat (Ctrl+P = reset pulse)\n"
        "  -w [ADDR:]PORT  built-in web page with Power Enable/Disable/Reset buttons;\n"
        "                  scripts: curl HOST:PORT/status | /power/enable|disable|reset\n"
        "                  With -l or -w the device link is (re)opened in the background.\n"
        "  -v, --version   print version and release date\n"
        "  --help, -?      this help\n"
        "\n"
        "Examples:\n"
        "  %s -h 192.168.1.10 -p 7000                    # 115200 8N1\n"
        "  %s -h 192.168.1.10 -p 7000 -b 9600 -y E       # 9600 8E1\n"
        "  %s -h 192.168.1.10 -p 7000 -r                 # auto-reconnect\n"
        "  %s -u /dev/ttyUSB0                             # direct serial mode\n"
        "  %s -u /dev/ttyUSB0 -b 460800 -t 500           # 460800 baud, 500 ms reset\n"
        "  %s -h 192.168.1.10 -p 7000 -n                 # no LF->CRLF (scripted capture)\n"
        "  %s -u /dev/ttyUSB0 -w 8080                    # web power console on :8080\n"
        "  %s -u /dev/ttyUSB0 -l 2021 -w 2081            # UART on TCP :2021 + web on :2081\n"
        "\n"
        "Session controls (interactive mode only):\n"
        "  Ctrl+P   RTS+DTR pulse — reset the target device\n"
        "  Ctrl+B   send break signal (stops U-Boot autoboot)\n"
        "  Ctrl+]   exit\n",
        prog, prog, prog, prog, prog, prog, prog, prog, prog, prog);
}

int main(int argc, char **argv)
{
    const char *host = NULL;
    const char *port = NULL;
    const char *device = NULL;
    uint32_t baud = 115200;
    int data_bits = 8;
    int stop_bits = 1;
    char parity = 'N';
    int want_help = 0;
    int want_version = 0;
    char web_addr[64] = "";
    int web_port = 0;
    char lsn_addr[64] = "";
    int lsn_port = 0;
    const struct option long_opts[] = {
        { "help",      no_argument,       &want_help,   1 },
        { "version",   no_argument,       &want_version, 1 },
        { "device",    required_argument, NULL,        'u' },
        { "host",      required_argument, NULL,        'h' },
        { "port",      required_argument, NULL,        'p' },
        { "baud",      required_argument, NULL,        'b' },
        { "data-bits", required_argument, NULL,        'd' },
        { "stop-bits", required_argument, NULL,        's' },
        { "parity",    required_argument, NULL,        'y' },
        { "reset-ms",  required_argument, NULL,        't' },
        { "reconnect", no_argument,       NULL,        'r' },
        { "no-crlf",   no_argument,       NULL,        'n' },
        { "web",       required_argument, NULL,        'w' },
        { "listen",    required_argument, NULL,        'l' },
        { 0, 0, 0, 0 }
    };

    openlog(g_progname, LOG_PID | LOG_NDELAY, LOG_DAEMON);
    atexit(closelog);
    int opt;
    while ((opt = getopt_long(argc, argv, "h:p:b:d:s:y:u:t:w:l:nrv?", long_opts, NULL)) != -1) {
        switch (opt) {
        case 'h': host = optarg; break;
        case 'p': {
            char *endp;
            errno = 0;
            long pv = strtol(optarg, &endp, 10);
            if (errno != 0 || *endp != '\0' || pv < 1 || pv > 65535) {
                log_message(LOG_ERR, "Invalid port number: %s", optarg);
                return 1;
            }
            port = optarg;
            break;
        }
        case 'b': {
            char *endp;
            errno = 0;
            unsigned long v = strtoul(optarg, &endp, 10);
            /* Upper bound matches speed_for_baud() and the server table. */
            if (errno != 0 || *endp != '\0' || v == 0 || v > 921600) {
                log_message(LOG_ERR, "Invalid baud rate: %s", optarg);
                return 1;
            }
            baud = (uint32_t)v;
            break;
        }
        case 'd': {
            char *endp;
            errno = 0;
            long v = strtol(optarg, &endp, 10);
            if (errno != 0 || *endp != '\0') {
                log_message(LOG_ERR, "Invalid data bits: %s", optarg);
                return 1;
            }
            data_bits = (int)v;
            break;
        }
        case 's': {
            char *endp;
            errno = 0;
            long v = strtol(optarg, &endp, 10);
            if (errno != 0 || *endp != '\0') {
                log_message(LOG_ERR, "Invalid stop bits: %s", optarg);
                return 1;
            }
            stop_bits = (int)v;
            break;
        }
        case 'y':
            if (optarg[0] == '\0') {
                log_message(LOG_ERR, "Invalid parity: empty value");
                return 1;
            }
            parity = (char)toupper((unsigned char)optarg[0]);
            break;
        case 'u': device = optarg; break;
        case 'r': g_reconnect = 1; break;
        case 'n': g_no_crlf   = 1; break;
        case 't': {
            char *endp;
            errno = 0;
            long v = strtol(optarg, &endp, 10);
            if (errno != 0 || *endp != '\0' || v < 1 || v > 60000) {
                log_message(LOG_ERR, "Invalid reset pulse duration: %s (1..60000 ms)", optarg);
                return 1;
            }
            g_reset_ms = (int)v;
            break;
        }
        case 'w':
            if (parse_bind(optarg, web_addr, sizeof web_addr, &web_port) != 0) {
                log_message(LOG_ERR, "Invalid -w spec: %s (use PORT or ADDR:PORT)", optarg);
                return 1;
            }
            break;
        case 'l':
            if (parse_bind(optarg, lsn_addr, sizeof lsn_addr, &lsn_port) != 0) {
                log_message(LOG_ERR, "Invalid -l spec: %s (use PORT or ADDR:PORT)", optarg);
                return 1;
            }
            break;
        case 'v': want_version = 1; break;
        case 0:
            break;
        case '?':
            if (optopt == '?') {
                want_help = 1;
                break;
            }
            usage(argv[0]);
            return 1;
        default:
            return 1;
        }
    }

    if (want_help) {
        usage(argv[0]);
        return 0;
    }

    if (want_version) {
        print_version(argv[0]);
        return 0;
    }

    if ((device != NULL) && (host != NULL || port != NULL)) {
        log_message(LOG_ERR, "Choose either RFC 2217 mode or direct serial mode");
        usage(argv[0]);
        return 1;
    }

    if (device == NULL && (!host || !port)) {
        log_message(LOG_ERR, "Missing -h <host> and/or -p <port> or -u <device>");
        usage(argv[0]);
        return 1;
    }
    if (data_bits < 5 || data_bits > 8) {
        log_message(LOG_ERR, "Invalid data bits: %d", data_bits);
        return 1;
    }
    if (stop_bits != 1 && stop_bits != 2) {
        log_message(LOG_ERR, "Stop bits must be 1 or 2");
        return 1;
    }

    uint8_t parity_v;
    switch (parity) {
    case 'N': parity_v = CPC_PARITY_NONE; break;
    case 'E': parity_v = CPC_PARITY_EVEN; break;
    case 'O': parity_v = CPC_PARITY_ODD;  break;
    default:
        log_message(LOG_ERR, "Invalid parity: %c (must be N/E/O)", parity);
        return 1;
    }

    {
        struct sigaction sa;
        memset(&sa, 0, sizeof sa);
        sa.sa_handler = on_signal;
        sa.sa_flags   = 0;
        sigemptyset(&sa.sa_mask);
        sigaction(SIGINT,  &sa, NULL);
        sigaction(SIGTERM, &sa, NULL);
        sa.sa_handler = SIG_IGN;
        sa.sa_flags   = 0;
        sigaction(SIGPIPE, &sa, NULL);
    }

    atexit(cleanup);

    g_serial_mode = (device != NULL);
    if (g_serial_mode)
        snprintf(g_web_target, sizeof g_web_target, "%s", device);
    else
        snprintf(g_web_target, sizeof g_web_target, "%s:%s", host, port);

    if (lsn_port > 0 && !g_serial_mode) {
        log_message(LOG_ERR, "-l serves a local port: combine it with -u DEVICE, not -h/-p");
        return 1;
    }

    /* -l / -w: bring the listeners up first so they are reachable at once,
     * before (and regardless of) the device link. */
    if (lsn_port > 0 && srv_listen(lsn_addr, lsn_port) != 0)
        return 1;
    if (web_port > 0 && web_listen(web_addr, web_port) != 0)
        return 1;

    /* Switch stdin to raw mode only when running interactively.
     * When stdin is a pipe or file, skip raw mode so the tool can
     * be driven by scripts: echo "cmd" | vectis -h host -p port */
    g_interactive = isatty(STDIN_FILENO);
    if (g_interactive)
        set_raw_terminal();

    /* Main session loop — iterates only when -r is active and connection drops. */
    do {
        if (g_serial_mode) {
            g_fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (g_fd < 0) {
                if (!g_link_quiet)
                    log_errno(LOG_ERR, "open(serial)");
                if (session_retry()) {
                    retry_pause();
                    continue;
                }
                return 1;
            }
            /* Clear O_NONBLOCK so subsequent reads/writes block normally. */
            {
                int flags = fcntl(g_fd, F_GETFL);
                if (flags < 0) {
                    log_errno(LOG_ERR, "fcntl(F_GETFL)");
                    return 1;
                }
                if (fcntl(g_fd, F_SETFL, flags & ~O_NONBLOCK) < 0) {
                    log_errno(LOG_ERR, "fcntl(F_SETFL)");
                    return 1;
                }
            }
            if (configure_serial(g_fd, (int)baud, data_bits, stop_bits, parity) != 0) {
                disconnect();
                if (session_retry()) {
                    retry_pause();
                    continue;
                }
                return 1;
            }
            (void)power_set(1, "link");
            log_message(LOG_INFO, "Opened serial device %s. baud=%u, data=%d, stop=%d, parity=%c",
                device, baud, data_bits, stop_bits, parity);
            if (g_interactive)
                log_message(LOG_INFO,
                    "Ctrl+P resets RTS+DTR for %d ms, Ctrl+B break, Ctrl+] exits",
                    g_reset_ms);
        } else {
            g_fd = connect_to(host, port);
            if (g_fd < 0) {
                if (session_retry()) {
                    retry_pause();
                    continue;
                }
                return 1;
            }

            log_message(LOG_INFO, "Connected to %s:%s. baud=%u, data=%d, stop=%d, parity=%c",
                host, port, baud, data_bits, stop_bits, parity);
            if (g_interactive)
                log_message(LOG_INFO,
                    "Ctrl+P resets RTS+DTR for %d ms, Ctrl+B break, Ctrl+] exits",
                    g_reset_ms);

            /* Request negotiations with the server. */
            if (telnet_send_negot(WILL, TELOPT_BINARY)  < 0 ||
                telnet_send_negot(DO,   TELOPT_BINARY)  < 0 ||
                telnet_send_negot(WILL, TELOPT_SGA)     < 0 ||
                telnet_send_negot(DO,   TELOPT_SGA)     < 0 ||
                telnet_send_negot(WILL, TELOPT_COMPORT) < 0) {
                log_errno(LOG_ERR, "Telnet negotiation failed");
                disconnect();
                if (session_retry()) {
                    retry_pause();
                    continue;
                }
                return 1;
            }

            /* Configure the port parameters.
               A small delay after WILL COM-PORT helps some servers reply with DO first. */
            sleep_us(100 * 1000);

            if (comport_set_baudrate(baud)               < 0 ||
                comport_set_datasize((uint8_t)data_bits) < 0 ||
                comport_set_stop((uint8_t)stop_bits)     < 0 ||
                comport_set_parity(parity_v)             < 0 ||
                comport_set_control(CPC_CTRL_NO_FLOW)    < 0 ||
                comport_set_control(CPC_CTRL_DTR_ON)     < 0 ||
                comport_set_control(CPC_CTRL_RTS_ON)     < 0) {
                log_errno(LOG_ERR, "Port configuration failed");
                disconnect();
                if (session_retry()) {
                    retry_pause();
                    continue;
                }
                return 1;
            }
        }

        /* Both paths leave DTR and RTS asserted; the link is up again. */
        if (!g_power_on)
            power_event(1, "link");
        g_dtr_on = g_rts_on = g_power_on = 1;
        g_link_quiet = 0;

        /* Main I/O loop: poll on the transport, stdin and the -w web server. */
        static uint8_t buf[READ_BUF_SIZE]; /* static to avoid large stack allocation (N2) */
        while (!g_quit) {
            struct pollfd pfds[2 + SRV_NPOLL + WEB_NPOLL];
            pfds[0].fd     = g_fd;
            pfds[0].events = POLLIN;
            /* A negative fd makes poll() skip the slot (revents = 0). */
            pfds[1].fd     = g_watch_stdin ? STDIN_FILENO : -1;
            pfds[1].events = POLLIN;
            int nsrv = srv_fill_pollfds(&pfds[2]);
            int nweb = web_fill_pollfds(&pfds[2 + nsrv]);

            int r = poll(pfds, 2 + nsrv + nweb, 500); /* 500 ms timeout to respond to signals promptly */
            if (r < 0) {
                if (errno == EINTR)
                    continue;
                log_errno(LOG_ERR, "poll");
                break;
            }

            /* Listeners first: the transport/stdin branches below may
             * `continue` or `break` and must not starve the TCP client
             * or the web page. */
            srv_handle_pollfds(&pfds[2], nsrv);
            web_handle_pollfds(&pfds[2 + nsrv], nweb);

            /* Transport error or hangup — no point continuing. */
            if (pfds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
                log_message(LOG_INFO, g_serial_mode
                    ? "Serial device error/hangup"
                    : "Connection error/hangup");
                break;
            }

            /* Data from the transport. */
            if (pfds[0].revents & POLLIN) {
                ssize_t n = read(g_fd, buf, sizeof buf);
                if (n < 0) {
                    /* Retry on signal interruption (N3). */
                    if (errno == EINTR || errno == EAGAIN)
                        continue;
                    log_errno(LOG_ERR, g_serial_mode ? "read(serial)" : "read(socket)");
                    break;
                }
                if (n == 0) {
                    log_message(LOG_INFO, g_serial_mode
                        ? "Serial device closed"
                        : "Connection closed");
                    break;
                }
                if (g_serial_mode) {
                    srv_send_uart(buf, (size_t)n);   /* -l client, if any */
                    if (process_serial_incoming(buf, (size_t)n) != 0) {
                        log_errno(LOG_ERR, "stdout");
                        break;
                    }
                } else {
                    if (process_incoming(buf, (size_t)n) != 0) {
                        log_errno(LOG_ERR, "stdout");
                        break;
                    }
                }
            }

            /* Keyboard / pipe input.  POLLHUP/POLLERR/POLLNVAL without
             * POLLIN means EOF with nothing left to read (a drained pipe
             * whose writer exited, a closed fd).  It has to be handled
             * here: poll() would otherwise report it again immediately,
             * on every iteration. */
            if (pfds[1].revents & (POLLIN | POLLHUP | POLLERR | POLLNVAL)) {
                ssize_t n = 0;
                if (pfds[1].revents & POLLIN) {
                    n = read(STDIN_FILENO, buf, sizeof buf);
                    if (n < 0) {
                        if (errno == EINTR || errno == EAGAIN) /* signal or spurious wakeup (N3) */
                            continue;
                        log_errno(LOG_WARNING, "read(stdin)");
                        n = 0; /* unreadable stdin: same handling as EOF */
                    }
                }
                if (n == 0) {
                    if (on_stdin_eof())
                        break;
                    continue;
                }

                /* Walk the input buffer: in interactive mode handle special keys;
                 * in non-interactive mode forward every byte directly so that
                 * control characters (0x02, 0x10, 0x1D) in piped data are not
                 * misinterpreted as commands. Batch the rest into one write. */
                static uint8_t outbuf[sizeof(buf) * 2]; /* worst case: every byte is IAC; static = no stack (N2) */
                size_t  outlen = 0;

                for (ssize_t i = 0; i < n; i++) {
                    uint8_t c = buf[i];

                    if (g_interactive) {
                        if (c == KEY_RESET) {
                            if (outlen > 0) {
                                if (write_all(g_fd, outbuf, outlen) < 0) {
                                    g_quit = 1;
                                    break;
                                }
                                outlen = 0;
                            }
                            send_reset_pulse("keyboard");
                            continue;
                        }
                        if (c == KEY_BREAK) {
                            if (outlen > 0) {
                                if (write_all(g_fd, outbuf, outlen) < 0) {
                                    g_quit = 1;
                                    break;
                                }
                                outlen = 0;
                            }
                            send_break();
                            continue;
                        }
                        if (c == KEY_EXIT) {
                            if (outlen > 0) {
                                if (write_all(g_fd, outbuf, outlen) < 0) {
                                    g_quit = 1;
                                    break;
                                }
                                outlen = 0;
                            }
                            g_quit = 1;
                            break;
                        }
                    }

                    if (g_serial_mode) {
                        outbuf[outlen++] = c;
                    } else if (c == IAC) {
                        outbuf[outlen++] = IAC;
                        outbuf[outlen++] = IAC; /* escape IAC */
                    } else {
                        outbuf[outlen++] = c;
                    }
                }

                if (!g_quit && outlen > 0) {
                    if (write_all(g_fd, outbuf, outlen) < 0)
                        g_quit = 1;
                }
            }
        }

        disconnect();

        if (session_retry()) {
            log_message(LOG_INFO, "Reconnecting in %d seconds...", RECONNECT_DELAY_S);
            sleep_reconnect(RECONNECT_DELAY_S);
        }
    } while (session_retry());

    log_message(LOG_INFO, "Exiting");
    return 0;
}
