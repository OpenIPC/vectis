/*
 *
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * vectis-cli.c — vectis-cli, a simple RFC 2217 Telnet client for Linux
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
#define CPC_STOP_15  3   /* 1.5 */

#define PROGRAM_VERSION      "1.4.0"
#define PROGRAM_RELEASE_DATE "2026-05-02"

#define RECONNECT_DELAY_S 5

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
static const char *g_progname = "vectis-cli";
static int g_serial_mode  = 0;
static int g_reset_ms     = 200;
static int g_reconnect    = 0;
static int g_no_crlf      = 0;  /* -n: disable LF->CR+LF translation */
static int g_interactive  = 0;  /* stdin is a tty */
static void set_serial_signal_state(int signal_flag, int active);

/* Telnet parser state */
static enum tn_state g_tn_state = TS_DATA;
static uint8_t       g_negot_cmd = 0;
/* 512 bytes: generous for any RFC 2217 sub-negotiation payload. */
static uint8_t       g_sb_buf[512];
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
}

static void cleanup(void)
{
    restore_terminal();
    disconnect();
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

static int speed_for_baud(int baud, speed_t *out)
{
    switch (baud) {
    case 300:    *out = B300;    return 0;
    case 1200:   *out = B1200;   return 0;
    case 2400:   *out = B2400;   return 0;
    case 4800:   *out = B4800;   return 0;
    case 9600:   *out = B9600;   return 0;
    case 19200:  *out = B19200;  return 0;
    case 38400:  *out = B38400;  return 0;
    case 57600:  *out = B57600;  return 0;
    case 115200: *out = B115200; return 0;
    case 230400: *out = B230400; return 0;
    case 460800: *out = B460800; return 0;
    case 921600: *out = B921600; return 0;
    default:                     return -1;
    }
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
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
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

static void send_reset_pulse(void)
{
    if (g_fd < 0)
        return;
    fputs("\r\n", stderr);
    if (g_serial_mode) {
        log_message(LOG_INFO, "[reset] RTS+DTR off for %d ms", g_reset_ms);
        set_serial_signal_state(TIOCM_DTR, 0);
        set_serial_signal_state(TIOCM_RTS, 0);
        sleep_us((unsigned int)g_reset_ms * 1000);
        set_serial_signal_state(TIOCM_DTR, 1);
        set_serial_signal_state(TIOCM_RTS, 1);
        log_message(LOG_INFO, "[reset] done");
        return;
    }

    log_message(LOG_INFO, "[reset] RTS+DTR off for %d ms", g_reset_ms);
    comport_set_control(CPC_CTRL_DTR_OFF);
    comport_set_control(CPC_CTRL_RTS_OFF);
    sleep_us((unsigned int)g_reset_ms * 1000);
    comport_set_control(CPC_CTRL_DTR_ON);
    comport_set_control(CPC_CTRL_RTS_ON);
    log_message(LOG_INFO, "[reset] done");
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

    if (s < 0)
        log_errno(LOG_ERR, "Unable to connect to %s:%s", host, port);
    return s;
}

/* Interruptible sleep between reconnect attempts (1-second ticks). */
static void sleep_reconnect(unsigned int seconds)
{
    struct pollfd dummy;
    dummy.fd     = -1;
    dummy.events = 0;
    unsigned int i;
    for (i = 0; i < seconds && !g_quit; i++)
        poll(&dummy, 1, 1000);
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
        "  -b BAUD         baud rate (default 115200)\n"
        "  -d 5|6|7|8      data bits (default 8)\n"
        "  -s 1|2          stop bits (default 1)\n"
        "  -y N|E|O        parity: None/Even/Odd (default N)\n"
        "\n"
        "Connection options:\n"
        "  -r              reconnect automatically on disconnect (RFC 2217 mode only)\n"
        "  -t MS           reset pulse duration in ms (default 200)\n"
        "  -n              disable LF->CR+LF translation (for devices sending \\r\\n)\n"
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
        "\n"
        "Session controls (interactive mode only):\n"
        "  Ctrl+P   RTS+DTR pulse — reset the target device\n"
        "  Ctrl+B   send break signal (stops U-Boot autoboot)\n"
        "  Ctrl+]   exit\n",
        prog, prog, prog, prog, prog, prog, prog, prog);
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
    const struct option long_opts[] = {
        { "help",    no_argument,       &want_help,    1 },
        { "version", no_argument,       &want_version, 1 },
        { "device",  required_argument, NULL,         'u' },
        { 0, 0, 0, 0 }
    };

    openlog(g_progname, LOG_PID | LOG_NDELAY, LOG_DAEMON);
    atexit(closelog);
    int opt;
    while ((opt = getopt_long(argc, argv, "h:p:b:d:s:y:u:t:nrv?", long_opts, NULL)) != -1) {
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
            if (errno != 0 || *endp != '\0' || v == 0 || v > 460800) {
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
        sa.sa_flags   = SA_RESTART;
        sigemptyset(&sa.sa_mask);
        sigaction(SIGINT,  &sa, NULL);
        sigaction(SIGTERM, &sa, NULL);
        sa.sa_handler = SIG_IGN;
        sa.sa_flags   = 0;
        sigaction(SIGPIPE, &sa, NULL);
    }

    atexit(cleanup);

    g_serial_mode = (device != NULL);

    /* Switch stdin to raw mode only when running interactively.
     * When stdin is a pipe or file, skip raw mode so the tool can
     * be driven by scripts: echo "cmd" | vectis-cli -h host -p port */
    g_interactive = isatty(STDIN_FILENO);
    if (g_interactive)
        set_raw_terminal();

    /* Main session loop — iterates only when -r is active and connection drops. */
    do {
        if (g_serial_mode) {
            g_fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (g_fd < 0) {
                log_errno(LOG_ERR, "open(serial)");
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
                return 1;
            }
            set_serial_signal_state(TIOCM_DTR, 1);
            set_serial_signal_state(TIOCM_RTS, 1);
            log_message(LOG_INFO, "Opened serial device %s. baud=%u, data=%d, stop=%d, parity=%c",
                device, baud, data_bits, stop_bits, parity);
            if (g_interactive)
                log_message(LOG_INFO,
                    "Ctrl+P resets RTS+DTR for %d ms, Ctrl+B break, Ctrl+] exits",
                    g_reset_ms);
        } else {
            g_fd = connect_to(host, port);
            if (g_fd < 0) {
                if (g_reconnect && !g_quit) {
                    log_message(LOG_INFO, "Retrying in %d seconds...", RECONNECT_DELAY_S);
                    sleep_reconnect(RECONNECT_DELAY_S);
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
                if (g_reconnect && !g_quit) {
                    log_message(LOG_INFO, "Retrying in %d seconds...", RECONNECT_DELAY_S);
                    sleep_reconnect(RECONNECT_DELAY_S);
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
                if (g_reconnect && !g_quit) {
                    log_message(LOG_INFO, "Retrying in %d seconds...", RECONNECT_DELAY_S);
                    sleep_reconnect(RECONNECT_DELAY_S);
                    continue;
                }
                return 1;
            }
        }

        /* Main I/O loop: poll on the transport and stdin. */
        static uint8_t buf[4096]; /* static to avoid large stack allocation (N2) */
        while (!g_quit) {
            struct pollfd pfds[2];
            pfds[0].fd     = g_fd;
            pfds[0].events = POLLIN;
            pfds[1].fd     = STDIN_FILENO;
            pfds[1].events = POLLIN;

            int r = poll(pfds, 2, -1);
            if (r < 0) {
                if (errno == EINTR)
                    continue;
                log_errno(LOG_ERR, "poll");
                break;
            }

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

            /* Keyboard / pipe input. */
            if (pfds[1].revents & POLLIN) {
                ssize_t n = read(STDIN_FILENO, buf, sizeof buf);
                if (n < 0) {
                    if (errno == EINTR || errno == EAGAIN) /* signal or spurious wakeup (N3) */
                        continue;
                    break;
                }
                if (n == 0)
                    break;

                /* Walk the input buffer: in interactive mode handle special keys;
                 * in non-interactive mode forward every byte directly so that
                 * control characters (0x02, 0x10, 0x1D) in piped data are not
                 * misinterpreted as commands. Batch the rest into one write. */
                static uint8_t outbuf[sizeof(buf) * 2]; /* worst case: every byte is IAC; static = no stack (N2) */
                size_t  outlen = 0;

                for (ssize_t i = 0; i < n; i++) {
                    uint8_t c = buf[i];

                    if (g_interactive) {
                        if (c == 0x10) { /* Ctrl+P -> reset pulse */
                            if (outlen > 0) {
                                if (write_all(g_fd, outbuf, outlen) < 0) {
                                    g_quit = 1;
                                    break;
                                }
                                outlen = 0;
                            }
                            send_reset_pulse();
                            continue;
                        }
                        if (c == 0x02) { /* Ctrl+B -> break signal */
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
                        if (c == 0x1D) { /* Ctrl+] -> exit */
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

        if (g_reconnect && !g_quit && !g_serial_mode) {
            log_message(LOG_INFO, "Reconnecting in %d seconds...", RECONNECT_DELAY_S);
            sleep_reconnect(RECONNECT_DELAY_S);
        }
    } while (g_reconnect && !g_quit && !g_serial_mode);

    log_message(LOG_INFO, "Exiting");
    return 0;
}
