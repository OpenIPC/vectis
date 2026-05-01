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
#include <termios.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netdb.h>
#include <arpa/inet.h>
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

/* Telnet options */
#define TELOPT_BINARY     0
#define TELOPT_ECHO       1
#define TELOPT_SGA        3
#define TELOPT_COMPORT    44   /* RFC 2217 */

/* Port-control commands (client -> server) */
#define CPC_SET_BAUDRATE      1
#define CPC_SET_DATASIZE      2
#define CPC_SET_PARITY        3
#define CPC_SET_STOPSIZE      4
#define CPC_SET_CONTROL       5
#define CPC_NOTIFY_LINESTATE  6
#define CPC_NOTIFY_MODEMSTATE 7
#define CPC_FLOWCONTROL_SUSP  8
#define CPC_FLOWCONTROL_RESUM 9
#define CPC_SET_LINESTATE_MASK  10
#define CPC_SET_MODEMSTATE_MASK 11
#define CPC_PURGE_DATA        12

/* Values for CPC_SET_CONTROL (RTS/DTR/flow control) */
#define CPC_CTRL_REQ_FLOW         0
#define CPC_CTRL_NO_FLOW          1
#define CPC_CTRL_XON_XOFF         2
#define CPC_CTRL_HW_FLOW          3

#define CPC_CTRL_REQ_DTR          7
#define CPC_CTRL_DTR_ON           8
#define CPC_CTRL_DTR_OFF          9

#define CPC_CTRL_REQ_RTS          10
#define CPC_CTRL_RTS_ON           11
#define CPC_CTRL_RTS_OFF          12

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

/* Global state */
static int g_sock = -1;
static struct termios g_old_tio;
static int g_tio_saved = 0;
static volatile sig_atomic_t g_quit = 0;
static const char *g_progname = "vectis-cli";

/* ---------- Logging ---------- */

static void log_message(int priority, const char *fmt, ...)
{
    char buf[1024];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);

    syslog(priority, "%s", buf);
    fprintf(stderr, "%s: %s\n", g_progname, buf);
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
    fprintf(stderr, "%s: %s: %s\n", g_progname, buf, strerror(saved_errno));
}

static void die_errno(const char *msg)
{
    log_errno(LOG_ERR, "%s", msg);
    exit(1);
}

static void sleep_us(unsigned int usec)
{
    while (usec > 0) {
        if (usleep(usec) == 0)
            return;
        if (errno != EINTR)
            return;
    }
}

/* ---------- Utilities ---------- */

static void restore_terminal(void)
{
    if (g_tio_saved) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_old_tio);
        g_tio_saved = 0;
    }
}

static void cleanup(void)
{
    restore_terminal();
    if (g_sock >= 0) {
        close(g_sock);
        g_sock = -1;
    }
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
    g_tio_saved = 1;

    tio = g_old_tio;
    /* Full raw mode so every key press (including Ctrl+P) reaches us. */
    cfmakeraw(&tio);
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &tio) != 0)
        die_errno("tcsetattr(stdin)");
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
    return write_all(g_sock, buf, 3);
}

/*
 * Send sub-negotiation for COM Port Control:
 * IAC SB COM-PORT-OPTION <subcmd> <data...> IAC SE
 *
 * Inside payload data, bytes with value 255 (IAC) are doubled.
 */
static int comport_send(uint8_t subcmd, const uint8_t *data, size_t len)
{
    uint8_t hdr[4] = { IAC, SB, TELOPT_COMPORT, subcmd };
    if (write_all(g_sock, hdr, 4) < 0)
        return -1;

    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        if (write_all(g_sock, &b, 1) < 0)
            return -1;
        if (b == IAC) {
            /* Escape IAC inside payload. */
            if (write_all(g_sock, &b, 1) < 0)
                return -1;
        }
    }

    uint8_t tail[2] = { IAC, SE };
    return write_all(g_sock, tail, 2);
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
    switch (cmd) {
    case DO:
        if (opt == TELOPT_BINARY || opt == TELOPT_SGA || opt == TELOPT_COMPORT)
            telnet_send_negot(WILL, opt);
        else
            telnet_send_negot(WONT, opt);
        break;
    case DONT:
        telnet_send_negot(WONT, opt);
        break;
    case WILL:
        if (opt == TELOPT_BINARY || opt == TELOPT_SGA || opt == TELOPT_ECHO || opt == TELOPT_COMPORT)
            telnet_send_negot(DO, opt);
        else
            telnet_send_negot(DONT, opt);
        break;
    case WONT:
        telnet_send_negot(DONT, opt);
        break;
    default:
        break;
    }
}

/* Process COM-PORT sub-negotiation for diagnostics. */
static void handle_subneg(const uint8_t *buf, size_t len)
{
    if (len < 1)
        return;
    if (buf[0] != TELOPT_COMPORT)
        return; /* We only care about COM-PORT. */

    if (len < 2)
        return;
    uint8_t sub = buf[1];

    /* The server answers our commands with an offset of +100 (see RFC 2217 §4.3).
       The exact value does not matter here; we only keep the hook for debugging. */
    (void)sub;
}

/* ---------- Incoming stream parser ---------- */

enum tn_state {
    TS_DATA,
    TS_IAC,
    TS_NEGOT,         /* Waiting for the option byte after DO/DONT/WILL/WONT. */
    TS_SB,            /* Collecting sub-negotiation data. */
    TS_SB_IAC         /* IAC seen inside SB. */
};

static enum tn_state g_state = TS_DATA;
static uint8_t       g_negot_cmd = 0;
static uint8_t       g_sb_buf[512];
static size_t        g_sb_len = 0;

/* Parse a chunk of incoming data. Plain data is written to stdout. */
static void process_incoming(const uint8_t *buf, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        uint8_t b = buf[i];
        switch (g_state) {
        case TS_DATA:
            if (b == IAC)
                g_state = TS_IAC;
            else
                (void)!write(STDOUT_FILENO, &b, 1);
            break;

        case TS_IAC:
            if (b == IAC) {
                /* Escaped 0xFF becomes data. */
                (void)!write(STDOUT_FILENO, &b, 1);
                g_state = TS_DATA;
            } else if (b == DO || b == DONT || b == WILL || b == WONT) {
                g_negot_cmd = b;
                g_state = TS_NEGOT;
            } else if (b == SB) {
                g_sb_len = 0;
                g_state = TS_SB;
            } else {
                /* Ignore other commands (NOP, etc.). */
                g_state = TS_DATA;
            }
            break;

        case TS_NEGOT:
            handle_negot(g_negot_cmd, b);
            g_state = TS_DATA;
            break;

        case TS_SB:
            if (b == IAC) {
                g_state = TS_SB_IAC;
            } else {
                if (g_sb_len < sizeof(g_sb_buf))
                    g_sb_buf[g_sb_len++] = b;
            }
            break;

        case TS_SB_IAC:
            if (b == SE) {
                handle_subneg(g_sb_buf, g_sb_len);
                g_sb_len = 0;
                g_state = TS_DATA;
            } else if (b == IAC) {
                /* Escaped IAC inside SB. */
                if (g_sb_len < sizeof(g_sb_buf))
                    g_sb_buf[g_sb_len++] = IAC;
                g_state = TS_SB;
            } else {
                /* Non-standard sequence: leave SB. */
                g_state = TS_DATA;
            }
            break;
        }
    }
}

/* ---------- Reset pulse (RTS+DTR are released for 200 ms) ---------- */

static void send_reset_pulse(void)
{
    fputs("\r\n", stderr);
    log_message(LOG_INFO, "[reset] RTS+DTR off for 200 ms");

    /* Release DTR and RTS to reset the device. */
    comport_set_control(CPC_CTRL_DTR_OFF);
    comport_set_control(CPC_CTRL_RTS_OFF);

    /* Hold for 200 ms. */
    sleep_us(200 * 1000);

    /* Return to active state so the device exits reset. */
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
        if (connect(s, ai->ai_addr, ai->ai_addrlen) == 0)
            break;
        close(s);
        s = -1;
    }
    freeaddrinfo(res);

    if (s < 0)
        log_message(LOG_ERR, "Unable to connect to %s:%s", host, port);
    return s;
}

/* ---------- CLI parser ---------- */

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s -h <host> -p <port> [options]\n"
        "\n"
        "Required:\n"
        "  -h HOST         RFC 2217 server address\n"
        "  -p PORT         TCP port\n"
        "\n"
        "Port settings (default 115200 8N1):\n"
        "  -b BAUD         baud rate (default 115200)\n"
        "  -d 5|6|7|8      data bits (default 8)\n"
        "  -s 1|2          stop bits (default 1)\n"
        "  -y N|E|O        parity: None/Even/Odd (default N)\n"
        "  --help, -?      this help\n"
        "\n"
        "Examples:\n"
        "  %s -h 192.168.1.10 -p 7000                    # 115200 8N1\n"
        "  %s -h 192.168.1.10 -p 7000 -b 9600 -y E       # 9600 8E1\n"
        "\n"
        "Session controls:\n"
        "  Ctrl+P   RTS+DTR pulse (200 ms) — reset the target device\n"
        "  Ctrl+]   exit\n",
        prog, prog, prog);
}

int main(int argc, char **argv)
{
    const char *host = NULL;
    const char *port = NULL;
    uint32_t baud = 115200;
    int data_bits = 8;
    int stop_bits = 1;
    char parity = 'N';

    openlog(g_progname, LOG_PID | LOG_NDELAY, LOG_DAEMON);
    atexit(closelog);

    /* Support --help outside getopt. */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        }
    }

    int opt;
    while ((opt = getopt(argc, argv, "h:p:b:d:s:y:?")) != -1) {
        switch (opt) {
        case 'h': host = optarg; break;
        case 'p': port = optarg; break;
        case 'b': baud = (uint32_t)strtoul(optarg, NULL, 10); break;
        case 'd': data_bits = atoi(optarg); break;
        case 's': stop_bits = atoi(optarg); break;
        case 'y': parity = (char)toupper((unsigned char)optarg[0]); break;
        case '?':
            if (optopt == 0) {
                usage(argv[0]);
                return 0;
            }
            usage(argv[0]);
            return 1;
        default:
            usage(argv[0]);
            return 1;
        }
    }

    if (!host || !port) {
        log_message(LOG_ERR, "Missing -h <host> and/or -p <port>");
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

    /* Connect. */
    g_sock = connect_to(host, port);
    if (g_sock < 0)
        return 1;

    atexit(cleanup);
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    log_message(LOG_INFO, "Connected to %s:%s. baud=%u, data=%d, stop=%d, parity=%c",
        host, port, baud, data_bits, stop_bits, parity);
    log_message(LOG_INFO, "Ctrl+P resets RTS+DTR for 200 ms, Ctrl+] exits");

    /* Request negotiations with the server. */
    telnet_send_negot(WILL, TELOPT_BINARY);
    telnet_send_negot(DO,   TELOPT_BINARY);
    telnet_send_negot(WILL, TELOPT_SGA);
    telnet_send_negot(DO,   TELOPT_SGA);
    telnet_send_negot(WILL, TELOPT_COMPORT);

    /* Configure the port parameters.
       A small delay after WILL COM-PORT helps some servers reply with DO first. */
    sleep_us(100 * 1000);

    comport_set_baudrate(baud);
    comport_set_datasize((uint8_t)data_bits);
    comport_set_stop((uint8_t)stop_bits);
    comport_set_parity(parity_v);
    /* No hardware flow control by default. */
    comport_set_control(CPC_CTRL_NO_FLOW);
    /* Bring lines into the active state so the device runs normally.
       This also keeps Ctrl+P working: the reset pulse drops the lines for
       200 ms and then restores them here. */
    comport_set_control(CPC_CTRL_DTR_ON);
    comport_set_control(CPC_CTRL_RTS_ON);

    /* Switch the terminal to raw mode. */
    set_raw_terminal();

    /* Main loop: select on socket and stdin. */
    uint8_t buf[4096];
    while (!g_quit) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(g_sock, &rfds);
        FD_SET(STDIN_FILENO, &rfds);
        int maxfd = g_sock > STDIN_FILENO ? g_sock : STDIN_FILENO;

        int r = select(maxfd + 1, &rfds, NULL, NULL, NULL);
        if (r < 0) {
            if (errno == EINTR)
                continue;
            log_errno(LOG_ERR, "select");
            break;
        }

        /* Data from the server. */
        if (FD_ISSET(g_sock, &rfds)) {
            ssize_t n = read(g_sock, buf, sizeof buf);
            if (n <= 0) {
                log_message(LOG_INFO, "Connection closed");
                break;
            }
            process_incoming(buf, (size_t)n);
        }

        /* Keyboard input. */
        if (FD_ISSET(STDIN_FILENO, &rfds)) {
            ssize_t n = read(STDIN_FILENO, buf, sizeof buf);
            if (n <= 0)
                break;

            /* Walk the input buffer: handle special keys,
               send the rest to the server with IAC escaping. */
            for (ssize_t i = 0; i < n; i++) {
                uint8_t c = buf[i];

                if (c == 0x10) { /* Ctrl+P -> reset pulse */
                    send_reset_pulse();
                    continue;
                }
                if (c == 0x1D) { /* Ctrl+] -> exit */
                    g_quit = 1;
                    break;
                }

                if (c == IAC) {
                    /* IAC IAC */
                    uint8_t pair[2] = { IAC, IAC };
                    if (write_all(g_sock, pair, 2) < 0) {
                        g_quit = 1;
                        break;
                    }
                } else {
                    if (write_all(g_sock, &c, 1) < 0) {
                        g_quit = 1;
                        break;
                    }
                }
            }
        }
    }

    log_message(LOG_INFO, "Exiting");
    return 0;
}
