/*
 *
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * vectis.c — UART terminal with RTS/DTR reset pulse support
 *
 */

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <syslog.h>
#include <linux/serial.h>
#include <termios.h>
#include <signal.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <time.h>  /* clock_gettime */

#define BUFFER_SIZE 4096
#define RESET_PULSE_MS 200
#define DEFAULT_TCP_PORT 35240
#define PROGRAM_VERSION "1.2.0"
#define PROGRAM_RELEASE_DATE "2026-05-01"

/* --- RFC 854 / RFC 2217 protocol constants --- */
#define TN_IAC_BYTE   255
#define TN_DONT       254
#define TN_DO         253
#define TN_WONT       252
#define TN_WILL       251
#define TN_SB         250
#define TN_SE         240

#define TN_OPT_BINARY  0   /* RFC 856 */
#define TN_OPT_SGA     3   /* RFC 858 */
#define TN_OPT_COMPORT 44  /* RFC 2217 */

#define COMPORT_SIGNATURE     0
#define COMPORT_SET_BAUDRATE  1
#define COMPORT_SET_DATASIZE  2
#define COMPORT_SET_PARITY    3
#define COMPORT_SET_STOPSIZE  4
#define COMPORT_SET_CONTROL   5
#define COMPORT_PURGE_DATA    12

/* Vendor extension (50): catch the HiSilicon boot ROM locally.
 *
 * RFC 2217 sub-options 1..12 are standard; 50 is well outside that
 * range and unallocated.  Clients that don't know about it are
 * unaffected (the server only emits a (50+100=150) reply when the
 * client asks).
 *
 * The flow handles a fundamental high-RTT problem: the HiSilicon
 * boot ROM emits 0x20 markers and listens for 0xAA in a ~100 ms
 * window after reset.  Over a network with one-way latency above
 * ~25 ms the client cannot complete a marker→ack round trip in
 * time.  Running the catch loop locally on Vectis (next to the
 * UART, microsecond response) makes it work regardless of link
 * RTT or jitter.
 *
 * Wire format (client → server):
 *     IAC SB COMPORT BOOTROM_CATCH
 *         <pulse_ms_4_BE> <max_wait_ms_4_BE>
 *     IAC SE
 *
 * Wire format (server → client, sub-option 150):
 *     IAC SB COMPORT (BOOTROM_CATCH+100) <status>
 *     IAC SE
 *
 *     status:
 *       0 = caught — boot ROM is in serial-download mode
 *       1 = timeout — no markers within max_wait_ms
 *       2 = io error
 */
#define COMPORT_BOOTROM_CATCH 50

#define BOOTROM_STATUS_OK      0
#define BOOTROM_STATUS_TIMEOUT 1
#define BOOTROM_STATUS_IO_ERR  2

#define BOOTROM_MARKER         0x20
#define BOOTROM_ACK            0xaa
#define BOOTROM_MARKER_COUNT   5

/* Optional 9th payload byte selects how to detect "boot ROM is in
 * download mode":
 *
 *   0 = MARKER (default, current behaviour) — wait for 5 consecutive
 *       0x20 markers from the chip and then send a final 0xAA.  This
 *       matches the behaviour documented for typical hi3516cv* / ev*
 *       bootroms.
 *
 *   1 = BLIND  — keep blasting 0xAA for the full ``max_wait_ms`` and
 *       always return ``OK``.  Some camera variants enter download
 *       mode silently on receiving 0xAA without ever emitting markers
 *       we can observe; for those, marker mode times out even though
 *       the bootrom is in fact ready.  In blind mode the client
 *       confirms by sending a HEAD frame and watching for an ACK.
 *
 * Older clients that send only 8 bytes of payload get MARKER mode
 * automatically — no behaviour change. */
#define BOOTROM_MODE_MARKER 0
#define BOOTROM_MODE_BLIND  1

#define COMPORT_SERVER_OFFSET 100  /* server replies use sub-opt + 100 */

/* Fixed line parameters reported to RFC 2217 clients.  Vectis is
 * hard-wired 8N1 (see configure_uart), so we always advertise that.
 * The values below are the on-the-wire RFC 2217 codes. */
#define COMPORT_DATASIZE_8     8   /* 8 data bits */
#define COMPORT_PARITY_NONE    1   /* no parity */
#define COMPORT_STOPSIZE_1     1   /* 1 stop bit */

/* PURGE-DATA values (RFC 2217 §3.7). */
#define COMPORT_PURGE_RX       1
#define COMPORT_PURGE_TX       2
#define COMPORT_PURGE_BOTH     3

#define COMPORT_CTRL_REQUEST 0
#define COMPORT_CTRL_DTR_ON  8
#define COMPORT_CTRL_DTR_OFF 9
/* Per RFC 2217 §3.5: value 10 is "Request RTS Signal State"; 11/12
 * set the line.  Earlier Vectis versions used 10/11 for ON/OFF, which
 * silently misrouted pyserial's `ser.rts = True` (sub-opt 11) onto
 * the deassert path.  These constants now match the standard. */
#define COMPORT_CTRL_RTS_REQ 10
#define COMPORT_CTRL_RTS_ON  11
#define COMPORT_CTRL_RTS_OFF 12

// Global variables
volatile sig_atomic_t running = 1;
int fd = -1;
struct termios orig_termios;
struct termios orig_uart_termios;
static int terminal_configured = 0;
static int uart_configured = 0;
static int tcp_listen_fd = -1;
static int tcp_client_fd = -1;
static char tcp_echo_buffer[BUFFER_SIZE];
static size_t tcp_echo_len = 0;

/* Inetd integration mode: when started by inetd in `nowait` form,
 * stdin/stdout *is* the TCP socket (one process per connection).  In
 * this mode we apply RFC 2217 detection on stdin and route Telnet
 * replies to stdout, mirroring the standalone `-t` listener behaviour.
 *
 * Detection: stdin is not a tty AND no `-t` listener was requested.
 * The interactive local-console mode (isatty stdin) is unchanged. */
static int inetd_mode = 0;

/* --- RFC 2217 / Telnet session state for the TCP client ---
 *
 * Vectis stays in legacy raw mode (Ctrl+P intercept, CRLF normalisation,
 * echo suppression) until the client transmits a Telnet IAC byte.  At
 * that point we lock into Telnet mode for the rest of the connection,
 * announce BINARY + SUPPRESS-GO-AHEAD + COM-PORT-OPTION, and process
 * SET-CONTROL / SET-BAUDRATE sub-negotiations.
 *
 * The detection rule is safe because:
 *   - Interactive humans never type 0xFF (it's an undefined Latin-1
 *     byte; all keyboards produce printable ASCII or named control
 *     codes like Ctrl+C/Ctrl+P).
 *   - Programmatic RFC 2217 clients (pyserial's rfc2217:// transport,
 *     ser2net, picocom --rfc2217, plain `telnet`) always send IAC at
 *     connect time during option negotiation.
 *
 * Existing socat / nc / cat workflows therefore stay legacy and
 * continue to fire the Ctrl+P pulse on a single 0x10.
 */
typedef enum {
    TN_STATE_DATA = 0,
    TN_STATE_IAC,
    TN_STATE_NEG,      /* saw IAC WILL/WONT/DO/DONT, awaiting option byte */
    TN_STATE_SB_OPT,   /* saw IAC SB, awaiting option byte */
    TN_STATE_SB_DATA,  /* collecting sub-negotiation parameters */
    TN_STATE_SB_IAC,   /* saw IAC inside SB; expecting SE or IAC */
} tn_state_t;

static struct {
    int telnet;             /* 0 = legacy mode, 1 = Telnet/RFC 2217 mode */
    tn_state_t state;
    int neg_cmd;            /* WILL/WONT/DO/DONT being processed */
    int sb_opt;
    unsigned char sb_buf[64];
    size_t sb_len;

    /* Negotiated options.  *_local = "we WILL X", *_remote = "they WILL X". */
    int binary_local, binary_remote;
    int sga_local, sga_remote;
    int comport_local, comport_remote;
} tn = { 0 };

static void generate_reset_pulse(void);
void set_signal_state(int signal_flag, int active);
static int set_uart_baudrate(int target);

/* BOOTROM-CATCH result with diagnostic counters; see definition near
 * COMPORT_BOOTROM_CATCH for the wire format. */
struct bootrom_catch_result {
    int      status;          /* BOOTROM_STATUS_* */
    uint32_t markers_seen;    /* total 0x20 bytes counted */
    uint8_t  max_marker_run;  /* longest consecutive 0x20 run (capped 255) */
    uint32_t bytes_rx;        /* total bytes received from UART */
    uint32_t bytes_tx;        /* total 0xAA bytes written to UART */
    uint32_t elapsed_ms;      /* actual catch-loop runtime */
    uint8_t  last_byte;       /* last non-marker byte (0 if none) */
};

static void bootrom_catch_local(int pulse_ms, int max_wait_ms, int mode,
                                struct bootrom_catch_result *out);

static void log_message(int priority, FILE *stream, const char *fmt, ...)
{
    char buffer[512];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, ap);
    va_end(ap);

    syslog(priority, "%s", buffer);
    fprintf(stream, "%s\n", buffer);
}

static void log_errno_message(const char *context)
{
    int saved_errno = errno;

    syslog(LOG_ERR, "%s: %s", context, strerror(saved_errno));
    fprintf(stderr, "%s: %s\n", context, strerror(saved_errno));
}

static void close_tcp_client(void)
{
    if (tcp_client_fd != -1) {
        close(tcp_client_fd);
        tcp_client_fd = -1;
    }
    tcp_echo_len = 0;
    memset(&tn, 0, sizeof(tn));
}

static void close_tcp_listener(void)
{
    if (tcp_listen_fd != -1) {
        close(tcp_listen_fd);
        tcp_listen_fd = -1;
    }
}

// Restore terminal settings on exit.
void restore_terminal() {
    if (terminal_configured) {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
        terminal_configured = 0;
    }
}

// Restore UART settings on exit.
void restore_uart() {
    if (uart_configured && fd != -1) {
        tcsetattr(fd, TCSANOW, &orig_uart_termios);
        uart_configured = 0;
    }
}

// Signal handler.
void signal_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) {
        ssize_t ignored;
        running = 0;
        ignored = write(STDOUT_FILENO, "\n\nExiting the program...\n", sizeof("\n\nExiting the program...\n") - 1);
        (void)ignored;
    }
}

// Configure the terminal for raw input.
int setup_terminal() {
    struct termios new_termios;

    if (!isatty(STDIN_FILENO)) {
        return 0;
    }
    
    // Read current terminal settings.
    if (tcgetattr(STDIN_FILENO, &orig_termios) != 0) {
        log_errno_message("tcgetattr(STDIN_FILENO)");
        return -1;
    }
    new_termios = orig_termios;
    
    // Disable canonical mode and echo, but keep signal generation enabled.
    new_termios.c_lflag &= ~(ICANON | ECHO);
    new_termios.c_iflag &= ~(IXON | IXOFF | IXANY);
    new_termios.c_cc[VMIN] = 1;
    new_termios.c_cc[VTIME] = 0;
    
    // Apply the settings.
    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_termios) != 0) {
        log_errno_message("tcsetattr(STDIN_FILENO)");
        return -1;
    }
    terminal_configured = 1;
    
    return 0;
}

// Configure the serial port.
int configure_uart(int fd, int baudrate) {
    struct termios tty;
    
    // Save the original settings.
    if (tcgetattr(fd, &orig_uart_termios) != 0) {
        log_errno_message("tcgetattr");
        return -1;
    }
    
    tty = orig_uart_termios;
    
    // Set the baud rate.
    speed_t speed;
    switch(baudrate) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default:
            log_message(LOG_ERR, stderr, "Unsupported baud rate: %d", baudrate);
            return -1;
    }
    
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    
    // 8 data bits, no parity, 1 stop bit (8N1)
    tty.c_cflag &= ~PARENB;  // no parity
    tty.c_cflag &= ~CSTOPB;  // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      // 8 data bits
    
    // Enable receiver
    tty.c_cflag |= CREAD | CLOCAL;
    
    // Disable hardware flow control
    #ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
    #endif
    
    // Disable software flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    
    // Raw input mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Raw output mode
    tty.c_oflag &= ~OPOST;
    
    // Configure timeouts
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;
    
    // Flush input buffers.
    if (tcflush(fd, TCIFLUSH) != 0) {
        log_errno_message("tcflush");
        return -1;
    }

    // Apply the settings.
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        log_errno_message("tcsetattr");
        return -1;
    }
    uart_configured = 1;
    
    return 0;
}

// Initialize RTS and DTR to the asserted state.
void init_signals() {
    if (fd == -1) return;
    
    int rts_flag = TIOCM_RTS;
    int dtr_flag = TIOCM_DTR;
    int ok = 1;

    // Set RTS to the asserted state (set the bit).
    if (ioctl(fd, TIOCMBIS, &rts_flag) == -1) {
        log_errno_message("Failed to assert RTS");
        ok = 0;
    }
    
    // Set DTR to the asserted state (set the bit).
    if (ioctl(fd, TIOCMBIS, &dtr_flag) == -1) {
        log_errno_message("Failed to assert DTR");
        ok = 0;
    }
    
    printf("Signals initialized:\n");
    printf("  RTS: asserted\n");
    printf("  DTR: asserted\n");
    if (ok) {
        syslog(LOG_INFO, "RTS and DTR initialized to asserted state");
    }
}

static int write_all(int out_fd, const char *data, size_t len)
{
    size_t total = 0;

    while (total < len) {
        ssize_t written = write(out_fd, data + total, len - total);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        total += (size_t)written;
    }

    return 0;
}

static int write_all_socket(int sock_fd, const char *data, size_t len)
{
    size_t total = 0;

    while (total < len) {
        ssize_t written = send(sock_fd, data + total, len - total, MSG_NOSIGNAL);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        total += (size_t)written;
    }

    return 0;
}

static void set_tcp_echo_suppression(const char *data, size_t len)
{
    if (len > sizeof(tcp_echo_buffer)) {
        len = sizeof(tcp_echo_buffer);
    }

    if (len == 0) {
        tcp_echo_len = 0;
        return;
    }

    memcpy(tcp_echo_buffer, data, len);
    tcp_echo_len = len;
}

static size_t filter_input_buffer(const char *src, size_t len, char *dst, size_t dst_size, int allow_exit_keys,
                                  int *reset_requested, int *exit_requested)
{
    size_t out = 0;

    if (reset_requested != NULL) {
        *reset_requested = 0;
    }
    if (exit_requested != NULL) {
        *exit_requested = 0;
    }

    for (size_t i = 0; i < len; i++) {
        unsigned char ch = (unsigned char)src[i];

        if (ch == 16) {
            if (reset_requested != NULL) {
                *reset_requested = 1;
            }
            continue;
        }

        if (allow_exit_keys && (ch == 3 || ch == 24)) {
            if (exit_requested != NULL) {
                *exit_requested = 1;
            }
            continue;
        }

        if (out < dst_size) {
            dst[out++] = (char)ch;
        }
    }

    return out;
}

static size_t normalize_tcp_input(const char *src, size_t len, char *dst, size_t dst_size)
{
    size_t out = 0;

    for (size_t i = 0; i < len && out < dst_size; i++) {
        if (src[i] == '\r') {
            dst[out++] = '\r';
            if (i + 1 < len && src[i + 1] == '\n') {
                i++;
            }
        } else if (src[i] == '\n') {
            dst[out++] = '\r';
        } else {
            dst[out++] = src[i];
        }
    }

    return out;
}

static size_t consume_tcp_echo_prefix(const char *data, size_t len)
{
    size_t match = 0;

    if (tcp_echo_len == 0 || len == 0) {
        return 0;
    }

    while (match < len && match < tcp_echo_len && data[match] == tcp_echo_buffer[match]) {
        match++;
    }

    if (match == 0) {
        tcp_echo_len = 0;
        return 0;
    }

    if (match == tcp_echo_len) {
        tcp_echo_len = 0;
    } else {
        memmove(tcp_echo_buffer, tcp_echo_buffer + match, tcp_echo_len - match);
        tcp_echo_len -= match;
    }

    return match;
}

static int parse_port(const char *text, int *port)
{
    char *end = NULL;
    long value;

    errno = 0;
    value = strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || value < 1 || value > 65535) {
        return -1;
    }

    *port = (int)value;
    return 0;
}

static int setup_tcp_listener(int port)
{
    int listen_fd;
    int yes = 1;
    struct sockaddr_in addr;

    listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd == -1) {
        log_errno_message("socket");
        return -1;
    }

    if (setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) != 0) {
        log_errno_message("setsockopt(SO_REUSEADDR)");
        close(listen_fd);
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);

    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        log_errno_message("bind");
        close(listen_fd);
        return -1;
    }

    if (listen(listen_fd, 1) != 0) {
        log_errno_message("listen");
        close(listen_fd);
        return -1;
    }

    tcp_listen_fd = listen_fd;
    return 0;
}

static void accept_tcp_client(void)
{
    struct sockaddr_in peer;
    socklen_t peer_len = sizeof(peer);
    int client_fd;
    char peer_addr[INET_ADDRSTRLEN];

    client_fd = accept(tcp_listen_fd, (struct sockaddr *)&peer, &peer_len);
    if (client_fd == -1) {
        if (errno != EINTR && errno != EAGAIN) {
            log_errno_message("accept");
        }
        return;
    }

    if (inet_ntop(AF_INET, &peer.sin_addr, peer_addr, sizeof(peer_addr)) == NULL) {
        snprintf(peer_addr, sizeof(peer_addr), "unknown");
    }

    close_tcp_client();
    tcp_client_fd = client_fd;
    syslog(LOG_INFO, "TCP client connected from %s:%u", peer_addr, (unsigned)ntohs(peer.sin_port));
    printf("[TCP] Client connected from %s:%u\n", peer_addr, (unsigned)ntohs(peer.sin_port));
}

/* --- RFC 2217 helpers --- */

static void tn_send(const unsigned char *bytes, size_t len)
{
    if (tcp_client_fd != -1) {
        write_all_socket(tcp_client_fd, (const char *)bytes, len);
    } else if (inetd_mode) {
        /* In inetd nowait mode stdout is the TCP socket. */
        write_all(STDOUT_FILENO, (const char *)bytes, len);
    }
}

static void tn_send_neg(unsigned char cmd, unsigned char opt)
{
    unsigned char buf[3] = { TN_IAC_BYTE, cmd, opt };
    tn_send(buf, sizeof(buf));
}

static void tn_announce_options(void)
{
    /* Proactively offer BINARY (both directions), SUPPRESS-GO-AHEAD,
     * and COM-PORT-OPTION.  The client confirms what it wants. */
    tn_send_neg(TN_WILL, TN_OPT_BINARY);
    tn_send_neg(TN_DO,   TN_OPT_BINARY);
    tn_send_neg(TN_WILL, TN_OPT_SGA);
    tn_send_neg(TN_DO,   TN_OPT_SGA);
    tn_send_neg(TN_WILL, TN_OPT_COMPORT);
    tn_send_neg(TN_DO,   TN_OPT_COMPORT);
    syslog(LOG_INFO, "Telnet/RFC 2217 mode entered; announced BINARY+SGA+COM-PORT-OPTION");
    printf("[TCP] Telnet/RFC 2217 mode\n");
    fflush(stdout);
}

static void tn_handle_neg(int cmd, int opt)
{
    int *local = NULL;   /* tracks WILL state on our (server) side */
    int *remote = NULL;  /* tracks WILL state on the client side */

    switch (opt) {
    case TN_OPT_BINARY:  local = &tn.binary_local;  remote = &tn.binary_remote;  break;
    case TN_OPT_SGA:     local = &tn.sga_local;     remote = &tn.sga_remote;     break;
    case TN_OPT_COMPORT: local = &tn.comport_local; remote = &tn.comport_remote; break;
    default:
        /* Refuse anything else — Q method: respond once, no loop. */
        if (cmd == TN_WILL) tn_send_neg(TN_DONT, (unsigned char)opt);
        else if (cmd == TN_DO) tn_send_neg(TN_WONT, (unsigned char)opt);
        return;
    }

    switch (cmd) {
    case TN_WILL:  /* peer will send option — accept iff state changes */
        if (!*remote) { *remote = 1; tn_send_neg(TN_DO, (unsigned char)opt); }
        break;
    case TN_WONT:
        if (*remote) { *remote = 0; tn_send_neg(TN_DONT, (unsigned char)opt); }
        break;
    case TN_DO:    /* peer wants us to send option */
        if (!*local) { *local = 1; tn_send_neg(TN_WILL, (unsigned char)opt); }
        break;
    case TN_DONT:
        if (*local) { *local = 0; tn_send_neg(TN_WONT, (unsigned char)opt); }
        break;
    }
}

static void tn_send_set_control_reply(unsigned char value)
{
    unsigned char buf[7] = {
        TN_IAC_BYTE, TN_SB, TN_OPT_COMPORT,
        COMPORT_SET_CONTROL + COMPORT_SERVER_OFFSET, value,
        TN_IAC_BYTE, TN_SE,
    };
    tn_send(buf, sizeof(buf));
}

static void tn_send_set_baudrate_reply(uint32_t baud)
{
    unsigned char buf[11] = {
        TN_IAC_BYTE, TN_SB, TN_OPT_COMPORT,
        COMPORT_SET_BAUDRATE + COMPORT_SERVER_OFFSET,
        (unsigned char)(baud >> 24), (unsigned char)(baud >> 16),
        (unsigned char)(baud >> 8),  (unsigned char)baud,
        TN_IAC_BYTE, TN_SE,
    };
    tn_send(buf, sizeof(buf));
}

/* Single-byte port-parameter reply (DATASIZE/PARITY/STOPSIZE follow
 * the same wire shape: IAC SB COMPORT (subopt+100) value IAC SE). */
static void tn_send_byte_param_reply(unsigned char subopt, unsigned char value)
{
    unsigned char buf[7] = {
        TN_IAC_BYTE, TN_SB, TN_OPT_COMPORT,
        (unsigned char)(subopt + COMPORT_SERVER_OFFSET), value,
        TN_IAC_BYTE, TN_SE,
    };
    tn_send(buf, sizeof(buf));
}

static int current_uart_baud(void)
{
    if (fd == -1) return 0;
    struct termios t;
    if (tcgetattr(fd, &t) != 0) return 0;
    speed_t s = cfgetospeed(&t);
    switch (s) {
    case B9600:   return 9600;
    case B19200:  return 19200;
    case B38400:  return 38400;
    case B57600:  return 57600;
    case B115200: return 115200;
    case B230400: return 230400;
    default:      return 0;
    }
}

static void tn_handle_subneg(int opt, const unsigned char *data, size_t len)
{
    if (opt != TN_OPT_COMPORT || len < 1) {
        return;
    }
    unsigned char subopt = data[0];

    switch (subopt) {
    case COMPORT_SET_CONTROL: {
        if (len < 2) return;
        unsigned char value = data[1];
        switch (value) {
        case COMPORT_CTRL_DTR_ON:  set_signal_state(TIOCM_DTR, 1); break;
        case COMPORT_CTRL_DTR_OFF: set_signal_state(TIOCM_DTR, 0); break;
        case COMPORT_CTRL_RTS_ON:  set_signal_state(TIOCM_RTS, 1); break;
        case COMPORT_CTRL_RTS_OFF: set_signal_state(TIOCM_RTS, 0); break;
        case COMPORT_CTRL_REQUEST: {
            /* Request DTR state (sub-value 7 in the spec collapses to
             * 0 = "Request Com Port Control" in our minimal mapping). */
            int status = 0;
            if (fd != -1) ioctl(fd, TIOCMGET, &status);
            value = (status & TIOCM_DTR) ? COMPORT_CTRL_DTR_ON : COMPORT_CTRL_DTR_OFF;
            break;
        }
        case COMPORT_CTRL_RTS_REQ: {
            int status = 0;
            if (fd != -1) ioctl(fd, TIOCMGET, &status);
            value = (status & TIOCM_RTS) ? COMPORT_CTRL_RTS_ON : COMPORT_CTRL_RTS_OFF;
            break;
        }
        default:
            /* Unsupported control (flow control etc.) — ack the value
             * we received so the client doesn't hang waiting. */
            break;
        }
        tn_send_set_control_reply(value);
        break;
    }
    case COMPORT_SET_BAUDRATE: {
        if (len < 5) return;
        uint32_t baud = ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) |
                        ((uint32_t)data[3] << 8)  |  (uint32_t)data[4];
        if (baud != 0) {
            if (set_uart_baudrate((int)baud) != 0) {
                /* Couldn't set — reply with current to inform the client. */
                baud = (uint32_t)current_uart_baud();
            }
        } else {
            baud = (uint32_t)current_uart_baud();
        }
        tn_send_set_baudrate_reply(baud);
        break;
    }
    case COMPORT_SET_DATASIZE:
    case COMPORT_SET_PARITY:
    case COMPORT_SET_STOPSIZE: {
        /* Vectis is hard-wired to 8N1 by configure_uart(), so the
         * reply is the same regardless of the client's request: we
         * report the value actually in effect (RFC 2217 §3.2-§3.4 say
         * the server replies with what was applied, not what was
         * asked for).  This unblocks pyserial's rfc2217:// transport,
         * which sets DATASIZE/PARITY/STOPSIZE at startup and waits
         * synchronously for an ack.  A request value of 0 ("query
         * current") produces the same answer. */
        unsigned char reply;
        switch (subopt) {
        case COMPORT_SET_DATASIZE: reply = COMPORT_DATASIZE_8;  break;
        case COMPORT_SET_PARITY:   reply = COMPORT_PARITY_NONE; break;
        case COMPORT_SET_STOPSIZE: reply = COMPORT_STOPSIZE_1;  break;
        default:                   reply = 0; break;  /* unreachable */
        }
        tn_send_byte_param_reply((unsigned char)subopt, reply);
        break;
    }
    case COMPORT_PURGE_DATA: {
        /* RFC 2217 §3.7: 1=purge RX, 2=purge TX, 3=purge both.  pyserial
         * calls this from reset_input_buffer() / reset_output_buffer()
         * during open() and waits synchronously for the ack — without
         * it the connection times out before any data flows. */
        if (len < 2) return;
        unsigned char value = data[1];
        if (fd != -1) {
            int how = -1;
            switch (value) {
            case COMPORT_PURGE_RX:   how = TCIFLUSH;  break;
            case COMPORT_PURGE_TX:   how = TCOFLUSH;  break;
            case COMPORT_PURGE_BOTH: how = TCIOFLUSH; break;
            default: break;
            }
            if (how != -1) {
                tcflush(fd, how);
            }
        }
        tn_send_byte_param_reply((unsigned char)subopt, value);
        break;
    }
    case COMPORT_BOOTROM_CATCH: {
        /* Vendor extension: pulse RTS+DTR for ``pulse_ms``, then run
         * the HiSilicon bootrom 0x20→0xAA handshake locally so the
         * client doesn't have to round-trip every byte across the
         * link.  See COMPORT_BOOTROM_CATCH definition for the wire
         * format. */
        if (len < 9) return;  /* 1 sub-opt + 4+4 BE arguments */
        uint32_t pulse_ms = ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) |
                            ((uint32_t)data[3] << 8)  |  (uint32_t)data[4];
        uint32_t max_wait_ms = ((uint32_t)data[5] << 24) | ((uint32_t)data[6] << 16) |
                               ((uint32_t)data[7] << 8)  |  (uint32_t)data[8];
        /* Optional mode byte (0 = MARKER, 1 = BLIND).  Older clients
         * that send only 8 payload bytes default to MARKER mode. */
        int mode = (len >= 10) ? (int)data[9] : BOOTROM_MODE_MARKER;
        if (mode != BOOTROM_MODE_MARKER && mode != BOOTROM_MODE_BLIND) {
            mode = BOOTROM_MODE_MARKER;
        }
        /* Clamp to sane bounds so a bad client can't pin the server
         * indefinitely or pulse for an unreasonable duration. */
        if (pulse_ms > 5000)    pulse_ms = 5000;
        if (max_wait_ms > 30000) max_wait_ms = 30000;
        if (max_wait_ms < 100)   max_wait_ms = 100;

        syslog(LOG_INFO,
               "bootrom_catch requested: pulse=%u ms, max_wait=%u ms, mode=%d",
               pulse_ms, max_wait_ms, mode);
        struct bootrom_catch_result r;
        bootrom_catch_local((int)pulse_ms, (int)max_wait_ms, mode, &r);

        /* Reply payload: status (1) + markers_seen (4 BE) + max_run (1)
         * + bytes_rx (4 BE) + bytes_tx (4 BE) + elapsed_ms (4 BE) +
         * last_byte (1) = 19 bytes after the sub-option byte.  Older
         * clients that read just the first byte (status) are
         * unaffected; new clients introspect the trailing counters to
         * see what actually happened during the catch. */
        unsigned char reply[7 /* IAC SB COMPORT subopt */ + 19 + 2 /* IAC SE */];
        size_t p = 0;
        reply[p++] = TN_IAC_BYTE;
        reply[p++] = TN_SB;
        reply[p++] = TN_OPT_COMPORT;
        reply[p++] = (unsigned char)(subopt + COMPORT_SERVER_OFFSET);
        reply[p++] = (unsigned char)r.status;
        reply[p++] = (unsigned char)(r.markers_seen >> 24);
        reply[p++] = (unsigned char)(r.markers_seen >> 16);
        reply[p++] = (unsigned char)(r.markers_seen >> 8);
        reply[p++] = (unsigned char)(r.markers_seen);
        reply[p++] = r.max_marker_run;
        reply[p++] = (unsigned char)(r.bytes_rx >> 24);
        reply[p++] = (unsigned char)(r.bytes_rx >> 16);
        reply[p++] = (unsigned char)(r.bytes_rx >> 8);
        reply[p++] = (unsigned char)(r.bytes_rx);
        reply[p++] = (unsigned char)(r.bytes_tx >> 24);
        reply[p++] = (unsigned char)(r.bytes_tx >> 16);
        reply[p++] = (unsigned char)(r.bytes_tx >> 8);
        reply[p++] = (unsigned char)(r.bytes_tx);
        reply[p++] = (unsigned char)(r.elapsed_ms >> 24);
        reply[p++] = (unsigned char)(r.elapsed_ms >> 16);
        reply[p++] = (unsigned char)(r.elapsed_ms >> 8);
        reply[p++] = (unsigned char)(r.elapsed_ms);
        reply[p++] = r.last_byte;
        reply[p++] = TN_IAC_BYTE;
        reply[p++] = TN_SE;
        tn_send(reply, p);
        break;
    }
    case COMPORT_SIGNATURE: {
        /* RFC 2217 §3.5: server may answer with its own signature. */
        const char sig[] = "Vectis " PROGRAM_VERSION;
        unsigned char hdr[4] = {
            TN_IAC_BYTE, TN_SB, TN_OPT_COMPORT,
            COMPORT_SIGNATURE + COMPORT_SERVER_OFFSET,
        };
        unsigned char tail[2] = { TN_IAC_BYTE, TN_SE };
        tn_send(hdr, sizeof(hdr));
        tn_send((const unsigned char *)sig, sizeof(sig) - 1);
        tn_send(tail, sizeof(tail));
        break;
    }
    default:
        /* Unsupported sub-option — silently ignore. */
        break;
    }
}

/* Run incoming TCP bytes through the Telnet state machine.  Extracted
 * payload bytes (with IAC IAC unescaped) land in out[]. */
static size_t tn_process(const unsigned char *in, size_t n,
                         unsigned char *out, size_t out_size)
{
    size_t out_pos = 0;

    for (size_t i = 0; i < n; i++) {
        unsigned char b = in[i];
        switch (tn.state) {
        case TN_STATE_DATA:
            if (b == TN_IAC_BYTE) {
                tn.state = TN_STATE_IAC;
            } else if (out_pos < out_size) {
                out[out_pos++] = b;
            }
            break;
        case TN_STATE_IAC:
            if (b == TN_IAC_BYTE) {
                if (out_pos < out_size) out[out_pos++] = TN_IAC_BYTE;
                tn.state = TN_STATE_DATA;
            } else if (b == TN_WILL || b == TN_WONT || b == TN_DO || b == TN_DONT) {
                tn.neg_cmd = b;
                tn.state = TN_STATE_NEG;
            } else if (b == TN_SB) {
                tn.state = TN_STATE_SB_OPT;
            } else {
                /* NOP/BRK/AYT/etc. — ignore. */
                tn.state = TN_STATE_DATA;
            }
            break;
        case TN_STATE_NEG:
            tn_handle_neg(tn.neg_cmd, b);
            tn.state = TN_STATE_DATA;
            break;
        case TN_STATE_SB_OPT:
            tn.sb_opt = b;
            tn.sb_len = 0;
            tn.state = TN_STATE_SB_DATA;
            break;
        case TN_STATE_SB_DATA:
            if (b == TN_IAC_BYTE) {
                tn.state = TN_STATE_SB_IAC;
            } else if (tn.sb_len < sizeof(tn.sb_buf)) {
                tn.sb_buf[tn.sb_len++] = b;
            }
            break;
        case TN_STATE_SB_IAC:
            if (b == TN_SE) {
                tn_handle_subneg(tn.sb_opt, tn.sb_buf, tn.sb_len);
                tn.state = TN_STATE_DATA;
            } else if (b == TN_IAC_BYTE) {
                if (tn.sb_len < sizeof(tn.sb_buf)) {
                    tn.sb_buf[tn.sb_len++] = TN_IAC_BYTE;
                }
                tn.state = TN_STATE_SB_DATA;
            } else {
                /* Bad escape inside SB — abort. */
                tn.state = TN_STATE_DATA;
            }
            break;
        }
    }

    return out_pos;
}

/* Process bytes received from a remote client (either the standalone
 * `-t` listener's accepted socket or, in inetd nowait mode, stdin).
 *
 * - Auto-detects the Telnet/RFC 2217 handshake on the first IAC byte
 *   (interactive `socat` / `nc` clients never send 0xFF, so they stay
 *   in legacy raw mode for the whole connection).
 * - In legacy mode applies the historic Ctrl+P intercept and CRLF
 *   normalisation for interactive use.  `allow_exit_keys` is true on
 *   the local-console stdin path so a human pressing Ctrl+C/X exits;
 *   it is false on the TCP path (closing the socket is the way out)
 *   and on the inetd-stdin path (same socket semantics).
 * - In Telnet mode runs bytes through the state machine, forwarding
 *   only payload bytes to UART.  The data path is binary safe.
 *
 * Returns 0 on success, -1 if an unrecoverable I/O error occurred. */
static int handle_remote_input(const unsigned char *buffer, size_t n,
                               int allow_exit_keys)
{
    int reset_requested = 0;
    int exit_requested = 0;
    char filtered[BUFFER_SIZE];
    char normalized[BUFFER_SIZE];

    if (!tn.telnet) {
        for (size_t i = 0; i < n; i++) {
            if (buffer[i] == TN_IAC_BYTE) { tn.telnet = 1; break; }
        }
        if (tn.telnet) {
            tn_announce_options();
        }
    }

    if (tn.telnet) {
        unsigned char data[BUFFER_SIZE];
        size_t data_len = tn_process(buffer, n, data, sizeof(data));
        if (data_len > 0) {
            if (write_all(fd, (const char *)data, data_len) != 0) {
                log_errno_message("Failed to write client input to UART");
                return -1;
            }
        }
        return 0;
    }

    /* Legacy path. */
    size_t filtered_len = filter_input_buffer((const char *)buffer, n,
                                              filtered, sizeof(filtered),
                                              allow_exit_keys,
                                              &reset_requested, &exit_requested);
    if (reset_requested) {
        generate_reset_pulse();
    }
    if (exit_requested) {
        running = 0;
        return 0;
    }
    if (filtered_len == 0) {
        return 0;
    }

    size_t normalized_len = normalize_tcp_input(filtered, filtered_len,
                                                normalized, sizeof(normalized));
    if (write_all(fd, normalized, normalized_len) != 0) {
        log_errno_message("Failed to write client input to UART");
        return -1;
    }
    /* Echo suppression is only meaningful on the standalone `-t`
     * listener — that's the path where stdout still receives the same
     * bytes locally and we don't want them to round-trip back to the
     * remote.  Skip it for inetd mode. */
    if (tcp_client_fd != -1) {
        set_tcp_echo_suppression(normalized, normalized_len);
    }
    return 0;
}

static void handle_tcp_client_input(void)
{
    unsigned char buffer[BUFFER_SIZE];
    ssize_t n = read(tcp_client_fd, buffer, sizeof(buffer));

    if (n > 0) {
        if (handle_remote_input(buffer, (size_t)n, 0) != 0) {
            running = 0;
        }
        return;
    }

    if (n == 0) {
        syslog(LOG_INFO, "TCP client disconnected");
        printf("[TCP] Client disconnected\n");
        close_tcp_client();
        return;
    }

    if (errno != EINTR && errno != EAGAIN) {
        log_errno_message("read(TCP client)");
        close_tcp_client();
    }
}

/* Forward UART bytes to the active client, escaping IAC bytes per
 * RFC 854 (IAC IAC = literal 0xFF).  Routes via tn_send so the same
 * code works for the standalone -t path (writes to tcp_client_fd) and
 * the inetd nowait path (writes to stdout). */
static void tn_send_uart_bytes(const char *data, size_t len)
{
    /* Worst case: every byte is 0xFF and gets doubled.  Stream out in
     * chunks so a long UART read doesn't need a 2*BUFFER_SIZE stack. */
    unsigned char chunk[BUFFER_SIZE];
    size_t out = 0;
    for (size_t i = 0; i < len; i++) {
        unsigned char b = (unsigned char)data[i];
        if (out + 2 > sizeof(chunk)) {
            tn_send(chunk, out);
            out = 0;
        }
        if (b == TN_IAC_BYTE) {
            chunk[out++] = TN_IAC_BYTE;
            chunk[out++] = TN_IAC_BYTE;
        } else {
            chunk[out++] = b;
        }
    }
    if (out > 0) {
        tn_send(chunk, out);
    }
}

static void forward_uart_output(const char *data, size_t len)
{
    if (len == 0) {
        return;
    }

    if (tn.telnet) {
        /* Binary-safe path: escape IAC and route to the active client.
         * In inetd mode this writes to stdout (the TCP socket); in
         * standalone -t mode this writes to tcp_client_fd.  No echo
         * suppression — RFC 2217 clients don't expect their input
         * mirrored back to them. */
        tn_send_uart_bytes(data, len);
        return;
    }

    /* Legacy path: write raw to stdout (local console OR inetd's
     * socket), and additionally to the standalone TCP client (with
     * echo suppression) when one is connected. */
    if (fwrite(data, 1, len, stdout) != len || fflush(stdout) != 0) {
        log_errno_message("stdout");
        running = 0;
        return;
    }

    if (tcp_client_fd == -1) {
        return;
    }

    size_t suppressed = consume_tcp_echo_prefix(data, len);
    if (suppressed > 0 && suppressed < len && data[suppressed - 1] == '\r' && data[suppressed] == '\n') {
        suppressed++;
    }
    const char *tcp_data = data + suppressed;
    size_t tcp_len = len - suppressed;

    if (tcp_len > 0 && write_all_socket(tcp_client_fd, tcp_data, tcp_len) != 0) {
        log_errno_message("Failed to write UART output to TCP client");
        syslog(LOG_INFO, "Closing TCP client after write failure");
        close_tcp_client();
    }
}

/* Translate an integer baud rate to the matching speed_t.  Returns -1
 * on unsupported values. */
static int speed_for_baud(int baud, speed_t *out)
{
    switch (baud) {
    case 9600:   *out = B9600;   return 0;
    case 19200:  *out = B19200;  return 0;
    case 38400:  *out = B38400;  return 0;
    case 57600:  *out = B57600;  return 0;
    case 115200: *out = B115200; return 0;
    case 230400: *out = B230400; return 0;
    default:                     return -1;
    }
}

static int set_uart_baudrate(int target)
{
    speed_t s;
    struct termios t;

    if (fd == -1 || speed_for_baud(target, &s) != 0) {
        return -1;
    }
    if (tcgetattr(fd, &t) != 0) {
        return -1;
    }
    cfsetispeed(&t, s);
    cfsetospeed(&t, s);
    if (tcsetattr(fd, TCSANOW, &t) != 0) {
        return -1;
    }
    syslog(LOG_INFO, "RFC 2217: UART baud rate changed to %d", target);
    return 0;
}

static int install_signal_handlers(void)
{
    struct sigaction sa;

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);

    if (sigaction(SIGINT, &sa, NULL) != 0) {
        log_errno_message("sigaction(SIGINT)");
        return -1;
    }

    if (sigaction(SIGTERM, &sa, NULL) != 0) {
        log_errno_message("sigaction(SIGTERM)");
        return -1;
    }

    return 0;
}

// Set a signal state directly.
void set_signal_state(int signal_flag, int active) {
    if (active) {
        // Active state: set the bit.
        if (ioctl(fd, TIOCMBIS, &signal_flag) == -1) {
            log_errno_message("Failed to activate signal");
        }
    } else {
        // Inactive state: clear the bit.
        if (ioctl(fd, TIOCMBIC, &signal_flag) == -1) {
            log_errno_message("Failed to deactivate signal");
        }
    }
}

// Generate an inverted reset pulse on RTS and DTR.
static void generate_reset_pulse(void) {
    if (fd == -1) return;

    int rts_flag = TIOCM_RTS;
    int dtr_flag = TIOCM_DTR;

    printf("[RESET] Pulsing RTS/DTR for %d ms\n", RESET_PULSE_MS);
    fflush(stdout);
    syslog(LOG_INFO, "Generating inverted reset pulse on RTS/DTR for %d ms", RESET_PULSE_MS);

    // Inverted pulse: deassert first, then assert.
    set_signal_state(rts_flag, 0);
    set_signal_state(dtr_flag, 0);

    // Delay.
    usleep(RESET_PULSE_MS * 1000);

    // Restore asserted state after the pulse.
    set_signal_state(rts_flag, 1);
    set_signal_state(dtr_flag, 1);

    syslog(LOG_INFO, "Inverted reset pulse finished");
}

/* Server-side HiSilicon bootrom catch.
 *
 * Reset the camera, then run the canonical 0x20-marker / 0xAA-ack
 * handshake locally on the UART so it isn't subject to TCP RTT.  Fills
 * ``out`` with the final status plus diagnostic counters that let the
 * client see what actually happened on the wire (markers seen, total
 * UART bytes, blast budget consumed, last non-marker byte).  The
 * counters are exposed in the reply payload so a high-RTT client can
 * tell why a catch failed without a separate trace mechanism.
 *
 * While running, the function owns the local UART exclusively: bytes
 * are not forwarded to the TCP client.  After return, normal
 * forwarding resumes — the client can read the ACK byte we already
 * deposited and proceed with HEAD/DATA frames against an in-mode boot
 * ROM.
 */
static void bootrom_catch_local(int pulse_ms, int max_wait_ms, int mode,
                                struct bootrom_catch_result *out)
{
    memset(out, 0, sizeof(*out));

    if (fd == -1) {
        out->status = BOOTROM_STATUS_IO_ERR;
        return;
    }

    /* 1. Reset the camera. */
    syslog(LOG_INFO, "bootrom_catch: start (pulse=%d ms, max_wait=%d ms, mode=%d)",
           pulse_ms, max_wait_ms, mode);
    set_signal_state(TIOCM_RTS, 0);
    set_signal_state(TIOCM_DTR, 0);
    usleep((useconds_t)pulse_ms * 1000);
    set_signal_state(TIOCM_RTS, 1);
    set_signal_state(TIOCM_DTR, 1);

    /* 2. Drop any stale UART data before starting the listen loop. */
    tcflush(fd, TCIFLUSH);

    /* 3. Marker / ACK loop, deadline-bound. */
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    int run = 0;  /* current consecutive-0x20 run length */
    while (1) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (long)((now.tv_sec - start.tv_sec) * 1000L
                                 + (now.tv_nsec - start.tv_nsec) / 1000000L);
        if (elapsed_ms >= max_wait_ms) {
            out->elapsed_ms = (uint32_t)elapsed_ms;
            /* In BLIND mode we don't need to see markers — we've been
             * blasting 0xAA the whole time and the bootrom will have
             * entered download mode silently if it's a variant that
             * doesn't emit markers.  The client confirms by sending a
             * HEAD frame and watching for an ACK. */
            if (mode == BOOTROM_MODE_BLIND) {
                syslog(LOG_INFO,
                       "bootrom_catch: blind blast complete (%ums, %u rx bytes)",
                       out->elapsed_ms, out->bytes_rx);
                out->status = BOOTROM_STATUS_OK;
                return;
            }
            out->status = BOOTROM_STATUS_TIMEOUT;
            return;
        }

        /* Keep the wire saturated with 0xAA — at 115200 baud the UART
         * clocks ~11.5 KB/s, so 64 bytes ≈ 5.5 ms of wire time. */
        unsigned char ack_burst[64];
        memset(ack_burst, BOOTROM_ACK, sizeof(ack_burst));
        ssize_t w = write(fd, ack_burst, sizeof(ack_burst));
        if (w < 0 && errno != EAGAIN && errno != EINTR) {
            log_errno_message("bootrom_catch: write(0xAA)");
            out->elapsed_ms = (uint32_t)elapsed_ms;
            out->status = BOOTROM_STATUS_IO_ERR;
            return;
        }
        if (w > 0) {
            out->bytes_tx += (uint32_t)w;
        }

        /* Poll the UART with a short timeout so write/read alternates
         * tightly. */
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv = { .tv_sec = 0, .tv_usec = 5000 };  /* 5 ms */
        int sel = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (sel < 0) {
            if (errno == EINTR) continue;
            log_errno_message("bootrom_catch: select");
            out->elapsed_ms = (uint32_t)elapsed_ms;
            out->status = BOOTROM_STATUS_IO_ERR;
            return;
        }
        if (sel == 0 || !FD_ISSET(fd, &rfds)) {
            continue;
        }

        unsigned char rxbuf[256];
        ssize_t n = read(fd, rxbuf, sizeof(rxbuf));
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            log_errno_message("bootrom_catch: read");
            out->elapsed_ms = (uint32_t)elapsed_ms;
            out->status = BOOTROM_STATUS_IO_ERR;
            return;
        }
        if (n > 0) {
            out->bytes_rx += (uint32_t)n;
        }

        /* Update diagnostic counters for every received byte regardless
         * of mode — the counters are part of the reply so the client
         * can see what was on the wire even when the catch succeeded
         * silently. */
        for (ssize_t i = 0; i < n; i++) {
            if (rxbuf[i] == BOOTROM_MARKER) {
                out->markers_seen++;
                run++;
                if (run > out->max_marker_run && run <= 255) {
                    out->max_marker_run = (uint8_t)run;
                } else if (run > 255 && out->max_marker_run < 255) {
                    out->max_marker_run = 255;
                }
                if (mode == BOOTROM_MODE_MARKER &&
                    run >= BOOTROM_MARKER_COUNT) {
                    /* Confirmed.  Send the final 0xAA so the boot ROM
                     * commits to serial-download mode, then return. */
                    unsigned char one = BOOTROM_ACK;
                    ssize_t wr = write(fd, &one, 1);
                    if (wr > 0) out->bytes_tx += (uint32_t)wr;
                    out->elapsed_ms = (uint32_t)elapsed_ms;
                    out->status = BOOTROM_STATUS_OK;
                    syslog(LOG_INFO,
                           "bootrom_catch: caught after %u ms (%u markers, "
                           "max-run %u, %u rx bytes)",
                           out->elapsed_ms, out->markers_seen,
                           out->max_marker_run, out->bytes_rx);
                    return;
                }
            } else if (rxbuf[i] != 0x00) {
                /* Non-marker, non-NUL byte: reset the run and remember
                 * it for diagnostics.  NULs are tolerated mid-run
                 * because some bootroms interleave a stray NUL under
                 * noisy conditions. */
                run = 0;
                out->last_byte = rxbuf[i];
            }
        }
    }
}

// Display the current signal state.
void show_signal_status() {
    int status;
    if (ioctl(fd, TIOCMGET, &status) == -1) {
        log_errno_message("Failed to read status");
        return;
    }
    
    int rts_bit = (status & TIOCM_RTS) ? 1 : 0;
    int dtr_bit = (status & TIOCM_DTR) ? 1 : 0;
    
    printf("\n=== Signal status ===\n");
    printf("RTS: bit=%d -> state: %s\n", 
           rts_bit,
           rts_bit ? "ASSERTED" : "DEASSERTED");
    printf("DTR: bit=%d -> state: %s\n",
           dtr_bit,
           dtr_bit ? "ASSERTED" : "DEASSERTED");
    printf("==========================================\n\n");
}

void print_help(const char *program_name) {
    printf("Vectis UART terminal with reset pulse support\n\n");
    printf("Version: %s\n", PROGRAM_VERSION);
    printf("Release date: %s\n\n", PROGRAM_RELEASE_DATE);
    printf("Usage: %s [options]\n\n", program_name);
    printf("Options:\n");
    printf("  -p, --port <port>     Serial port (default: /dev/ttyUSB0)\n");
    printf("  -b, --baud <rate>     Baud rate (default: 115200)\n");
    printf("  -t, --tcp-port [port] Enable TCP listener (default port: %d)\n", DEFAULT_TCP_PORT);
    printf("  -s, --status          Show RTS/DTR status\n");
    printf("  -v, --version         Show version information\n");
    printf("  -h, --help            Show this help\n\n");
    printf("Operating mode:\n");
    printf("  8 data bits, no parity, 1 stop bit (8N1)\n");
    printf("  RTS and DTR use normal asserted/deasserted signaling.\n\n");
    printf("Hotkeys:\n");
    printf("  Ctrl+P    - Generate an inverted reset pulse on RTS and DTR (200 ms)\n");
    printf("  Ctrl+C    - Exit the program\n");
    printf("  Ctrl+X    - Exit the program\n\n");
    printf("Examples:\n");
    printf("  %s                              # Start with default settings\n", program_name);
    printf("  %s -p /dev/ttyUSB1 -b 9600     # Port /dev/ttyUSB1, baud 9600\n", program_name);
    printf("  %s -t                          # TCP listener on port %d\n", program_name, DEFAULT_TCP_PORT);
    printf("  %s -t 40000                    # TCP listener on port 40000\n", program_name);
    printf("  %s -s                          # Show signal status\n", program_name);
}

static void print_version(const char *program_name)
{
    printf("%s %s\n", program_name, PROGRAM_VERSION);
    printf("Release date: %s\n", PROGRAM_RELEASE_DATE);
}

int main(int argc, char *argv[]) {
    const char *port = "/dev/ttyUSB0";
    int baudrate = 115200;
    int show_status = 0;
    int tcp_enabled = 0;
    int tcp_port = DEFAULT_TCP_PORT;
    fd_set readfds;
    char buffer[BUFFER_SIZE];
    int n;
    struct timeval tv;
    int exit_code = 1;

    openlog("vectis", LOG_PID | LOG_NDELAY, LOG_DAEMON);
    atexit(closelog);
    
    // Parse command-line arguments.
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--port") == 0) {
            if (i + 1 < argc) {
                port = argv[++i];
            } else {
                log_message(LOG_ERR, stderr, "Error: port not specified");
                return 1;
            }
        } else if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
            if (i + 1 < argc) {
                baudrate = atoi(argv[++i]);
            } else {
                log_message(LOG_ERR, stderr, "Error: baud rate not specified");
                return 1;
            }
        } else if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--status") == 0) {
            show_status = 1;
        } else if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--version") == 0) {
            print_version(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--tcp-port") == 0) {
            tcp_enabled = 1;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                if (parse_port(argv[++i], &tcp_port) != 0) {
                    log_message(LOG_ERR, stderr, "Error: invalid TCP port");
                    return 1;
                }
            }
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_help(argv[0]);
            return 0;
        } else {
            log_message(LOG_ERR, stderr, "Error: unknown option %s", argv[i]);
            print_help(argv[0]);
            return 1;
        }
    }
    
    // Install signal handlers.
    if (install_signal_handlers() != 0) {
        goto cleanup;
    }
    
    // Open the serial port.
    log_message(LOG_INFO, stdout, "Opening port %s...", port);
    fd = open(port, O_RDWR | O_NOCTTY);
    
    if (fd == -1) {
        log_errno_message("Failed to open port");
        log_message(LOG_ERR, stderr, "Check whether the port exists and whether you have permission to access it");
        goto cleanup;
    }
    
    // Configure UART.
    if (configure_uart(fd, baudrate) == -1) {
        goto cleanup;
    }

    /* Inetd nowait mode: stdin is the TCP socket, no `-t` listener.
     * The stdin handler below will apply RFC 2217 detection. */
    inetd_mode = (!tcp_enabled && !isatty(STDIN_FILENO));

    // Initialize signals (inactive HIGH state).
    init_signals();
    
    printf("\nPort %s opened, baud rate %d, 8N1\n", port, baudrate);
    printf("==========================================\n");
    printf("Vectis started\n");
    printf("Version: %s\n", PROGRAM_VERSION);
    printf("Release date: %s\n", PROGRAM_RELEASE_DATE);
    printf("Startup state: asserted\n");
    printf("Shutdown state: deasserted\n");
    printf("Ctrl+P - Reset pulse (RTS+DTR %d ms)\n", RESET_PULSE_MS);
    printf("Ctrl+C - Exit\n");
    printf("==========================================\n\n");
    syslog(LOG_INFO, "UART port %s opened at %d baud", port, baudrate);
    
    // Show status if requested.
    if (show_status) {
        show_signal_status();
    }

    if (tcp_enabled) {
        if (setup_tcp_listener(tcp_port) == 0) {
            printf("TCP listener active on port %d\n", tcp_port);
            syslog(LOG_INFO, "TCP listener started on port %d", tcp_port);
        } else {
            log_message(LOG_ERR, stderr, "TCP listener unavailable; continuing with console only");
        }
    }

    // Configure the terminal for input.
    if (setup_terminal() != 0) {
        goto cleanup;
    }
    
    // Main loop using select().
    while (running) {
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        FD_SET(STDIN_FILENO, &readfds);
        if (tcp_listen_fd != -1) {
            FD_SET(tcp_listen_fd, &readfds);
        }
        if (tcp_client_fd != -1) {
            FD_SET(tcp_client_fd, &readfds);
        }
        
        int max_fd = (fd > STDIN_FILENO) ? fd : STDIN_FILENO;
        if (tcp_listen_fd > max_fd) {
            max_fd = tcp_listen_fd;
        }
        if (tcp_client_fd > max_fd) {
            max_fd = tcp_client_fd;
        }
        
        // Use a timeout so we can re-check running.
        tv.tv_sec = 0;
        tv.tv_usec = 100000; // 100 ms
        
        if (select(max_fd + 1, &readfds, NULL, NULL, &tv) < 0) {
            if (errno == EINTR) continue;
            log_errno_message("select");
            break;
        }

        if (tcp_listen_fd != -1 && FD_ISSET(tcp_listen_fd, &readfds)) {
            accept_tcp_client();
        }

        if (tcp_client_fd != -1 && FD_ISSET(tcp_client_fd, &readfds)) {
            handle_tcp_client_input();
        }
        
        // Read from UART.
        if (FD_ISSET(fd, &readfds)) {
            n = read(fd, buffer, sizeof(buffer));
            if (n > 0) {
                forward_uart_output(buffer, (size_t)n);
                if (!running) {
                    break;
                }
            } else if (n == -1 && errno != EAGAIN) {
                log_errno_message("Failed to read from UART");
                break;
            }
        }
        
        // Read from the keyboard / inetd socket.
        if (FD_ISSET(STDIN_FILENO, &readfds)) {
            n = read(STDIN_FILENO, buffer, sizeof(buffer));
            if (n > 0) {
                /* When stdin IS the TCP socket (inetd nowait mode),
                 * treat it like the standalone -t client path so the
                 * same RFC 2217 detection + state machine apply.
                 * In interactive console mode (isatty stdin), keep
                 * the historic Ctrl+C/Ctrl+X exit semantics — humans
                 * don't speak Telnet by hand, so RFC 2217 detection
                 * here is a no-op for them. */
                int allow_exit_keys = !inetd_mode;
                if (handle_remote_input((const unsigned char *)buffer, (size_t)n,
                                        allow_exit_keys) != 0) {
                    break;
                }
            } else if (n == 0 && inetd_mode) {
                /* Inetd socket closed by remote end. */
                running = 0;
            }
        }
    }
    
    exit_code = 0;

cleanup:
    if (exit_code == 0) {
        syslog(LOG_INFO, "Shutting down Vectis");
    }

    // Close the port and restore the terminal.
    close_tcp_client();
    close_tcp_listener();
    if (fd != -1) {
        // Clear signals before closing.
        int rts_flag = TIOCM_RTS;
        int dtr_flag = TIOCM_DTR;
        ioctl(fd, TIOCMBIC, &rts_flag);
        ioctl(fd, TIOCMBIC, &dtr_flag);
        restore_uart();
        close(fd);
        fd = -1;
    }
    restore_terminal();
    
    if (exit_code == 0) {
        printf("\nProgram completed.\n");
        syslog(LOG_INFO, "Vectis exited");
    }
    return exit_code;
}
