/*
 *
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * usbserial.c — usbfs side of the USB-UART support (see usbserial.h):
 * device lookup by USB path, taking the interface from the kernel driver,
 * bulk IN URBs of one packet each queued all the time and reaped from
 * the main poll loop, bulk OUT in short synchronous chunks.
 *
 * Reading: usbfs reports POLLOUT on its fd when an asynchronous URB has
 * completed, and POLLERR | POLLHUP once the device is gone.  The IN URBs
 * are reaped into a backlog that vectis drains like a tty read.  A write
 * reaps between its chunks too, so a long paste does not overflow the
 * chip's small receive FIFO while the echo comes back.
 *
 */

#define _GNU_SOURCE

#include "usbserial.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>

#ifdef VECTIS_CHIP_CH34X
extern const struct usbs_chip usbs_chip_ch34x;
#endif

const struct usbs_chip *const usbs_chips[] = {
#ifdef VECTIS_CHIP_CH34X
    &usbs_chip_ch34x,
#endif
    NULL
};

#define USBS_N_IN        16     /* IN URBs kept queued */
#define USBS_IN_MAX      512    /* largest IN URB: a high-speed bulk packet */
#define USBS_OUT_CHUNK   256    /* max bytes per synchronous OUT transfer */
#define USBS_CTRL_TIMEOUT 1000  /* ms per control request */
#define USBS_RX_SIZE     65536  /* receive backlog */

/* ioctl() takes the request as unsigned long in glibc and uClibc, as int
 * in musl; the usbfs numbers do not fit an int, so convert explicitly. */
#ifdef __GLIBC__
#define USBS_IOCTL(fd, req, arg) ioctl((fd), (unsigned long)(req), (arg))
#else
#define USBS_IOCTL(fd, req, arg) ioctl((fd), (int)(req), (arg))
#endif

static struct usbs_dev g_dev = { .fd = -1 };
/* Allocated once: struct usbdevfs_urb ends in a flexible array, so it
 * cannot be an array element itself. */
static struct usbdevfs_urb *g_urb[USBS_N_IN];
static uint8_t g_urb_buf[USBS_N_IN][USBS_IN_MAX];
/* Bytes per IN URB: one packet of the IN endpoint (see in_packet_size()). */
static size_t g_in_size;

static uint8_t g_rx[USBS_RX_SIZE];
static size_t  g_rx_head, g_rx_len;
static int     g_rx_overflow;       /* logged once per overflow */

/* The last bulk OUT: when it completed and how long it was, so a break
 * can wait until those bytes have left the chip's FIFO. */
static struct timespec g_out_done;
static size_t          g_out_len;

static char g_err[160];
static char g_name[64];

static long long ms_since(const struct timespec *t)
{
    struct timespec now;

    clock_gettime(CLOCK_MONOTONIC, &now);
    return (long long)(now.tv_sec - t->tv_sec) * 1000 + (now.tv_nsec - t->tv_nsec) / 1000000;
}

/* Sleep about ms milliseconds in 5 ms slices, reaping the IN side. */
static void wait_reaping(long long ms)
{
    struct timespec start;

    clock_gettime(CLOCK_MONOTONIC, &start);
    while (ms_since(&start) < ms) {
        struct timespec slice = { 0, 5 * 1000000L };
        nanosleep(&slice, NULL);
        (void)usbs_service();
    }
}

const char *usbs_error(void) { return g_err; }
const char *usbs_name(void)  { return g_name; }

const char *usbs_chip_names(void)
{
    static char names[64];
    size_t o = 0;

    names[0] = '\0';
    for (int i = 0; usbs_chips[i] != NULL; i++) {
        int n = snprintf(names + o, sizeof names - o, "%s%s", i ? ", " : "",
                         usbs_chips[i]->name);
        if (n < 0 || (size_t)n >= sizeof names - o)
            break;
        o += (size_t)n;
    }
    return names;
}

/* ---------- -U spec ---------- */

static int valid_path(const char *p)
{
    /* sysfs USB device names: "1-1", "1-1.3.4", "2-4.1" */
    if (*p < '0' || *p > '9')
        return 0;
    for (; *p; p++)
        if (!((*p >= '0' && *p <= '9') || *p == '-' || *p == '.'))
            return 0;
    return 1;
}

int usbs_parse_spec(const char *spec, uint16_t *vid, uint16_t *pid,
                    char *path, size_t path_size, const char **why)
{
    const char *at;
    unsigned v, p;
    int n = 0;

    *vid = *pid = 0;
    path[0] = '\0';
    if (strncmp(spec, "usb:", 4) == 0) {
        if (sscanf(spec + 4, "%4x:%4x%n", &v, &p, &n) != 2 ||
            (spec[4 + n] != '\0' && spec[4 + n] != '@')) {
            *why = "expected usb:VID:PID or usb:VID:PID@PATH (hex VID/PID)";
            return -1;
        }
        if (v == 0) {
            *why = "VID 0000 is not a valid vendor ID";
            return -1;
        }
        *vid = (uint16_t)v;
        *pid = (uint16_t)p;
        at = spec[4 + n] == '@' ? spec + 4 + n + 1 : NULL;
    } else {
        at = spec;
    }
    if (at != NULL) {
        if (!valid_path(at) || strlen(at) >= path_size) {
            *why = "expected a USB path such as 1-1.3.4 (see /sys/bus/usb/devices)";
            return -1;
        }
        snprintf(path, path_size, "%s", at);
    }
    return 0;
}

/* ---------- lookup ---------- */

static int read_sysfs_hex(const char *dev, const char *attr, unsigned *out)
{
    char fn[256], buf[32];
    FILE *f;
    int ok;

    snprintf(fn, sizeof fn, "/sys/bus/usb/devices/%s/%s", dev, attr);
    f = fopen(fn, "r");
    if (f == NULL)
        return -1;
    ok = fgets(buf, sizeof buf, f) != NULL;
    fclose(f);
    if (!ok)
        return -1;
    return sscanf(buf, "%x", out) == 1 ? 0 : -1;
}

static int read_sysfs_dec(const char *dev, const char *attr, unsigned *out)
{
    char fn[256], buf[32];
    FILE *f;
    int ok;

    snprintf(fn, sizeof fn, "/sys/bus/usb/devices/%s/%s", dev, attr);
    f = fopen(fn, "r");
    if (f == NULL)
        return -1;
    ok = fgets(buf, sizeof buf, f) != NULL;
    fclose(f);
    if (!ok)
        return -1;
    return sscanf(buf, "%u", out) == 1 ? 0 : -1;
}

static const struct usbs_chip *chip_for(uint16_t vid, uint16_t pid)
{
    for (int i = 0; usbs_chips[i] != NULL; i++)
        for (const uint16_t (*id)[2] = usbs_chips[i]->ids; (*id)[0] != 0; id++)
            if ((*id)[0] == vid && (*id)[1] == pid)
                return usbs_chips[i];
    return NULL;
}

/* Find the device in sysfs.  Fills name, ids, chip and node; 0 or -1. */
static int find_device(uint16_t vid, uint16_t pid, const char *path,
                       char *found, size_t found_size, uint16_t *fvid, uint16_t *fpid,
                       const struct usbs_chip **chip, char *node, size_t node_size)
{
    DIR *dir;
    struct dirent *e;
    int matched = 0;

    if (path[0] != '\0') {
        unsigned v, p;
        if (read_sysfs_hex(path, "idVendor", &v) != 0 ||
            read_sysfs_hex(path, "idProduct", &p) != 0) {
            snprintf(g_err, sizeof g_err, "no USB device at %s", path);
            errno = ENODEV;
            return -1;
        }
        if (vid != 0 && (v != vid || p != pid)) {
            snprintf(g_err, sizeof g_err, "%s is %04x:%04x, not %04x:%04x",
                     path, v, p, vid, pid);
            errno = ENODEV;
            return -1;
        }
        snprintf(found, found_size, "%s", path);
        *fvid = (uint16_t)v;
        *fpid = (uint16_t)p;
        matched = 1;
    } else {
        dir = opendir("/sys/bus/usb/devices");
        if (dir == NULL) {
            snprintf(g_err, sizeof g_err, "/sys/bus/usb/devices: %s", strerror(errno));
            return -1;
        }
        while ((e = readdir(dir)) != NULL) {
            unsigned v, p;
            if (strchr(e->d_name, ':') != NULL || !valid_path(e->d_name) ||
                strlen(e->d_name) >= found_size)
                continue;   /* interfaces, root hubs, "." */
            if (read_sysfs_hex(e->d_name, "idVendor", &v) != 0 ||
                read_sysfs_hex(e->d_name, "idProduct", &p) != 0 ||
                v != vid || p != pid)
                continue;
            snprintf(found, found_size, "%.*s", (int)found_size - 1, e->d_name);
            *fvid = (uint16_t)v;
            *fpid = (uint16_t)p;
            matched = 1;
            break;
        }
        closedir(dir);
        if (!matched) {
            snprintf(g_err, sizeof g_err, "no USB device %04x:%04x", vid, pid);
            errno = ENODEV;
            return -1;
        }
    }

    *chip = chip_for(*fvid, *fpid);
    if (*chip == NULL) {
        snprintf(g_err, sizeof g_err, "%s: %04x:%04x is not a supported chip (built in: %s)",
                 found, *fvid, *fpid, usbs_chip_names());
        errno = ENODEV;
        return -1;
    }

    unsigned bus, devnum;
    if (read_sysfs_dec(found, "busnum", &bus) != 0 ||
        read_sysfs_dec(found, "devnum", &devnum) != 0) {
        snprintf(g_err, sizeof g_err, "%s: no busnum/devnum in sysfs", found);
        errno = ENODEV;
        return -1;
    }
    snprintf(node, node_size, "/dev/bus/usb/%03u/%03u", bus, devnum);
    return 0;
}

/* ---------- transfers ---------- */

int usbs_ctrl(struct usbs_dev *d, uint8_t req_type, uint8_t req,
              uint16_t value, uint16_t index, void *data, uint16_t len)
{
    struct usbdevfs_ctrltransfer c;

    memset(&c, 0, sizeof c);
    c.bRequestType = req_type;
    c.bRequest     = req;
    c.wValue       = value;
    c.wIndex       = index;
    c.wLength      = len;
    c.timeout      = USBS_CTRL_TIMEOUT;
    c.data         = data;
    return USBS_IOCTL(d->fd, USBDEVFS_CONTROL, &c);
}

static int submit_in(int i)
{
    struct usbdevfs_urb *u = g_urb[i];

    memset(u, 0, sizeof *u);
    u->type          = USBDEVFS_URB_TYPE_BULK;
    u->endpoint      = g_dev.chip->ep_in;
    u->buffer        = g_urb_buf[i];
    u->buffer_length = (int)g_in_size;
    u->usercontext   = (void *)(intptr_t)i;
    return USBS_IOCTL(g_dev.fd, USBDEVFS_SUBMITURB, u);
}

static void rx_put(const uint8_t *p, size_t n)
{
    if (n > USBS_RX_SIZE - g_rx_len) {
        if (!g_rx_overflow)
            usbs_log(LOG_WARNING, "usb: receive backlog full, dropping data");
        g_rx_overflow = 1;
        n = USBS_RX_SIZE - g_rx_len;
    }
    for (size_t i = 0; i < n; i++)
        g_rx[(g_rx_head + g_rx_len + i) % USBS_RX_SIZE] = p[i];
    g_rx_len += n;
}

size_t usbs_pending(void) { return g_rx_len; }

size_t usbs_read(uint8_t *buf, size_t n)
{
    if (n > g_rx_len)
        n = g_rx_len;
    for (size_t i = 0; i < n; i++)
        buf[i] = g_rx[(g_rx_head + i) % USBS_RX_SIZE];
    g_rx_head = (g_rx_head + n) % USBS_RX_SIZE;
    g_rx_len -= n;
    if (g_rx_len == 0) {
        g_rx_head = 0;
        g_rx_overflow = 0;
    }
    return n;
}

void usbs_purge_rx(void)
{
    /* Drop what completed URBs hold as well, not only the backlog. */
    if (g_dev.fd >= 0)
        (void)usbs_service();
    g_rx_head = g_rx_len = 0;
    g_rx_overflow = 0;
}

int usbs_service(void)
{
    if (g_dev.fd < 0) {
        errno = ENODEV;
        return -1;
    }
    for (;;) {
        struct usbdevfs_urb *u = NULL;
        int i;

        if (USBS_IOCTL(g_dev.fd, USBDEVFS_REAPURBNDELAY, &u) != 0) {
            if (errno == EAGAIN)
                return 0;
            if (errno == EINTR)
                continue;
            return -1;   /* ENODEV: unplugged */
        }
        i = (int)(intptr_t)u->usercontext;
        if (i < 0 || i >= USBS_N_IN)
            continue;
        switch (u->status) {
        case 0:
            rx_put(g_urb_buf[i], (size_t)u->actual_length);
            break;
        case -EPIPE: {
            unsigned ep = g_dev.chip->ep_in;
            (void)USBS_IOCTL(g_dev.fd, USBDEVFS_CLEAR_HALT, &ep);
            break;
        }
        case -ENOENT: case -ECONNRESET: case -ENODEV: case -ESHUTDOWN:
            /* vectis never discards a URB itself: the kernel killed it
             * (unplug, reset, interface taken away).  Report the link as
             * lost, so the caller reopens it, rather than receive less
             * and less until nothing arrives. */
            errno = ENODEV;
            return -1;
        default:
            break;               /* -EPROTO, -EOVERFLOW, ...: try again */
        }
        if (submit_in(i) != 0)
            return -1;
    }
}

int usbs_write(const uint8_t *buf, size_t n)
{
    /* About 100 ms of line time per chunk (32 bytes at least), and a
     * timeout of twice the chunk's line time plus a second, so a slow
     * baud rate neither times out nor holds the poll loop for long. */
    uint32_t baud = g_dev.line.baud ? g_dev.line.baud : 115200;
    size_t chunk = baud / 100;

    if (chunk < 32)
        chunk = 32;
    if (chunk > USBS_OUT_CHUNK)
        chunk = USBS_OUT_CHUNK;
    while (n > 0) {
        struct usbdevfs_bulktransfer b;
        size_t len = n > chunk ? chunk : n;
        int r;

        if (g_dev.fd < 0) {
            errno = ENODEV;
            return -1;
        }
        memset(&b, 0, sizeof b);
        b.ep      = g_dev.chip->ep_out;
        b.len     = (unsigned)len;
        b.timeout = (unsigned)(1000 + 2 * len * 12 * 1000 / baud);   /* 12 bits/byte max */
        b.data    = (void *)buf;
        r = USBS_IOCTL(g_dev.fd, USBDEVFS_BULK, &b);
        if (r < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        buf += r;
        n   -= (size_t)r;
        clock_gettime(CLOCK_MONOTONIC, &g_out_done);
        g_out_len = (size_t)r;
        /* Keep the IN side drained while the chip is busy sending. */
        if (usbs_service() != 0)
            return -1;
    }
    return 0;
}

/* ---------- open / close ---------- */

/* wMaxPacketSize of endpoint ep of interface iface (alternate setting 0),
 * from the descriptors usbfs returns on read(), or 0 if not found.
 *
 * A bulk IN URB completes only when its buffer is full or the device
 * sends a short packet.  Larger than one packet, it keeps the bytes of a
 * burst that ends on a full packet (32 bytes on a CH340) until more data
 * arrives (#48); one packet per URB, as the kernel's usb-serial sizes
 * them, completes every packet. */
static size_t in_packet_size(int fd, unsigned iface, uint8_t ep)
{
    uint8_t d[1024];
    ssize_t n;
    int cur_if = -1, cur_alt = -1;

    if (lseek(fd, 0, SEEK_SET) != 0 || (n = read(fd, d, sizeof d)) <= 0)
        return 0;
    for (ssize_t p = 0; p + 2 <= n && d[p] >= 2; p += d[p]) {
        if (p + d[p] > n)
            break;
        if (d[p + 1] == 4 && d[p] >= 9) {           /* interface */
            cur_if  = d[p + 2];
            cur_alt = d[p + 3];
        } else if (d[p + 1] == 5 && d[p] >= 7 &&    /* endpoint */
                   cur_if == (int)iface && cur_alt == 0 && d[p + 2] == ep) {
            return (size_t)((d[p + 4] | d[p + 5] << 8) & 0x7ff);
        }
    }
    return 0;
}

int usbs_open(uint16_t vid, uint16_t pid, const char *path)
{
    char found[32], node[64];
    struct usbdevfs_getdriver gd;
    const struct usbs_chip *chip;
    uint16_t fvid, fpid;
    unsigned ifno;
    int fd;

    usbs_close();
    g_err[0] = '\0';
    if (usbs_chips[0] == NULL) {
        snprintf(g_err, sizeof g_err, "this build has no USB chip support");
        errno = ENOTSUP;
        return -1;
    }
    for (int i = 0; i < USBS_N_IN; i++) {
        if (g_urb[i] == NULL && (g_urb[i] = calloc(1, sizeof *g_urb[i])) == NULL) {
            snprintf(g_err, sizeof g_err, "out of memory");
            errno = ENOMEM;
            return -1;
        }
    }
    if (find_device(vid, pid, path, found, sizeof found, &fvid, &fpid, &chip,
                    node, sizeof node) != 0)
        return -1;

    fd = open(node, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        int e = errno;
        snprintf(g_err, sizeof g_err, "open %s (%s): %s%s", node, found, strerror(e),
                 e == EACCES ? " (needs a udev rule, see README)" : "");
        errno = e;
        return -1;
    }

    /* Take the interface from the kernel driver (ch341 & co.), if any.
     * It is not given back on close: see usbs_close(). */
    int detached = 0;

    ifno = chip->iface;
    memset(&gd, 0, sizeof gd);
    gd.interface = ifno;
    if (USBS_IOCTL(fd, USBDEVFS_GETDRIVER, &gd) == 0 && strcmp(gd.driver, "usbfs") != 0) {
        struct usbdevfs_ioctl cmd = { .ifno = (int)ifno, .ioctl_code = USBDEVFS_DISCONNECT,
                                      .data = NULL };
        if (USBS_IOCTL(fd, USBDEVFS_IOCTL, &cmd) < 0) {
            int e = errno;
            snprintf(g_err, sizeof g_err, "%s: detach kernel driver %.32s: %s",
                     found, gd.driver, strerror(e));
            close(fd);
            errno = e;
            return -1;
        }
        usbs_log(LOG_INFO, "usb: %s: kernel driver %.32s detached", found, gd.driver);
        detached = 1;
    }
    if (USBS_IOCTL(fd, USBDEVFS_CLAIMINTERFACE, &ifno) != 0) {
        int e = errno;
        /* Someone else took it in between: give the driver back. */
        if (detached) {
            struct usbdevfs_ioctl cmd = { .ifno = (int)ifno, .ioctl_code = USBDEVFS_CONNECT,
                                          .data = NULL };
            (void)USBS_IOCTL(fd, USBDEVFS_IOCTL, &cmd);
        }
        snprintf(g_err, sizeof g_err, "%s: claim interface %u: %s%s", found, ifno,
                 strerror(e), e == EBUSY ? " (another program holds it)" : "");
        close(fd);
        errno = e;
        return -1;
    }

    /* No guessing: a URB larger than the packet brings the held-back
     * tail back, a smaller one overflows. */
    g_in_size = in_packet_size(fd, ifno, chip->ep_in);
    if (g_in_size == 0 || g_in_size > USBS_IN_MAX) {
        snprintf(g_err, sizeof g_err, "%s: no usable wMaxPacketSize for IN endpoint 0x%02x (%u)",
                 found, chip->ep_in, (unsigned)g_in_size);
        /* As usbs_close(): the kernel driver is not given back, its probe
         * would drop the modem lines on every retry of the open. */
        (void)USBS_IOCTL(fd, USBDEVFS_RELEASEINTERFACE, &ifno);
        close(fd);
        errno = EPROTO;
        return -1;
    }

    memset(&g_dev, 0, sizeof g_dev);
    g_dev.fd   = fd;
    g_dev.chip = chip;
    g_dev.vid  = fvid;
    g_dev.pid  = fpid;
    g_dev.dtr  = g_dev.rts = -1;
    snprintf(g_dev.path, sizeof g_dev.path, "%s", found);
    snprintf(g_name, sizeof g_name, "%s (%04x:%04x %s)", found, fvid, fpid, chip->name);
    usbs_purge_rx();

    if (chip->init(&g_dev) != 0) {
        int e = errno;
        snprintf(g_err, sizeof g_err, "%s: %s init: %s", found, chip->name, strerror(e));
        usbs_close();
        errno = e;
        return -1;
    }
    for (int i = 0; i < USBS_N_IN; i++) {
        if (submit_in(i) != 0) {
            int e = errno;
            snprintf(g_err, sizeof g_err, "%s: submit IN URB: %s", found, strerror(e));
            usbs_close();
            errno = e;
            return -1;
        }
    }
    return fd;
}

void usbs_close(void)
{
    if (g_dev.fd >= 0) {
        unsigned ifno = g_dev.chip->iface;
        /* Releasing the interface kills our URBs; the modem lines stay
         * as they are, and the kernel driver is not reattached. */
        (void)USBS_IOCTL(g_dev.fd, USBDEVFS_RELEASEINTERFACE, &ifno);
        close(g_dev.fd);
    }
    g_dev.fd = -1;
    usbs_purge_rx();
}

/* ---------- line control ---------- */

int usbs_set_line(const struct usbs_line *l)
{
    if (g_dev.fd < 0) {
        errno = ENODEV;
        return -1;
    }
    if (g_dev.chip->set_line(&g_dev, l) != 0)
        return -1;
    g_dev.line = *l;
    return 0;
}

int usbs_get_line(struct usbs_line *l)
{
    if (g_dev.fd < 0 || g_dev.line.baud == 0)
        return -1;
    *l = g_dev.line;
    return 0;
}

int usbs_set_modem(int dtr, int rts)
{
    if (g_dev.fd < 0) {
        errno = ENODEV;
        return -1;
    }
    if (dtr < 0)
        dtr = g_dev.dtr > 0;
    if (rts < 0)
        rts = g_dev.rts > 0;
    if (g_dev.chip->set_modem(&g_dev, dtr, rts) != 0)
        return -1;
    g_dev.dtr = dtr;
    g_dev.rts = rts;
    return 0;
}

int usbs_send_break(int ms)
{
    uint32_t baud = g_dev.line.baud ? g_dev.line.baud : 115200;

    if (g_dev.fd < 0) {
        errno = ENODEV;
        return -1;
    }
    /* A finished bulk OUT only means the chip has the bytes: let the
     * last chunk leave the wire first (12 bits per byte at most), as
     * tcsendbreak() drains a tty. */
    if (g_out_len > 0) {
        long long left = (long long)g_out_len * 12 * 1000 / baud + 5 - ms_since(&g_out_done);
        if (left > 0)
            wait_reaping(left);
    }
    if (g_dev.chip->set_break(&g_dev, 1) != 0)
        return -1;
    wait_reaping(ms);
    return g_dev.chip->set_break(&g_dev, 0);
}
