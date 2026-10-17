/*
 *
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * usbserial.h — USB-UART adapters driven from user space through usbfs
 * (/dev/bus/usb/BBB/DDD), for hosts whose kernel has no driver for the
 * adapter and cannot be rebuilt.  Linux only, no libusb: libc and the
 * kernel's usbfs ioctls.
 *
 * usbserial.c owns the device (lookup by USB path, claiming the
 * interface, bulk URBs, the receive backlog); one file per chip family
 * (ch34x.c, ...) speaks the chip's vendor requests.  Which chips are
 * built in is chosen at compile time (Makefile USB_CHIPS).
 *
 */

#ifndef VECTIS_USBSERIAL_H
#define VECTIS_USBSERIAL_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

/* Line settings, the same values vectis uses on the command line. */
struct usbs_line {
    uint32_t baud;
    int      data_bits;   /* 5..8 */
    int      stop_bits;   /* 1 or 2 */
    char     parity;      /* 'N', 'E', 'O' */
};

/* The open device.  One at a time: vectis drives a single port. */
struct usbs_dev {
    int fd;
    const struct usbs_chip *chip;
    uint16_t vid, pid;
    char path[32];              /* USB path in sysfs: "1-1.3.4" */
    struct usbs_line line;      /* as last set */
    int dtr, rts;               /* as last set; -1 = unknown */
    unsigned chip_priv[4];      /* chip file's own state (version, quirks) */
};

/* One chip family. */
struct usbs_chip {
    const char *name;                 /* "ch34x" */
    const uint16_t (*ids)[2];         /* {vid, pid} pairs, ends with {0, 0} */
    uint8_t iface, ep_in, ep_out;
    /* Bring the chip up.  Must not change the modem lines: they may be
     * holding the target's power from a previous run. */
    int (*init)(struct usbs_dev *d);
    int (*set_line)(struct usbs_dev *d, const struct usbs_line *l);
    int (*set_modem)(struct usbs_dev *d, int dtr, int rts);
    int (*set_break)(struct usbs_dev *d, int on);
};

/* Chips built into this binary, NULL-terminated. */
extern const struct usbs_chip *const usbs_chips[];

/* Supported chips as text ("ch34x"), for usage and --version. */
const char *usbs_chip_names(void);

/* Parse "-U" spec: "1-1.3.4", "usb:1a86:7522" or "usb:1a86:7522@1-1.3.4".
 * Returns 0, or -1 with a reason in *why. */
int usbs_parse_spec(const char *spec, uint16_t *vid, uint16_t *pid,
                    char *path, size_t path_size, const char **why);

/* Find the device, take the interface from the kernel driver if one is
 * bound, bring the chip up and start receiving.  The modem lines are not
 * touched.  Returns the usbfs fd for poll() (POLLOUT = URBs to reap,
 * POLLERR/POLLHUP = device gone), or -1 with the reason in usbs_error()
 * (not logged here: the caller decides, see vectis' quiet retries). */
int usbs_open(uint16_t vid, uint16_t pid, const char *path);

/* Why the last usbs_open() failed. */
const char *usbs_error(void);

/* Close: release the interface, leave the modem lines as they are and do
 * not hand the interface back to the kernel driver (its probe would drop
 * the lines).  Safe to call when nothing is open. */
void usbs_close(void);

/* Device name for messages: "1-1.3.4 (1a86:7522 ch34x)". */
const char *usbs_name(void);

/* Reap completed URBs into the receive backlog and resubmit them.
 * Returns 0, or -1 when the device is gone. */
int usbs_service(void);

/* Bytes waiting in the receive backlog. */
size_t usbs_pending(void);

/* Take up to n bytes from the receive backlog. */
size_t usbs_read(uint8_t *buf, size_t n);

/* Send all bytes (bulk OUT, in chunks; receiving goes on meanwhile).
 * Returns 0, or -1 with errno set. */
int usbs_write(const uint8_t *buf, size_t n);

int usbs_set_line(const struct usbs_line *l);
int usbs_get_line(struct usbs_line *l);
/* -1 leaves a line as it is. */
int usbs_set_modem(int dtr, int rts);
/* Break for ms milliseconds (blocks, like tcsendbreak). */
int usbs_send_break(int ms);
/* Drop the receive backlog (RFC 2217 PURGE-DATA). */
void usbs_purge_rx(void);

/* Internal, for the chip files. */
int usbs_ctrl(struct usbs_dev *d, uint8_t req_type, uint8_t req,
              uint16_t value, uint16_t index, void *data, uint16_t len);

/* Provided by the program (vectis.c): syslog + stderr. */
void usbs_log(int priority, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

#endif
