/*
 *
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * ch34x.c — WCH CH340/CH341 USB-UART over usbfs (see usbserial.h).
 *
 * The chip is driven by vendor control requests on endpoint 0; data goes
 * over bulk endpoints 0x02 (out) and 0x82 (in), raw, with no status
 * bytes.  Register layout and the baud-rate divisor follow the Linux
 * ch341 driver (drivers/usb/serial/ch341.c).
 *
 */

#include "usbserial.h"

#include <errno.h>
#include <string.h>
#include <syslog.h>

#define CH34X_REQ_READ_VERSION 0x5F
#define CH34X_REQ_WRITE_REG    0x9A
#define CH34X_REQ_READ_REG     0x95
#define CH34X_REQ_SERIAL_INIT  0xA1
#define CH34X_REQ_MODEM_CTRL   0xA4

#define CH34X_REG_BREAK        0x05
#define CH34X_REG_PRESCALER    0x12
#define CH34X_REG_DIVISOR      0x13
#define CH34X_REG_LCR          0x18
#define CH34X_REG_LCR2         0x25

#define CH34X_NBREAK_BITS      0x01

#define CH34X_LCR_ENABLE_RX    0x80
#define CH34X_LCR_ENABLE_TX    0x40
#define CH34X_LCR_PAR_EVEN     0x10
#define CH34X_LCR_ENABLE_PAR   0x08
#define CH34X_LCR_STOP_BITS_2  0x04
#define CH34X_LCR_CS8          0x03
#define CH34X_LCR_CS7          0x02
#define CH34X_LCR_CS6          0x01
#define CH34X_LCR_CS5          0x00

/* Modem control bits; the chip takes them inverted. */
#define CH34X_BIT_DTR          0x20
#define CH34X_BIT_RTS          0x40

#define CH34X_CLKRATE          48000000u
#define CH34X_CLK_DIV(ps, fact) (1u << (12 - 3 * (ps) - (fact)))
#define CH34X_MIN_RATE(ps)     (CH34X_CLKRATE / (CH34X_CLK_DIV((ps), 1) * 512))
#define CH34X_MIN_BPS          ((CH34X_CLKRATE + CH34X_CLK_DIV(0, 0) * 256 - 1) / \
                                (CH34X_CLK_DIV(0, 0) * 256))
#define CH34X_MAX_BPS          (CH34X_CLKRATE / (CH34X_CLK_DIV(3, 0) * 2))

#define REQ_OUT 0x40   /* vendor, device, host-to-device */
#define REQ_IN  0xC0   /* vendor, device, device-to-host */

/* chip_priv[] slots */
#define PRIV_VERSION 0
#define PRIV_QUIRKS  1
#define QUIRK_LIMITED_PRESCALER 0x1
#define QUIRK_SIMULATE_BREAK    0x2

static const uint16_t ch34x_ids[][2] = {
    { 0x1a86, 0x5523 },   /* CH341 in serial mode */
    { 0x1a86, 0x7522 },   /* CH340K */
    { 0x1a86, 0x7523 },   /* CH340 */
    { 0x2184, 0x0057 },
    { 0x4348, 0x5523 },
    { 0x9986, 0x7523 },
    { 0, 0 }
};

static int out_req(struct usbs_dev *d, uint8_t req, uint16_t value, uint16_t index)
{
    return usbs_ctrl(d, REQ_OUT, req, value, index, NULL, 0) < 0 ? -1 : 0;
}

static int in_req(struct usbs_dev *d, uint8_t req, uint16_t value, uint16_t index,
                  uint8_t *buf, uint16_t len)
{
    int r = usbs_ctrl(d, REQ_IN, req, value, index, buf, len);

    if (r < 0)
        return -1;
    if (r < len) {
        errno = EIO;
        return -1;
    }
    return 0;
}

/* Divisor and prescaler for a baud rate, as in the Linux driver:
 * returns (0x100 - div) << 8 | fact << 2 | ps, or -1. */
static int ch34x_divisor(unsigned quirks, uint32_t speed)
{
    static const uint32_t min_rates[] = {
        CH34X_MIN_RATE(0), CH34X_MIN_RATE(1), CH34X_MIN_RATE(2), CH34X_MIN_RATE(3)
    };
    unsigned fact = 1, div, clk_div;
    int ps;

    if (speed < CH34X_MIN_BPS)
        speed = CH34X_MIN_BPS;
    if (speed > CH34X_MAX_BPS)
        speed = CH34X_MAX_BPS;

    /* Highest base clock (fact = 1) that gives a divisor below 512. */
    for (ps = 3; ps >= 0; ps--)
        if (speed > min_rates[ps])
            break;
    if (ps < 0)
        return -1;

    clk_div = CH34X_CLK_DIV(ps, fact);
    div = CH34X_CLKRATE / (clk_div * speed);

    /* Halve the base clock (fact = 0) if required. */
    if (div < 9 || div > 255 || (ps < 3 && (quirks & QUIRK_LIMITED_PRESCALER))) {
        div /= 2;
        clk_div *= 2;
        fact = 0;
    }
    if (div < 2)
        return -1;

    /* Next divisor if that rate is closer (scaled against rounding). */
    if (16 * CH34X_CLKRATE / (clk_div * div) - 16 * speed >=
        16 * speed - 16 * CH34X_CLKRATE / (clk_div * (div + 1)))
        div++;

    /* Prefer the lower base clock if the divisor is even. */
    if (fact == 1 && div % 2 == 0) {
        div /= 2;
        fact = 0;
    }
    return (int)((0x100 - div) << 8 | fact << 2 | (unsigned)ps);
}

static int ch34x_set_line(struct usbs_dev *d, const struct usbs_line *l)
{
    unsigned version = d->chip_priv[PRIV_VERSION];
    uint8_t lcr = CH34X_LCR_ENABLE_RX | CH34X_LCR_ENABLE_TX;
    int val = ch34x_divisor(d->chip_priv[PRIV_QUIRKS], l->baud);

    if (val < 0) {
        errno = EINVAL;
        return -1;
    }
    switch (l->data_bits) {
    case 5: lcr |= CH34X_LCR_CS5; break;
    case 6: lcr |= CH34X_LCR_CS6; break;
    case 7: lcr |= CH34X_LCR_CS7; break;
    case 8: lcr |= CH34X_LCR_CS8; break;
    default: errno = EINVAL; return -1;
    }
    switch (l->parity) {
    case 'N': break;
    case 'O': lcr |= CH34X_LCR_ENABLE_PAR; break;
    case 'E': lcr |= CH34X_LCR_ENABLE_PAR | CH34X_LCR_PAR_EVEN; break;
    default: errno = EINVAL; return -1;
    }
    if (l->stop_bits == 2)
        lcr |= CH34X_LCR_STOP_BITS_2;
    /* Before version 0x30 the line format sits elsewhere: 8N1 only, so
     * refuse anything else rather than report it as applied. */
    if (version < 0x30 && (l->data_bits != 8 || l->parity != 'N' || l->stop_bits != 1)) {
        errno = EINVAL;
        return -1;
    }

    /* Without bit 7 the chip holds received data until a full 32-byte
     * packet; version 0x27 has the bit inverted. */
    if (version > 0x27)
        val |= 0x80;
    if (out_req(d, CH34X_REQ_WRITE_REG,
                CH34X_REG_DIVISOR << 8 | CH34X_REG_PRESCALER, (uint16_t)val) != 0)
        return -1;
    if (version < 0x30)
        return 0;
    return out_req(d, CH34X_REQ_WRITE_REG, CH34X_REG_LCR2 << 8 | CH34X_REG_LCR, lcr);
}

static int ch34x_set_modem(struct usbs_dev *d, int dtr, int rts)
{
    uint8_t control = (uint8_t)((dtr ? CH34X_BIT_DTR : 0) | (rts ? CH34X_BIT_RTS : 0));

    return out_req(d, CH34X_REQ_MODEM_CTRL, (uint16_t)~control, 0);
}

static int ch34x_set_break(struct usbs_dev *d, int on)
{
    const uint16_t reg = CH34X_REG_LCR << 8 | CH34X_REG_BREAK;
    uint8_t r[2];

    if (d->chip_priv[PRIV_QUIRKS] & QUIRK_SIMULATE_BREAK) {
        /* No break register: a NUL byte at the lowest rate holds the line
         * low for its start and data bits, ~195 ms at 46 bps; the stop
         * bit ends it.  That is shorter than the 250 ms asked for, but
         * enough for U-Boot and SysRq.  Then restore the settings. */
        struct usbs_line slow = { CH34X_MIN_BPS, 8, 1, 'N' };
        static const uint8_t nul = 0;
        if (!on)
            return ch34x_set_line(d, &d->line);
        if (ch34x_set_line(d, &slow) != 0)
            return -1;
        if (usbs_write(&nul, 1) != 0) {
            int e = errno;
            (void)ch34x_set_line(d, &d->line);   /* do not stay at 46 bps */
            errno = e;
            return -1;
        }
        return 0;
    }
    if (in_req(d, CH34X_REQ_READ_REG, reg, 0, r, sizeof r) != 0)
        return -1;
    if (on) {
        r[0] &= (uint8_t)~CH34X_NBREAK_BITS;
        r[1] &= (uint8_t)~CH34X_LCR_ENABLE_TX;
    } else {
        r[0] |= CH34X_NBREAK_BITS;
        r[1] |= CH34X_LCR_ENABLE_TX;
    }
    return out_req(d, CH34X_REQ_WRITE_REG, reg, (uint16_t)(r[0] | r[1] << 8));
}

/* Bring the chip up without touching DTR/RTS.  The serial-init request
 * (0xA1) is sent only to a chip that is not set up yet (receiver and
 * transmitter disabled in LCR, as after power-up): a chip that vectis or
 * the kernel driver configured before keeps its state, modem lines
 * included, across a restart of vectis. */
static int ch34x_init(struct usbs_dev *d)
{
    uint8_t buf[2];

    if (in_req(d, CH34X_REQ_READ_VERSION, 0, 0, buf, sizeof buf) != 0)
        return -1;
    d->chip_priv[PRIV_VERSION] = buf[0];
    d->chip_priv[PRIV_QUIRKS] = 0;

    /* Chips without a break register stall this read (the Linux driver's
     * test); they also have a limited prescaler. */
    if (in_req(d, CH34X_REQ_READ_REG, CH34X_REG_BREAK, 0, buf, sizeof buf) != 0) {
        if (errno != EPIPE)
            return -1;
        d->chip_priv[PRIV_QUIRKS] = QUIRK_LIMITED_PRESCALER | QUIRK_SIMULATE_BREAK;
    }

    /* Chips before 0x30 have no LCR to tell a set-up chip by: always
     * initialised.  Newer ones only when RX/TX are still disabled. */
    if (d->chip_priv[PRIV_VERSION] >= 0x30 &&
        in_req(d, CH34X_REQ_READ_REG, CH34X_REG_LCR2 << 8 | CH34X_REG_LCR, 0,
               buf, sizeof buf) != 0)
        return -1;
    if (d->chip_priv[PRIV_VERSION] < 0x30 ||
        (buf[0] & (CH34X_LCR_ENABLE_RX | CH34X_LCR_ENABLE_TX)) !=
        (CH34X_LCR_ENABLE_RX | CH34X_LCR_ENABLE_TX)) {
        if (out_req(d, CH34X_REQ_SERIAL_INIT, 0, 0) != 0)
            return -1;
        if (d->chip_priv[PRIV_VERSION] < 0x30)
            usbs_log(LOG_WARNING, "usb: ch34x version 0x%02x has no LCR: initialised on "
                     "every start, RTS/DTR may drop on a restart", d->chip_priv[PRIV_VERSION]);
        usbs_log(LOG_INFO, "usb: ch34x version 0x%02x%s: initialised", d->chip_priv[PRIV_VERSION],
                 (d->chip_priv[PRIV_QUIRKS] & QUIRK_SIMULATE_BREAK) ? ", simulated break" : "");
    } else {
        usbs_log(LOG_INFO, "usb: ch34x version 0x%02x%s: already set up, state kept",
                 d->chip_priv[PRIV_VERSION],
                 (d->chip_priv[PRIV_QUIRKS] & QUIRK_SIMULATE_BREAK) ? ", simulated break" : "");
    }
    return 0;
}

const struct usbs_chip usbs_chip_ch34x = {
    .name      = "ch34x",
    .ids       = ch34x_ids,
    .iface     = 0,
    .ep_in     = 0x82,
    .ep_out    = 0x02,
    .init      = ch34x_init,
    .set_line  = ch34x_set_line,
    .set_modem = ch34x_set_modem,
    .set_break = ch34x_set_break,
};
