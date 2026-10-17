#
# Copyright (c) OpenIPC  https://openipc.org  MIT License
#
# Makefile — build rules for Vectis
#
# `make` builds `vectis` from vectis.c: the serial console, RFC 2217
# client/server and web power console; and, for every chip family in
# USB_CHIPS, `vectis-<chip>`: the same program plus -U, the USB-UART
# driven from user space through usbfs (usbserial.c + <chip>.c), for hosts
# whose kernel has no driver for the adapter.
#
#   make                    vectis and vectis-ch34x
#   make vectis             tty and RFC 2217 only
#   make vectis-ch34x       with -U for CH340/CH341
#   make USB=0              vectis only
#
# vectis-bootrom.c is the original
# UART bridge with the HiSilicon BOOTROM-CATCH extension; it is kept for
# reference and is not part of the default build (`make vectis-bootrom`
# builds it on demand).
#

TARGET  := vectis
CC      ?= cc

# USB-UART chip families for -U, one binary each (vectis-<chip>).
USB       ?= 1
USB_CHIPS ?= ch34x

ifeq ($(USB),1)
USB_TARGETS := $(addprefix vectis-,$(USB_CHIPS))
endif
STRIP   ?= strip

# Set DEBUG=1 on the command line to build with debug symbols and no strip:
#   make DEBUG=1
DEBUG ?= 0

ifeq ($(DEBUG),1)
CFLAGS  ?= -std=gnu99 -Wall -Wextra -Wpedantic -g -O0
LDFLAGS ?=
else
CFLAGS  ?= -std=gnu99 -Wall -Wextra -Wpedantic -Os -ffunction-sections -fdata-sections
LDFLAGS ?= -Wl,--gc-sections
endif

all: $(TARGET) $(USB_TARGETS)

$(TARGET): vectis.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $< $(LDFLAGS)
	@if [ "$(DEBUG)" = "0" ]; then $(STRIP) $@; fi

# vectis-ch34x: -DVECTIS_USB -DVECTIS_CHIP_CH34X, vectis.c usbserial.c ch34x.c
vectis-%: vectis.c usbserial.c usbserial.h %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -DVECTIS_USB \
		-DVECTIS_CHIP_$(shell echo '$*' | tr '[:lower:]-' '[:upper:]_') \
		-o $@ vectis.c usbserial.c $*.c $(LDFLAGS)
	@if [ "$(DEBUG)" = "0" ]; then $(STRIP) $@; fi

# Optional, not built by `make all`.
vectis-bootrom: vectis-bootrom.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $< $(LDFLAGS)
	@if [ "$(DEBUG)" = "0" ]; then $(STRIP) $@; fi

clean:
	rm -f $(TARGET) vectis-bootrom $(addprefix vectis-,$(USB_CHIPS))

.PHONY: all clean
