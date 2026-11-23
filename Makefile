#
# Copyright (c) OpenIPC  https://openipc.org  MIT License
#
# Makefile — build rules for Vectis
#
# `make` builds `vectis` from vectis.c: the serial console, RFC 2217
# client/server and web power console, with -U: a USB-UART driven from
# user space through usbfs (usbserial.c + one <chip>.c per family in
# USB_CHIPS), for hosts whose kernel has no driver for the adapter.  The
# chip is picked by its USB VID:PID at run time.
#
#   make                    vectis with -U (USB_CHIPS = ch34x: CH340/CH341)
#   make USB=0              vectis without -U (tty and RFC 2217 only)
#   make WEBTERM=0          vectis without the web terminal on the -w page
#                           (no /ws/terminal, no built-in xterm.js)
#   make FONTS=0            vectis without the brass fonts of the -w page
#                           (~44 KB; the page uses the fallback fonts)
#
# vectis-bootrom.c is the original
# UART bridge with the HiSilicon BOOTROM-CATCH extension; it is kept for
# reference and is not part of the default build (`make vectis-bootrom`
# builds it on demand).
#

TARGET  := vectis
CC      ?= cc

# USB-UART chip families for -U, all linked into vectis.
USB       ?= 1
USB_CHIPS ?= ch34x

# The UART terminal of the -w page: xterm.js (webterm_assets.h, made by
# web/gen-assets.sh from web/xterm/) and the WebSocket /ws/terminal.
WEBTERM ?= 1

ifeq ($(WEBTERM),1)
WEBTERM_DEPS  := webterm_assets.h
else
WEBTERM_FLAGS := -DVECTIS_NO_WEBTERM
endif

# The brass fonts of the -w page (web/fonts/*.woff2, brand's brass 1.5.0):
# webfonts_assets.h is made here by web/gen-fonts.sh and never committed.
FONTS ?= 1

ifeq ($(FONTS),1)
FONTS_DEPS  := webfonts_assets.h
FONTS_FLAGS := -DVECTIS_FONTS
endif

ifeq ($(USB),1)
USB_SRC   := usbserial.c $(addsuffix .c,$(USB_CHIPS))
USB_DEPS  := $(USB_SRC) usbserial.h
USB_FLAGS := -DVECTIS_USB $(foreach c,$(USB_CHIPS),-DVECTIS_CHIP_$(shell echo '$(c)' | tr '[:lower:]-' '[:upper:]_'))
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

all: $(TARGET)

# The USB, WEBTERM and FONTS settings decide what goes into vectis, so a change of
# USB, USB_CHIPS, WEBTERM or FONTS rebuilds it without `make clean`: .build-config is
# rewritten only when they differ from the last build.
BUILD_CONFIG := USB=$(USB) USB_CHIPS=$(USB_CHIPS) WEBTERM=$(WEBTERM) FONTS=$(FONTS)
.build-config: FORCE
	@echo '$(BUILD_CONFIG)' | cmp -s - $@ || echo '$(BUILD_CONFIG)' > $@

# USB=1: -DVECTIS_USB -DVECTIS_CHIP_<CHIP> for every chip, vectis.c usbserial.c <chip>.c
$(TARGET): vectis.c $(USB_DEPS) $(WEBTERM_DEPS) $(FONTS_DEPS) .build-config
	$(CC) $(CPPFLAGS) $(CFLAGS) $(USB_FLAGS) $(WEBTERM_FLAGS) $(FONTS_FLAGS) -o $@ vectis.c $(USB_SRC) $(LDFLAGS)
	@if [ "$(DEBUG)" = "0" ]; then $(STRIP) $@; fi

# web/fonts itself too: a removed font changes the directory, not a file left
webfonts_assets.h: web/gen-fonts.sh web/fonts $(sort $(wildcard web/fonts/*.woff2))
	sh web/gen-fonts.sh $@

# Optional, not built by `make all`.
vectis-bootrom: vectis-bootrom.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $< $(LDFLAGS)
	@if [ "$(DEBUG)" = "0" ]; then $(STRIP) $@; fi

# vectis-<chip>: the separate USB builds before 1.8.0, removed if left over.
clean:
	rm -f $(TARGET) vectis-bootrom $(addprefix vectis-,$(USB_CHIPS)) .build-config webfonts_assets.h

.PHONY: all clean FORCE
