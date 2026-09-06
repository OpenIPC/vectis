#
# Copyright (c) OpenIPC  https://openipc.org  MIT License
#
# Makefile — build rules for Vectis
#
# `make` builds `vectis` from vectis.c: the serial console, RFC 2217
# client/server and web power console.  vectis-bootrom.c is the original
# UART bridge with the HiSilicon BOOTROM-CATCH extension; it is kept for
# reference and is not part of the default build (`make vectis-bootrom`
# builds it on demand).
#

TARGET  := vectis
CC      ?= cc
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

$(TARGET): vectis.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $< $(LDFLAGS)
	@if [ "$(DEBUG)" = "0" ]; then $(STRIP) $@; fi

# Optional, not built by `make all`.
vectis-bootrom: vectis-bootrom.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $< $(LDFLAGS)
	@if [ "$(DEBUG)" = "0" ]; then $(STRIP) $@; fi

clean:
	rm -f $(TARGET) vectis-bootrom

.PHONY: all clean
