#
# Copyright (c) OpenIPC  https://openipc.org  MIT License
#
# Makefile — build rules for the Vectis utilities
#

TARGET     := vectis
CLI_TARGET := vectis-cli
CC         ?= cc
STRIP      ?= strip

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

all: $(TARGET) $(CLI_TARGET)

$(TARGET): vectis.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $< $(LDFLAGS)
	@if [ "$(DEBUG)" = "0" ]; then $(STRIP) $@; fi

$(CLI_TARGET): vectis-cli.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $< $(LDFLAGS)
	@if [ "$(DEBUG)" = "0" ]; then $(STRIP) $@; fi

clean:
	rm -f $(TARGET) $(CLI_TARGET)

.PHONY: all clean
