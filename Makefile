#
# Copyright (c) OpenIPC  https://openipc.org  MIT License
#
# Makefile — build rules for the Vectis utilities
#

TARGET := vectis
CLI_TARGET := vectis-cli
CC ?= cc
STRIP ?= strip
CFLAGS ?= -std=gnu99 -Wall -Wextra -Wpedantic -Os -ffunction-sections -fdata-sections
LDFLAGS ?= -Wl,--gc-sections

all: $(TARGET) $(CLI_TARGET)

$(TARGET): vectis.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $< $(LDFLAGS)
	$(STRIP) $@

$(CLI_TARGET): vectis-cli.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $< $(LDFLAGS)
	$(STRIP) $@

clean:
	rm -f $(TARGET) $(CLI_TARGET)

.PHONY: all clean
