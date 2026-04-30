#
# Copyright (c) OpenIPC  https://openipc.org  MIT License
#
# Makefile — build rules for the Vectis UART utility
#

TARGET := vectis
CC ?= cc
CFLAGS ?= -std=gnu99 -Wall -Wextra -Wpedantic -Os -ffunction-sections -fdata-sections
LDFLAGS ?= -Wl,--gc-sections

all: $(TARGET)

$(TARGET): vectis.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f $(TARGET)

.PHONY: all clean
