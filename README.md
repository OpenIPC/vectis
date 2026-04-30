# Vectis

Vectis is intended for fast debugging and device control during development and integration of embedded systems with the OpenIPC UART 38x38 board.
It opens a serial port, bridges keyboard input to UART, prints UART data to the terminal, and can generate a reset pulse on RTS/DTR lines.
It can also expose the same UART bridge over TCP/IP for one remote client at a time.

## Features

- Configurable serial port and baud rate
- Optional TCP listener port
- Raw terminal input/output bridge
- TCP/IP client bridge
- RTS/DTR status display
- Ctrl+P reset pulse on RTS/DTR
- Syslog logging for key events and errors

## Build

```sh
make
```

## Usage

```sh
./vectis [options]
```

### Options

- `-p, --port <port>` — serial device path, default: `/dev/ttyUSB0`
- `-b, --baud <rate>` — baud rate, default: `115200`
- `-t, --tcp-port [port]` — enable TCP listener, default port: `35240`
- `-s, --status` — print RTS/DTR status
- `-v, --version` — print version and build date
- `-h, --help` — show help

### Hotkeys

- `Ctrl+P` — generate an inverted reset pulse on RTS and DTR
- `Ctrl+C` — exit
- `Ctrl+X` — exit

## Notes

- UART is configured for `8N1`.
- RTS and DTR are asserted on startup and deasserted on exit.
- RTS and DTR use normal asserted/deasserted signaling.
- The Ctrl+P pulse uses the opposite asserted/deasserted sequence.
- This tool is designed for rapid debugging and control, including power-related device handling, during integration work with OpenIPC UART 38x38.
- The TCP listener accepts one client at a time.
- The TCP listener starts only when `-t` is provided.
- For interactive TCP use, run the local terminal side in raw mode so keys like `Ctrl+P` are sent immediately without waiting for `Enter`.
- Example:

```sh
socat -,raw,echo=0 TCP:192.168.1.10:35240
```

```sh
nc -C 192.168.1.10 35240
```

## The inetd integration

Vectis can also be launched from `inetd`:

```conf
# OpenIPC
#
35240   stream  tcp     nowait  root    /usr/local/sbin/vectis vectis -s -p /dev/ttyUSB0 -b 115200
```

On Debian/Ubuntu, install `inetd` with:

```sh
sudo apt update
sudo apt install inetutils-inetd
```

## The inittab integration

Vectis can also be started from `inittab` on systems with BusyBox init:

```conf
# OpenIPC
#
ttyS0::respawn:/usr/local/sbin/vectis -s -p /dev/ttyUSB0 -b 115200
```
