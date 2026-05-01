# Vectis

Vectis is intended for fast debugging and device control during development and integration of embedded systems with the OpenIPC UART 38x38 board.
It opens a serial port, bridges keyboard input to UART, prints UART data to the terminal, and can generate a reset pulse on RTS/DTR lines.
It can also expose the same UART bridge over TCP/IP for one remote client at a time.

![image](device.jpg)

## Features

- Configurable serial port and baud rate
- Optional TCP listener port
- Raw terminal input/output bridge
- TCP/IP client bridge (legacy raw mode + auto-detected RFC 2217)
- RTS/DTR status display
- Ctrl+P reset pulse on RTS/DTR
- Syslog logging for key events and errors
- **RFC 2217** (Telnet COM Port Control Option, [RFC 2217][rfc2217])
  on the TCP listener — auto-detected per connection, no extra
  CLI flags. Gives standard clients a binary-safe data path and
  out-of-band RTS/DTR/baud-rate control.

[rfc2217]: https://datatracker.ietf.org/doc/html/rfc2217

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

## RFC 2217 mode (binary-safe + out-of-band control)

The TCP listener auto-detects whether a connecting client speaks
[RFC 2217][rfc2217] (Telnet COM-Port Control Option). Detection is
purely passive: Vectis stays in legacy raw mode until the client
transmits a Telnet `IAC` byte (`0xFF`). Interactive humans never
type `0xFF`, so existing `socat`/`nc`/`cat` workflows are unchanged.
The moment a programmatic client begins option negotiation Vectis
locks into Telnet mode for the rest of that connection and:

- stops intercepting `Ctrl+P` (`0x10`) — the data path is binary-safe;
- stops `\n`→`\r` normalisation and TCP-echo suppression;
- escapes `0xFF` in both directions as `0xFF 0xFF` per RFC 854;
- accepts `SET-CONTROL` to drive RTS/DTR independently
  (values `8`/`9` = DTR ON/OFF, `10`/`11` = RTS ON/OFF);
- accepts `SET-BAUDRATE` (the same fixed list `9600`, `19200`, `38400`,
  `57600`, `115200`, `230400` that `-b` supports);
- replies to `SIGNATURE` with `Vectis <version>`.

The legacy `Ctrl+P` hotkey on the local console (the `-t`-less stdin
path) is unchanged.

### User stories

#### A. Existing interactive user — *no change*

```sh
socat -,raw,echo=0 TCP:192.168.1.10:35240
# press Ctrl+P → camera resets, exactly like before
```

`socat` never sends `0xFF`, so Vectis stays in legacy mode.

#### B. Telnet client — *binary safe + DTR/RTS control over an out-of-band channel*

```sh
telnet 192.168.1.10 35240
```

`telnet` opens with `IAC DO SUPPRESS-GO-AHEAD` etc., so Vectis flips
into RFC 2217 mode. From the `telnet>` escape prompt you can issue
`send brk`, `set binary`, etc. The data path is now 8-bit clean.

#### C. Programmatic client — `pyserial`'s `rfc2217://` transport

```python
import serial, time
ser = serial.serial_for_url("rfc2217://192.168.1.10:35240")
ser.baudrate = 115200          # SET-BAUDRATE under the hood
ser.dtr = False; ser.rts = False
time.sleep(0.2)                # 200 ms reset pulse
ser.dtr = True;  ser.rts = True
ser.write(open("u-boot.bin", "rb").read())   # binary safe
print(ser.read(256))
```

`pyserial` performs the IAC handshake transparently, escapes any
`0xFF` byte in the firmware as `0xFF 0xFF` on the wire, and turns
`ser.dtr = False` into a `SET-CONTROL` command. This is exactly the
shape any "flash a firmware blob through the camera's UART" tool
needs, and it works against the **same TCP listener** that
interactive humans use — no second port, no flags.

#### D. ser2net interop

```sh
ser2net -t '192.168.1.10,35240,RFC2217'
```

`ser2net` natively groks Vectis's RFC 2217 server, so Vectis can be
slotted into existing serial-server fleets without new conventions.

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
