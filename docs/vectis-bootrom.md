# vectis-bootrom: the original UART bridge

[Back to the README](../README.md)

`vectis-bootrom.c` is the program this repository started with: a UART
terminal and TCP bridge for the OpenIPC UART 38x38 board with the same
RTS/DTR reset pulse, later extended with RFC 2217 auto-detection on its
TCP listener, inetd/inittab integration, and a vendor extension,
`BOOTROM-CATCH`, that runs the HiSilicon boot-ROM `0x20`-marker /
`0xAA`-ack handshake next to the UART so it works over links with high
round-trip time. Everything else it did is now covered by `vectis`
(`-u` for the console, `-l` for the TCP side, `-w` for power control),
so the file is kept for reference and for that boot-ROM extension. It is
not built by default:

```sh
make vectis-bootrom        # or: cc -std=gnu99 -Os -o vectis-bootrom vectis-bootrom.c
```

In this document `Vectis` and `vectis-bootrom` mean this original binary.

## Usage

```sh
./vectis-bootrom [options]
```

### Options

- `-p, --port <port>` — serial device path, default: `/dev/ttyUSB0`
- `-b, --baud <rate>` — baud rate, default: `115200`. Supported rates:
  `9600`, `19200`, `38400`, `57600`, `115200`, `230400`, `460800`, `921600`
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
- The TCP listener accepts one client at a time; a new connection
  replaces the current client.
- The TCP listener starts only when `-t` is provided. When `-t` is given
  and stdin is not a terminal (for example a backgrounded
  `vectis -t > log 2>&1 &`), stdin is ignored and the console hotkeys are
  unavailable.
- If the serial device disappears (a USB adapter is unplugged), Vectis
  logs `UART closed` and exits with status 1. Use the `inittab` or
  `inetd` respawn setups below to bring it back automatically.
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
  (values `8`/`9` = DTR ON/OFF, `11`/`12` = RTS ON/OFF, as assigned by
  RFC 2217 §3.5; `10` reports the current RTS state and `0` the current
  DTR state; anything else, such as flow-control settings, is
  acknowledged as received so the client never hangs waiting);
- accepts `SET-BAUDRATE` (the same fixed list that `-b` supports:
  `9600`, `19200`, `38400`, `57600`, `115200`, `230400`, `460800`,
  `921600`). A rate outside the list is answered with the rate still in
  effect, and `0` queries the current rate;
- acknowledges `SET-DATASIZE`, `SET-PARITY` and `SET-STOPSIZE` with the
  values actually in effect. The UART is hard-wired to 8N1, so the reply
  is always `8`, none, `1` regardless of the request (pyserial waits
  synchronously for these acks when opening the port);
- acknowledges `PURGE-DATA` (`1` = RX, `2` = TX, `3` = both) and flushes
  the matching UART queue;
- replies to `SIGNATURE` with `Vectis <version>`;
- accepts a vendor-extension sub-option `BOOTROM-CATCH` (50) that
  pulses RTS/DTR and runs the HiSilicon bootrom `0x20`-marker /
  `0xAA`-ack handshake locally — see "BOOTROM-CATCH" below.

### BOOTROM-CATCH (vendor extension, sub-option 50)

The HiSilicon boot ROM emits `0x20` markers and listens for `0xAA`
in a ~100 ms window after reset.  Over a network with one-way
latency above ~25 ms a remote client cannot complete a marker→ack
round trip in time, so the camera autoboots before the client sees
the markers.  The `BOOTROM-CATCH` sub-option moves the catch loop
into Vectis itself, next to the UART, where the response time is
in microseconds — RTT and jitter no longer matter.

Wire format (client → server):

```
IAC SB COMPORT BOOTROM-CATCH
    <pulse_ms     : 4 bytes BE>
    <max_wait_ms  : 4 bytes BE>
    [<mode        : 1 byte>]   # optional, default 0 = MARKER
    [<min_markers : 1 byte>]   # optional, MARKER threshold; 0 → default (5)
    [<head_frame  : ≤ 64 bytes>] # optional pipelined HEAD frame
IAC SE
```

- `pulse_ms` (clamped to `0..5000`) — RTS+DTR low time before the
  catch loop starts.  Set to `0` when external hardware (e.g. a
  PoE switch) handles the actual reset.
- `max_wait_ms` (clamped to `100..30000`) — overall catch deadline.
  `5000` is a comfortable default.
- `mode` (optional, default `0`):
  - `0  MARKER` — wait for `min_markers` consecutive `0x20` bytes
    from the chip, then send a final `0xAA`.  Matches the documented
    hi3516 bootrom protocol; finishes early (typically ~500 ms) when
    the chip emits markers.  Returns ``OK`` on confirmation,
    ``TIMEOUT`` if no qualifying run within `max_wait_ms`.
  - `1  BLIND` — keep blasting `0xAA` for the full `max_wait_ms`
    and unconditionally return ``OK``.  Use for chip variants that
    enter download mode silently without emitting markers we can
    observe.  The client confirms by sending a HEAD frame and
    watching for an ACK.
- `min_markers` (optional, MARKER mode only).  How many *consecutive*
  `0x20` bytes the chip must emit before the catch is declared
  successful.  `0` (and the no-byte default) → 5, the canonical
  HiSilicon protocol value.  Some hi3516 variants emit shorter
  bursts and need `4` (or even `3`).  Clamped server-side to `1..16`.
  An adaptive client can issue the catch with `0`, observe the
  reply's `max_marker_run`, and retry with `min_markers =
  max_marker_run` if the first attempt timed out — purely
  protocol-driven, no operator intervention.

- `head_frame` (optional, any bytes appended after byte 10).  The
  *first* HEAD frame the client wants to send to the chip after the
  catch.  When present, Vectis writes the final `0xAA` followed —
  with no network round trip in between — by the HEAD frame and
  captures the chip's reply byte locally.  This is the bit that
  closes the RTT-window problem: on links where the chip's
  download-mode hold window is shorter than one client→server round
  trip, the chip otherwise moves on to SPL before any client-issued
  HEAD can arrive.  Capped server-side to 64 bytes (a HiSilicon
  HEAD frame is 14, with margin for variants).

Older clients that send only the 8/9/10-byte payload default to
MARKER mode, the canonical threshold, and no HEAD pipelining
respectively — no behaviour change.

Wire format (server → client):

```
IAC SB COMPORT (BOOTROM-CATCH+100=150)
    <status        : 1 byte>      0=ok / 1=timeout / 2=io_err
    <markers_seen  : 4 bytes BE>  total 0x20 bytes counted
    <max_marker_run: 1 byte>      longest consecutive 0x20 run (capped 255)
    <bytes_rx      : 4 bytes BE>  total bytes received from UART
    <bytes_tx      : 4 bytes BE>  total 0xAA bytes blasted
    <elapsed_ms    : 4 bytes BE>  actual catch-loop runtime
    <last_byte     : 1 byte>      last non-marker byte (0 if none)
    <head_ack      : 1 byte>      chip's reply to the pipelined HEAD,
                                  or 0 if no HEAD was provided
    <head_ack_seen : 1 byte>      1 if any byte arrived during the
                                  post-HEAD window (so a `head_ack=0`
                                  can be told from "no reply at all")
IAC SE
```

The trailing counters are diagnostic: a high-RTT or noisy-link
client can see whether the catch failed because the chip really
emitted nothing (`bytes_rx=0`), because a different byte family
arrived (`last_byte` non-zero, `markers_seen=0`), or because some
markers arrived but never strung together (`markers_seen` high but
`max_marker_run < 5`).  Older clients that read just the first
byte (status) still work — the extra counters are appended after
it.

While the catch loop runs, Vectis owns the local UART exclusively
(no bytes forwarded to the client).  After it returns, normal
forwarding resumes — the client can immediately send a `HEAD` frame
and proceed with the firmware upload.

Example client (Python, with `pyserial`):

```python
import serial, struct, socket
ser = serial.serial_for_url("rfc2217://vectis.lan:35240", baudrate=115200)
sock = ser._socket
# Ask Vectis to do a 200 ms RTS/DTR pulse, then catch the bootrom
# locally with a 5-second deadline.  Mode 0 = MARKER (default), 1 = BLIND.
sock.sendall(
    b"\xff\xfa\x2c\x32"
    + struct.pack(">IIB", 200, 5000, 0)
    + b"\xff\xf0",
)
# (Read the IAC SB 44 150 <status> IAC SE reply with your IAC parser.)
ser.write(head_frame)            # bootrom is now in download mode
print(ser.read(1))                # → b"\xaa"
```

RFC 2217 detection runs on **both** TCP entry points:

- the standalone `-t [port]` listener, and
- the inetd `nowait` socket (where stdin/stdout *is* the TCP socket).

The legacy `Ctrl+P` hotkey on the local interactive console (no `-t`,
stdin is a tty) is unchanged — humans don't speak Telnet by hand, so
the detection is a no-op for them.

## The inetd integration

Vectis can also be launched from `inetd`:

```conf
# OpenIPC
#
35240   stream  tcp     nowait  root    /usr/local/sbin/vectis-bootrom vectis-bootrom -s -p /dev/ttyUSB0 -b 115200
```

In `nowait` mode each TCP connection spawns a fresh Vectis with the
socket wired to its stdin/stdout. RFC 2217 detection works on this
path too: a programmatic client sending `IAC` flips the connection
into Telnet mode, sub-options like `SET-CONTROL` are answered over
the same socket, and `0xFF` bytes are escaped per RFC 854.

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
ttyS0::respawn:/usr/local/sbin/vectis-bootrom -s -p /dev/ttyUSB0 -b 115200
```

[rfc2217]: https://www.rfc-editor.org/rfc/rfc2217
