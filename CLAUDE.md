# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and test

```sh
make                          # vectis with -U (USB_CHIPS = ch34x), -Os, stripped
make USB=0                    # vectis without -U (tty and RFC 2217 only)
make DEBUG=1                  # -g -O0, no strip
make vectis-bootrom           # the original bridge, not part of `make all`
make clean                    # run it when switching CC: make compares timestamps only
python3 tests/test_vectis.py .   # regression tests, Python 3 stdlib only, no hardware
```

C99/GNU99, libc + Linux headers only, no libusb. `-Wall -Wextra -Wpedantic` is the
quality gate: CI fails on any `warning:` in the build log, for x86_64, arm, aarch64 and
mipsel, with and without `USB=0`. Cross builds: `make CC=<prefix>gcc STRIP=<prefix>strip`.
The release binaries are static musl builds from `ci/build-static.sh` (x86_64, in
`alpine:3.22`) and `ci/build-mipsel.sh` (soft-float MIPS32r2, OpenWrt toolchain).

The tests use a pty pair in place of the serial device and cover the `-l` server (raw and
Telnet/RFC 2217), the `-w` web API, the RFC 2217 client against `-l`, stdin EOF and device
loss without a busy loop, and the `-U` command line. Add a test there for any behaviour
change that can be exercised without hardware.

## Layout

- `vectis.c` → **`vectis`**, the one shipped binary. Three roles side by side:
  console (local tty `-u`, USB-UART via usbfs `-U`, or RFC 2217 client `-h/-p`),
  TCP server for the local port (`-l`), web power console with a JSON API (`-w`).
  It is the former `vectis-cli`; the name `vectis-cli` and the file `vectis-cli.c` are gone.
- `usbserial.c` / `usbserial.h` → the `-U` transport: usbfs device lookup, interface
  claim, bulk URBs, receive backlog. Compiled in only with `-DVECTIS_USB` (`USB=1`).
- `ch34x.c` → the CH340/CH341 chip family. One file per family, listed in `USB_CHIPS`;
  a new family implements `struct usbs_chip` (`usbserial.h`) and gets
  `-DVECTIS_CHIP_<NAME>`. The chip is picked at run time by VID:PID.
- `vectis-bootrom.c` → **`vectis-bootrom`**, the original UART bridge (version line
  1.3.x) with the HiSilicon BOOTROM-CATCH extension, kept for reference and built by CI.
  Its own constants use the `TN_*` / `COMPORT_*` prefixes; `vectis.c` uses `IAC` / `CPC_*`.
  See `docs/vectis-bootrom.md`.
- `tests/test_vectis.py`, `ci/`, `.github/workflows/{build,version}.yml`,
  `.github/check-version.sh`.

## Invariants

Easy to break and not obvious from one function — read the surrounding block comments in
`vectis.c` before touching these.

1. **`-l` detects RFC 2217 per connection, passively.** Every client starts in legacy raw
   mode (bytes pass through, LF/CRLF → CR, `Ctrl+P` 0x10 fires the reset pulse) and flips
   to Telnet/RFC 2217 for the rest of the connection on its first `IAC` (0xFF) byte. In
   Telnet mode the data path is binary safe (IAC escaped both ways) and the client may set
   baud, data/stop bits, parity, DTR/RTS, purge and break. Don't add a server-side
   "negotiate first" handshake: it breaks plain `nc`/`socat` users.
2. **`-l` serves one client at a time.** `g_srv_fd` is a single fd; a new `accept()`
   replaces the current client. Multi-client would need a decision about what the reset
   pulse and port settings mean across clients first.
3. **RFC 2217 numbering follows the RFC.** `CPC_CTRL_RTS_ON = 11`, `RTS_OFF = 12`,
   `DTR_ON = 8`, `DTR_OFF = 9`; server replies are sub-option + 100 (`CPC_SERVER_OFFSET`).
   pyserial's `rfc2217://` depends on this. `vectis-bootrom.c` has the same values.
4. **Interactive vs piped stdin.** `g_interactive = isatty(STDIN_FILENO)`. Only when
   interactive is the terminal put in raw mode and are `Ctrl+P` / `Ctrl+B` / `Ctrl+]`
   session keys; piped stdin is forwarded verbatim, and after stdin EOF vectis keeps
   copying device output to stdout. `-n` separately disables LF → CR+LF. Status messages
   go to stderr and syslog so stdout carries only device data
   (`echo cmd | timeout 3 vectis ... -n > out`).
5. **RTS/DTR contract differs by transport.** With `-u` opening the tty asserts both lines
   and closing it drops them, so a restart power-cycles the target. With `-U` the lines are
   set on start without being dropped first and left as they are on exit, so a restart or
   update does not touch the target. Chip `init` must not change the modem lines, and
   `usbs_close()` must not hand the interface back to the kernel driver (its probe would
   drop them). RTS and DTR always move together.
6. **BOOTROM-CATCH lives in `vectis-bootrom.c` only.** Vendor sub-option 50, reply 150:
   the HiSilicon `0x20` marker / `0xAA` ack handshake runs on the server because a remote
   client over > ~25 ms RTT misses the ~100 ms download-mode window. The request is
   variable length, the reply a fixed prefix plus appended counters; keep it compatible
   in both directions. The wire format is in the comment above `COMPORT_BOOTROM_CATCH`.

## Versioning and release

- `PROGRAM_VERSION` / `PROGRAM_RELEASE_DATE` near the top of `vectis.c` (`MAJOR.MINOR.PATCH`).
  `vectis-bootrom.c` keeps its own, separate version.
- Pull requests that touch code or build (`*.c`, `*.h`, `Makefile`, `ci/`,
  `.github/workflows/build.yml`, Markdown excluded) must bump `PROGRAM_VERSION` and add a
  `## <version> — YYYY-MM-DD` section to `CHANGELOG.md`; `.github/check-version.sh`
  enforces it (`version.yml`). The `no-release` label lets through a change that leaves
  the release files as they are. Docs, tests and the rest of `.github/` need no bump.
- Every push to `master` that passes all builds replaces the single `latest` release
  (`vectis-linux-x86_64`, `vectis-linux-mipsel`, no version in the file names); no `v*`
  tags. A new version is also posted to the OpenIPC Telegram dev channel.
- Keep the three READMEs (`README.md`, `README.ru.md`, `README.zh-CN.md`) in step when
  changing user-visible behaviour or options; `usage()` in `vectis.c` is the option reference.
