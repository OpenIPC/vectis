# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and test

```sh
make                          # vectis with -U (USB_CHIPS = ch34x), -Os, stripped
make USB=0                    # vectis without -U (tty and RFC 2217 only)
make WEBTERM=0                # vectis without the web terminal (/ws/terminal, xterm.js)
make FONTS=0                  # vectis without the page's brass fonts (/fonts/*.woff2)
make DEBUG=1                  # -g -O0, no strip
make vectis-bootrom           # the original bridge, not part of `make all`
make clean                    # run it when switching CC: make compares timestamps only
python3 tests/test_vectis.py .   # regression tests, Python 3 stdlib only, no hardware
tests/android/vpn-yield-e2e.sh   # vectis and the camex app on one emulator (Docker, KVM): one VPN at a time
```

C99/GNU99, libc + Linux headers only, no libusb. `-Wall -Wextra -Wpedantic` is the
quality gate: CI fails on any `warning:` in the build log, for x86_64, arm, aarch64 and
mipsel, with and without `USB=0`, `WEBTERM=0` and `FONTS=0`. Cross builds: `make CC=<prefix>gcc STRIP=<prefix>strip`.
The release binaries are static musl builds from `ci/build-static.sh` (x86_64, in
`alpine:3.22`) and `ci/build-mipsel.sh` (soft-float MIPS32r2, OpenWrt toolchain).

The tests use a pty pair in place of the serial device and cover the `-l` server (raw and
Telnet/RFC 2217), the `-w` web API, the web terminal (WebSocket) and password, the
RFC 2217 client against `-l`, stdin EOF and device
loss without a busy loop, the `-U` command line, and `-U fd:@NAME` against a fake
app socket. Add a test there for any behaviour
change that can be exercised without hardware.

## Layout

- `vectis.c` → **`vectis`**, the one shipped binary. Three roles side by side:
  console (local tty `-u`, USB-UART via usbfs `-U`, or RFC 2217 client `-h/-p`),
  TCP server for the local port (`-l`), web power console with a JSON API and a UART
  terminal (`-w`: WebSocket `/ws/terminal`, optional password `-W`/`VECTIS_WEB_PASSWORD`).
  It is the former `vectis-cli`; the name `vectis-cli` and the file `vectis-cli.c` are gone.
- `web/xterm/` → xterm.js and its fit addon (MIT, pinned versions, `web/xterm/README.md`).
  `sh web/gen-assets.sh` turns them into **`webterm_assets.h`** (gzipped C arrays), which is
  committed so every build (make, the NDK, plain cc) has it; CI checks the two agree.
  `make WEBTERM=0` (`-DVECTIS_NO_WEBTERM`) leaves the terminal out.
- `web/fonts/` → the page's brass fonts (brand's brass 1.5.0: Brass Serif, Brass Mono,
  `OFL.txt`), byte for byte as brand's. `web/gen-fonts.sh` turns the `.woff2` files into
  **`webfonts_assets.h`** at build time (make, `android/build-native.sh`); it is never committed,
  unlike `webterm_assets.h`. vectis serves them at `/fonts/<file>` (`-DVECTIS_FONTS`); without
  it (`make FONTS=0`, a plain cc) the page has no `@font-face`. CSS names fonts only in the
  `--serif`/`--mono` tokens.
- `usbserial.c` / `usbserial.h` → the `-U` transport: usbfs device lookup, interface
  claim, bulk URBs, receive backlog. Compiled in only with `-DVECTIS_USB` (`USB=1`).
- `ch34x.c` → the CH340/CH341 chip family. One file per family, listed in `USB_CHIPS`;
  a new family implements `struct usbs_chip` (`usbserial.h`) and gets
  `-DVECTIS_CHIP_<NAME>`. The chip is picked at run time by VID:PID.
- `vectis-bootrom.c` → **`vectis-bootrom`**, the original UART bridge (version line
  1.3.x) with the HiSilicon BOOTROM-CATCH extension, kept for reference and built by CI.
  Its own constants use the `TN_*` / `COMPORT_*` prefixes; `vectis.c` uses `IAC` / `CPC_*`.
  See `docs/vectis-bootrom.md`.
- `android/` → the Android app (`org.openipc.vectis`): a foreground service owns the
  USB adapter (UsbManager) and runs vectis, built by `android/build-native.sh` as
  `libvectis.so`, with `-U fd:@org.openipc.vectis.usb`; a fresh `openDevice()` per
  connection; web port 8085 by default (taken: the next free one, `freePort()`), the
  web page password from the settings in `VECTIS_WEB_PASSWORD`; the app's own requests
  (`Http`) and its terminal go to `--web-local @org.openipc.vectis.web` (an abstract
  unix socket, own UID, no password), so they work with remote access off (no `-l`,
  no `-w`); `TerminalActivity` is a native screen on `/ws/terminal`
  (`WsCodec`, `TermBuffer`). Remote access through a camex server (#97): the submodule
  `bundles/camex` (pinned to a camex release; `git submodule update --init`), its tunnel
  library as `:camex-tunnel`, started and stopped with vectis by the service
  (`startTunnel()`), its settings in `Settings.tunnel*`. Version from `PROGRAM_VERSION`. Build: `cd android && gradle assembleDebug`;
  tests of the device-free logic: `gradle testDebugUnitTest` (`android/app/src/test`).
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
7. **The web terminal never blocks the poll loop.** Device output reaches it through
   `wt_broadcast()` wherever it reaches stdout and the `-l` client (both serial paths and
   `process_incoming()`); each terminal has its own queue, sent non-blocking, and one that
   falls `WT_OUT_MAX` behind is dropped. Input is verbatim: no session keys, `Ctrl+P` is a
   byte (the page's button resets). The password (`-W`) guards every `-w` request, the
   upgrade included; `-l` stays without one (invariant 1).
8. **Device writes never block the poll loop for long.** `link_write()` only queues
   (`g_txq`, the "Transmit queue for the device" block); `txq_pump()` sends one piece per
   pass (a `usbs_out_chunk()` of about 10 ms with `-U`, a non-blocking `write()` otherwise).
   Sources (`-l` client, web terminals, stdin) are polled and read only within `txq_room()`.
   Anything that changes the line or writes to it directly (settings, break, RTS/DTR,
   `write_all()` to `g_fd`) calls `txq_flush()` first, so it keeps its place after earlier
   data; a transmit purge and `disconnect()` empty the queue. A device that takes nothing for
   `TXQ_STALL_MS` (1 s when leaving) loses the queue with a warning, so nothing waits on it
   forever: the sources are read again and power, reset and settings still apply.

## Versioning and release

- `PROGRAM_VERSION` / `PROGRAM_RELEASE_DATE` near the top of `vectis.c` (`MAJOR.MINOR.PATCH`).
  `vectis-bootrom.c` keeps its own, separate version.
- Pull requests that touch code or build (`*.c`, `*.h`, `web/` (fonts included), `Makefile`, `ci/`,
  `.github/workflows/build.yml`, `android/`, `bundles/`, `.gitmodules`, Markdown excluded) must bump `PROGRAM_VERSION` and add a
  `## <version> — YYYY-MM-DD` section to `CHANGELOG.md`; `.github/check-version.sh`
  enforces it (`version.yml`). The `no-release` label lets through a change that leaves
  the release files as they are. Docs, tests and the rest of `.github/` need no bump.
- Every push to `master` that passes all builds replaces the single `latest` release
  (`vectis-linux-x86_64`, `vectis-linux-mipsel`, `vectis-android.apk`, no version in the
  file names); no `v*` tags. The APK is signed with the project's release key (repository
  secrets `VECTIS_KEYSTORE_*`; the `android-release` job checks the certificate's
  SHA-256). A new version is also posted to the OpenIPC Telegram dev channel.
- Keep the three READMEs (`README.md`, `README.ru.md`, `README.zh-CN.md`) in step when
  changing user-visible behaviour or options; `usage()` in `vectis.c` is the option reference.
