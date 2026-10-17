# Changelog

The version lives in `PROGRAM_VERSION` in `vectis.c` (`MAJOR.MINOR.PATCH`). Binaries are
published only as the [`latest`](https://github.com/OpenIPC/vectis/releases/latest) release,
rebuilt from every push to `master`; this file is the change history. Versions
up to 1.3.2 are those of the original UART bridge, now `vectis-bootrom.c`; from 1.6.0 on,
`vectis` is the former `vectis-cli`.

## 1.9.0 — 2026-10-17

A release binary for OpenWrt routers (#49) and a `-U` receive fix (#48).

- `-U`: the end of a burst is delivered at once. Bulk IN URBs were 256 bytes and complete
  only when full or on a short packet; a burst ending on a full 32-byte CH340 packet stayed
  in the URB until more data arrived, so the last 32…192 bytes of e.g. a boot log showed
  up only with the next output. Each IN URB is now one packet of the IN endpoint
  (wMaxPacketSize from the device's descriptors).
- Releases: `vectis-linux-mipsel`, static musl, soft-float MIPS32r2, for MediaTek
  MT7620/MT7621/MT7628 and other MIPS little-endian routers, with or without an FPU; built
  by `ci/build-mipsel.sh` with the OpenWrt 23.05.5 toolchain. `SHA256SUMS` covers both
  files.
- Checked on a ZBT WR8305RT (MT7620N, no FPU, OpenWrt 15.05.1, kernel 3.18, whose `ch341`
  does not bind `1a86:7522`) with a CH340 in RX/TX loopback: 1200…921600 baud,
  8N1/8E1/8O2/7N1, up to 64 KiB per burst, RTS/DTR, break, the web power console.

## 1.8.0 — 2026-10-17

One binary and one release (#39, #40).

- `-U` is part of `vectis`: the USB chip drivers of `USB_CHIPS` (now `ch34x`) are linked
  into the one binary and the chip is picked by its VID:PID. The separate `vectis-ch34x`
  build is gone; use `vectis -U` instead, the command line is the same. `make USB=0`
  still builds `vectis` without `-U`, and CI checks that it builds.
- Releases: one release, `latest`, rebuilt from every push to `master` once all builds
  pass, titled `Vectis <version>`. The file names carry no version:
  `vectis-linux-x86_64` and `SHA256SUMS` under
  `https://github.com/OpenIPC/vectis/releases/latest/download/`. The `v1.7.0` release and
  its `vectis-1.7.0-linux-x86_64` / `vectis-ch34x-1.7.0-linux-x86_64` links are gone.
- Size: the x86_64 release binary is 141 176 bytes, as `vectis-ch34x` was; for users of
  the plain build +12 KiB.
- READMEs: "Releases and permanent links", the release badge.

## 1.7.0 — 2026-10-17

USB mode without a kernel driver (#29, PR #37) and the first release builds (#33, PR #38).

- `-U`, in a second build `vectis-ch34x`: drives a CH340/CH341 USB-UART adapter from user
  space through usbfs, for routers and other hosts without the `ch341` driver. The adapter
  is found by USB path or VID:PID; RTS/DTR are kept across restarts, so a restart does not
  power-cycle the target. The console, `-l`, `-w`, Ctrl+P/B and the RFC 2217 port controls
  work as on a tty (#30); a lost adapter is reopened after an unplug or re-enumeration
  (#31).
- USB fixes from review: a failed write is a link loss (the daemon reopens the link), the
  bulk chunk size follows the baud rate (300…1200 baud no longer time out), PURGE-DATA also
  drops data already received, a break drains the chip first, old chips (before 0x30) are
  initialised without reading LCR and refuse non-8N1 settings (#30).
- Ctrl+] always exits; `-U` checks the baud rate before opening the adapter.
- `-w`: `/power/reset` is answered to clients that half-close their side (`nc -N`,
  `shutdown(SHUT_WR)`).
- `-l`: offered Telnet options are tracked as pending (RFC 1143), so a client refusing
  COM-PORT-OPTION no longer gets a redundant WONT/DONT round.
- Regression tests on a pty, no hardware needed; CI builds every platform, smoke-tests the
  binaries under qemu-user and runs the tests with a read-only token.
- Releases: static musl x86_64 binaries `vectis` and `vectis-ch34x` with `SHA256SUMS`,
  built by `ci/build-static.sh` in `alpine:3.22`.
- READMEs in English, Russian and Simplified Chinese, with the web console screenshot.

## 1.6.1 — 2026-10-15

PR #16.

- Ctrl+] works while the device link is down; other keys are dropped instead of reaching
  the device in one burst when the link is back (#12).
- The RTS+DTR reset pulse no longer blocks the poll loop: device output, the keyboard, the
  `-l` client and the web page stay served during the pulse (#15).
- `-l`: the client's acknowledgements of our Telnet offers no longer trigger a second
  WILL/DO round (#13); only an empty SIGNATURE request is answered, a client's own
  signature is logged (#14).
- `vectis-bootrom` 1.3.3: no stray 0x00 after the SET-BAUDRATE reply (#11).

## 1.6.0 — 2026-09-06

Console, RFC 2217 server and web power console in one binary.

- `vectis-cli.c` becomes `vectis.c`, the only program built by `make`; the original bridge
  is kept as `vectis-bootrom.c` (`make vectis-bootrom`).
- `-w [ADDR:]PORT`: web power console — one page (Enable / Disable / Reset on RTS+DTR) and a
  JSON API (`GET /status`, `GET|POST /power/enable|disable|reset`). No authentication: bind
  it to a trusted address.
- `-l [ADDR:]PORT`: serves the local port over TCP to one client at a time, raw until the
  first IAC, then Telnet/RFC 2217; works with telnet, another vectis, pyserial `rfc2217://`
  and nc/socat.
- Daemon behaviour: with `-l` or `-w` the listeners come up first and the device link is
  reopened automatically; SIGTERM ends the session cleanly.
- Fixes: no 100% CPU spin when stdin is a finished pipe; `-b 921600` is accepted.
- README rewritten around vectis: options, `-l` and `-w`, JSON reference, udev rule for
  stable names of several CH340 adapters, systemd units.

## 1.3.2 — 2026-06-10

- No 100% CPU spin when the USB-serial UART is unplugged (PR #10).

## 1.3.1 — 2026-06-04

- No 100% CPU spin when backgrounded with `-t` (PR #9).

## 1.3.0 — 2026-06-03

- Server baud rates 460800 and 921600 (PR #8).
- RFC 2217: inetd `nowait` mode (PR #2); DATASIZE/PARITY/STOPSIZE/PURGE are acknowledged
  and the RTS sub-option values are fixed (PR #3).
- BOOTROM-CATCH, a vendor sub-option (50) for the HiSilicon boot ROM, with a BLIND mode,
  diagnostic counters, an optional `min_markers` threshold and the first HEAD frame
  pipelined (PR #4–#7).
- `vectis-cli.c`: an RFC 2217 client with a direct serial mode, break signal, `-n`, reset
  logs, and security and robustness fixes.
- README: ser2net YAML example.

## 1.2.0 — 2026-05-01

- RFC 2217 (Telnet COM Port Control Option) on the TCP listener (PR #1).

## 1.1.1 — 2026-04-30

- Ctrl+P over TCP is handled.

## 1.1.0 — 2026-04-30

- First version in the repository: a UART-to-TCP bridge.
