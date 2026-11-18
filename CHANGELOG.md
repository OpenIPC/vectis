# Changelog

The version lives in `PROGRAM_VERSION` in `vectis.c` (`MAJOR.MINOR.PATCH`). Binaries are
published only as the [`latest`](https://github.com/OpenIPC/vectis/releases/latest) release,
rebuilt from every push to `master`; this file is the change history. Versions
up to 1.3.2 are those of the original UART bridge, now `vectis-bootrom.c`; from 1.6.0 on,
`vectis` is the former `vectis-cli`.

## 1.17.0 — 2026-11-19

The interface review of 2026-11-18 (brand brass 1.3.0), and the app no longer needs the
network to reach vectis.

- New option `--web-local @NAME`: the `-w` page, API and terminal also on an abstract unix
  socket, for an app on the same device. Only a peer with vectis' own uid is served
  (`SO_PEERCRED`), without the page password; it works without `-w` (#111).
- The `-w` page: the title is "Vectis" with the version number beside it, the subtitle
  "USB-UART console and RTS / DTR power"; a lamp board (Adapter, Power, Pulse, Client,
  Last; a lamp per row) replaces the two lamps and the state display; the buttons are
  PWR ON, PWR OFF and RESET, three in a row on a phone too; the terminal bar's lamp and
  plate show the UART line (`USB · ONLINE`, `NO ADAPTER`, `NO SERVICE`) (#113).
- Android, main screen: the lamp board (ADAPTER, POWER, VECTIS, CONSOLE, WEB with a lock
  for the page password, CAMEX when the tunnel is on), the version number beside the
  title, buttons two in a row, no **Open web console** button, the check boxes **Start
  on boot** (now off by default) and **Show log** (the log is hidden by default) (#108).
- Android, terminal: CTRL+C, RESET, PWR ON, PWR OFF across the row (no `y` and `n`); the
  plate shows the UART line (`CH340 · 115200`, `NO ADAPTER`, `NO SERVICE`) (#109).
- Android, settings: one brass SETTINGS window with the sections VECTIS (**Allow remote
  access**, baud, ports, the page password last) and CAMEX (no link field: a camex link
  opened on the box still fills it). Remote access off: vectis opens no network port, the
  CONSOLE and WEB rows read OFF (#110).
- Android: the screen, the power buttons and the terminal talk to vectis on
  `--web-local @org.openipc.vectis.web` instead of TCP on 127.0.0.1 (#111).
- Android: when another app holds the web port, vectis takes the next free one; the WEB
  row, the log and the camex tunnel use the port in use (#112).

## 1.16.2 — 2026-11-18

The `-w` page is one panel (#105):

- The power console and the UART terminal were two panels of different widths; now one
  panel, about 1000 px wide, with the lamps and the readout on the left and the buttons on
  the right (one column on a narrow screen).
- The terminal is a section at the bottom of the panel, closed by default; a click on its
  "UART terminal" bar opens and closes it. The WebSocket opens on the first opening and
  stays, so output collects while the section is closed, and a lamp on the bar shows that
  new output arrived. A page that never opens the terminal takes no terminal slot.
- The browser remembers whether the section was open (`localStorage`).

## 1.16.1 — 2026-11-18

Android fixes:

- Stop, then Start a few seconds later could leave vectis not running ("cannot open the
  socket @org.openipc.vectis.usb: Address already in use"): the old socket's thread is
  woken before it is closed, so it lets go of the name, and a name still taken is tried
  again every second, up to 10 times (#101).
- A camex link is used once: reopening the app from recents no longer opens the settings
  filled from the link that started it (review of #102).

## 1.16.0 — 2026-11-18

The Android app gets remote access through a camex server, built in as in Decoder and
Microsnander (#97).

- camex 4.0.1 as the submodule `bundles/camex`, its tunnel library as the gradle
  project `:camex-tunnel`; CI checks out submodules.
- Settings are in two sections, Serial and Network. Network takes the tunnel: on or
  off, server, client name, UDP (default) or TCP, the server's public key and psk, or a
  camex connect link pasted there or opened with the app (`camex://connect?…`,
  `https://openipc.github.io/camex/connect/#…`), which also turns the tunnel on. The
  web page password stays the last field.
- The tunnel tells the server the web port (`--web-port`, 8085 by default), so nginx
  on the server opens the page by the client name; the page, its terminal and the
  password work through it.
- VPN consent once (the system dialog after Save, a button on the screen, or `adb
  shell appops set org.openipc.vectis ACTIVATE_VPN allow`); without it the app says so
  and tries again every minute. The tunnel runs while vectis does and reconnects by
  itself; the screen (TUNNEL) and the notification show its state and address.
- The README shows the camex client next to vectis on a router or Linux host.
- `.github/check-version.sh` counts `android/`, `bundles/` and `.gitmodules` as code.

## 1.15.1 — 2026-11-18

Android terminal fixes from the review of #99:

- ST (U+009C) ends an OSC or DCS string, and CAN or SUB cancels an escape sequence;
  before, the text after them stayed hidden.
- Leaving the terminal screen right after Enter no longer drops the command: frames
  already accepted are written before the connection closes.

## 1.15.0 — 2026-11-17

The Android app gets the web terminal and password of 1.14.0 (#96).

- **Terminal** screen in the app: the UART on the box's own screen as plain text (colours
  dropped, CR rewrites the line), an input line with Enter, keys for Ctrl+C, y, n, Reset,
  Power on and Power off, usable with a TV remote. A native screen rather than a WebView,
  which may be too old for xterm.js on Android 5-7. It is one more client of
  `/ws/terminal`: the history and the input are shared with the browsers.
- **Web page password** in Settings (the last field, empty by default: open). Passed to
  vectis in `VECTIS_WEB_PASSWORD`, never on the command line; the app's own `/status` and
  terminal requests use it. The screen and the notification say whether the page is
  protected.
- The web port is **8085** by default (was 2081), as in Decoder and Microsnander and as camex
  servers expect; a port saved before is kept.

## 1.14.0 — 2026-11-17

The `-w` page gets a UART terminal and an optional password (#95).

- The page shows a terminal on the same UART under the power panel: xterm.js 5.5.0 built into
  the binary (gzipped, about 70 KB; the browser needs no internet), on the WebSocket
  `/ws/terminal`. On connect it gets the last 64 KB of device output, then everything live.
  Everyone with the page open may type, as with `screen -x`; the bytes go to the device as
  they are (no session keys: reset with the button). Up to 8 terminals; one that stops
  reading is dropped and never holds up the device, one that vanished is closed after 30 s
  without an answer to a ping. The page opens it with a per-process token from `/status`
  (works behind a proxy such as camex's nginx); without the token a browser handshake whose
  `Origin` is another site is refused, so no other page can drive the board through the
  operator's browser. Works in every mode (`-u`, `-U`, `-h/-p`) and on Android, which runs
  the same binary. `/status` gains `terminals` and `terminal_token`.
- `-W PASSWORD` or `VECTIS_WEB_PASSWORD` (`-W` wins) closes the whole `-w` port (page,
  `/status`, `/power/*`, terminal) behind HTTP Basic with any user name; unset or empty, the
  page stays open as before. `-l` has no password.
- `make WEBTERM=0` builds vectis without the terminal (no `/ws/terminal`, no xterm.js); CI
  builds and tests it, and checks that `webterm_assets.h` matches `web/xterm/`.

## 1.13.1 — 2026-11-15

- Android: `-PvectisAbis` must be a non-empty list of the ABIs `android/build-native.sh` builds
  (arm64-v8a, armeabi-v7a, x86_64), otherwise the build stops with that list (#91); an empty
  entry (`arm64-v8a,,x86_64`) is an error too. An empty value used to pack every ABI of the NDK
  into the APK; the default `arm64-v8a,armeabi-v7a` is unchanged. `android/test-abis.sh` (run by
  CI) checks it.

## 1.13.0 — 2026-11-15

The Android app is released: `vectis-android.apk` in `latest` (#89).

- The app (#87, from `android/`) runs vectis on an Android TV box or phone with a USB host,
  without root (Android 5.0 and later, 32- and 64-bit ARM). A foreground service owns the
  CH340/CH341 adapter, hands it to vectis with `-U fd:@NAME`, and serves the console and
  the web power console on TCP ports 2021 and 2081. It starts on boot and keeps the ports
  up across unplugs. The status screen shows the adapter, power and addresses.
- CI builds a debug APK with unit tests and lint on every run. On `master` it builds the
  release APK, signed with the project's key and checked against its certificate. The
  APK is published with the Linux binaries and posted to Telegram with them.

## 1.12.0 — 2026-11-15

`-U fd:@NAME` takes an open USB-UART from an Android app (#85).

- On Android only root can open `/dev/bus/usb`; an app gets the adapter from
  `UsbManager` as an open usbfs descriptor. vectis now receives that descriptor over
  the abstract unix socket `@NAME` (`SCM_RIGHTS`), reads VID:PID from the device
  descriptor and drives the chip as with a USB path. The socket's owner must run
  under the same uid; every re-open after a device loss is a new connection.
- Checked on an Android 10 TV box (32-bit ARM, no root) with a CH340 loopback: raw
  and RFC 2217 data from 9600 to 921600 baud, and RTS/DTR from the web console.
- `/status` shows such a target as `fd:@NAME`.

## 1.11.2 — 2026-11-14

The web console (`-w`) follows the shared brass palette (#83).

- The footer takes its colour from `--foot` (`#a08a69`), which raises its contrast at
  the top edge of the background from 3.53:1 to 4.04:1.
- The tab icon lamp has a thin `#6e5320` outline, as on the other brass pages.

## 1.11.1 — 2026-11-12

The board is now called OpenIPC Vectis 38x38 (#81).

- The web console (`-w`) subtitle names the board as OpenIPC Vectis 38x38.
- The READMEs describe both variants, CH340E (power switched by RTS) and CH340K (by
  DTR), with the ratings (up to 15 V and 1 A switched), the connectors, the X4 debug
  header pinout and top views of both boards.

## 1.11.0 — 2026-11-09

The web console (`-w`) gets a lamp tab icon and respects reduced motion (#79).

- The browser tab shows a round lamp in the colour of the state: green with power
  enabled, grey with power disabled, amber while a reset runs, red with no link. A
  background tab may update it less often, as browsers slow down its timers.
- With `prefers-reduced-motion: reduce` the offline lamp no longer blinks, and the Reset
  indicator and gear stand still; the colour and the state display still show the state.

## 1.10.0 — 2026-11-08

The web console (`-w`) loses the arrow gauge and shows more of the state (#76, #77).

- The arrow gauge is gone: it had only two positions, and the lamp, the state display and
  the lit button already show the power state. The state display takes the full width.
- The browser tab title follows the state: `Power enabled · Vectis`, `Power disabled · Vectis`,
  `Resetting · Vectis`, `Link offline · Vectis`.
- A permanent `LAST` line shows the last power event from any side, such as
  `reset by web, 12 s ago`. It is hidden while there is no event or the link is offline.
- Labels and the footer reach the WCAG AA contrast of 4.5:1.
- The pressed button no longer greys out while it is disabled, so the Reset glow and gear
  stay visible during the pulse.

## 1.9.1 — 2026-11-04

`-u` no longer reconnects to another adapter (#74), and a failed publish no longer takes
`latest` away (#73).

- `-u`: a plain device path is tied to the USB adapter of the first open. After a
  re-enumeration the kernel may give `/dev/ttyUSB0` to another adapter; vectis opened it,
  raising RTS and DTR there, while the adapter powering the board stayed closed. Now it
  stays offline, says so once on stderr and in syslog, and reconnects when the adapter is
  back. Adapters are compared by VID:PID and serial number, or by VID:PID and USB port
  when they have no serial number. A symlink (`by-id`, `by-path`, udev) is not checked:
  it already follows the device. Ports that are not on USB have no identity.
- Releases: `latest` is updated in place, the title last, and is no longer deleted first.
  A re-run of a failed publish finishes the job and posts a new version to Telegram once.

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
