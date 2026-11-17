#!/usr/bin/env python3
#
# Copyright (c) OpenIPC  https://openipc.org  MIT License
#
# test_vectis.py — regression tests for vectis without hardware.
#
# A pty pair stands in for the serial device: vectis opens the slave
# through a symlink (so a "replug" can point it at a new pty), the test
# plays the target on the master.  Covered: the -l server in raw and
# Telnet/RFC 2217 mode (LF->CR, 0xFF escaping, SET-BAUDRATE, SET-CONTROL,
# SIGNATURE, BRK), the -w web API, the web terminal (WebSocket, history,
# several typists, a stalled browser) and the web password, the RFC 2217
# client (-h/-p) against
# the -l server, stdin EOF and device loss without a busy loop, the -u
# check of the USB adapter on reconnect (a fake sysfs), the -U command
# line of the USB builds, and -U fd:@NAME against a fake app socket.
#
# Usage: tests/test_vectis.py [DIR]   (DIR holds vectis and vectis-*; default .)
# Python 3 standard library only; Linux.
#

import base64
import gzip
import hashlib
import json
import os
import re
import pty
import select
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import tty
import urllib.error
import urllib.request

BIN_DIR = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else ".")
VECTIS = os.path.join(BIN_DIR, "vectis")

IAC, DONT, DO, WONT, WILL, SB, SE, BRK = 255, 254, 253, 252, 251, 250, 240, 243
COMPORT = 44

failures = []


def check(cond, what):
    print(("PASS " if cond else "FAIL ") + what, flush=True)
    if not cond:
        failures.append(what)


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class FakeDevice:
    """A pty whose slave is reached through a stable symlink."""

    def __init__(self, link):
        self.link = link
        self.master = None
        self.plug()

    def plug(self):
        self.master, slave = pty.openpty()
        tty.setraw(self.master)
        name = os.ttyname(slave)
        os.close(slave)
        tmp = self.link + ".new"
        if os.path.lexists(tmp):
            os.unlink(tmp)
        os.symlink(name, tmp)
        os.replace(tmp, self.link)

    def unplug(self):
        os.close(self.master)
        self.master = None
        os.unlink(self.link)

    def read(self, timeout=1.0, until=None):
        data = b""
        end = time.time() + timeout
        while time.time() < end:
            r, _, _ = select.select([self.master], [], [], 0.05)
            if r:
                try:
                    chunk = os.read(self.master, 4096)
                except OSError:
                    # EIO until vectis opens the slave side: keep waiting.
                    time.sleep(0.05)
                    continue
                data += chunk
                if until is not None and until in data:
                    break
        return data

    def write(self, data):
        os.write(self.master, data)


def recv_until(sock, until, timeout=2.0):
    data = b""
    end = time.time() + timeout
    sock.settimeout(0.05)
    while time.time() < end:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            continue
        if not chunk:
            break
        data += chunk
        if until is not None and until in data:
            break
    return data


def telnet_subneg(data):
    """COM-PORT replies (sub-option, payload) found in a Telnet stream."""
    out = []
    i = 0
    while True:
        i = data.find(bytes([IAC, SB, COMPORT]), i)
        if i < 0:
            return out
        j = data.find(bytes([IAC, SE]), i)
        if j < 0:
            return out
        out.append((data[i + 3], data[i + 4:j]))
        i = j + 2


def http(port, path):
    with urllib.request.urlopen("http://127.0.0.1:%d%s" % (port, path), timeout=5) as r:
        return json.loads(r.read().decode())


class Ws:
    """A minimal WebSocket client (RFC 6455) for /ws/terminal."""

    def __init__(self, port, auth=None, read=True):
        self.sock = socket.create_connection(("127.0.0.1", port), 5)
        key = base64.b64encode(os.urandom(16)).decode()
        req = ("GET /ws/terminal HTTP/1.1\r\nHost: 127.0.0.1\r\n"
               "Upgrade: websocket\r\nConnection: Upgrade\r\n"
               "Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n" % key)
        if auth is not None:
            req += "Authorization: Basic %s\r\n" % base64.b64encode(auth.encode()).decode()
        self.sock.sendall((req + "\r\n").encode())
        head = b""
        while b"\r\n\r\n" not in head:
            chunk = self.sock.recv(1)
            if not chunk:
                break
            head += chunk
        self.head = head.decode(errors="replace")
        self.status = int(self.head.split(" ")[1]) if self.head.startswith("HTTP/") else 0
        want = base64.b64encode(hashlib.sha1(
            (key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
        self.accept_ok = ("Sec-WebSocket-Accept: %s" % want) in self.head
        self.buf = b""

    def send(self, payload, opcode=2, fin=True):
        mask = os.urandom(4)
        n = len(payload)
        h = bytes([(0x80 if fin else 0) | opcode])
        if n < 126:
            h += bytes([0x80 | n])
        elif n < 65536:
            h += bytes([0x80 | 126]) + struct.pack(">H", n)
        else:
            h += bytes([0x80 | 127]) + struct.pack(">Q", n)
        body = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
        self.sock.sendall(h + mask + body)

    def frames(self, timeout=2.0, until=None):
        """(opcode, payload) of the frames that arrive within timeout."""
        out = []
        data = b""
        end = time.time() + timeout
        self.sock.settimeout(0.05)
        while time.time() < end:
            while True:
                f = self._parse()
                if f is None:
                    break
                out.append(f)
                if f[0] == 2:
                    data += f[1]
                if until is not None and (until in data or f[0] == 8):
                    return out
            try:
                chunk = self.sock.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                break
            self.buf += chunk
        return out

    def data(self, timeout=2.0, until=None):
        return b"".join(p for op, p in self.frames(timeout, until) if op == 2)

    def _parse(self):
        b = self.buf
        if len(b) < 2:
            return None
        n, hl = b[1] & 0x7F, 2
        if n == 126:
            if len(b) < 4:
                return None
            n, hl = struct.unpack(">H", b[2:4])[0], 4
        elif n == 127:
            if len(b) < 10:
                return None
            n, hl = struct.unpack(">Q", b[2:10])[0], 10
        if len(b) < hl + n:
            return None
        self.buf = b[hl + n:]
        return (b[0] & 0x0F, b[hl:hl + n])

    def close(self):
        self.sock.close()


def http_raw(port, path, auth=None):
    """(status, headers, body) of a GET, without raising on 4xx."""
    req = urllib.request.Request("http://127.0.0.1:%d%s" % (port, path))
    if auth is not None:
        req.add_header("Authorization", "Basic " + base64.b64encode(auth.encode()).decode())
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, r.headers, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.headers, e.read()


def cpu_seconds(pid):
    with open("/proc/%d/stat" % pid) as f:
        fields = f.read().rsplit(")", 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")


def cpu_share(pid, seconds):
    a = cpu_seconds(pid)
    time.sleep(seconds)
    return (cpu_seconds(pid) - a) / seconds


def start(args, **kw):
    return subprocess.Popen(args, stdin=kw.pop("stdin", subprocess.DEVNULL),
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, **kw)


def stop(p):
    if p.poll() is None:
        p.send_signal(signal.SIGTERM)
        try:
            p.wait(5)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()
    return p.stdout.read().decode(errors="replace")


def wait_listen(port, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        try:
            socket.create_connection(("127.0.0.1", port), 0.2).close()
            return True
        except OSError:
            time.sleep(0.05)
    return False


def wait_status(wport, key, value, timeout=10.0):
    end = time.time() + timeout
    st = {}
    while time.time() < end:
        try:
            st = http(wport, "/status")
            if st.get(key) == value:
                return st
        except OSError:
            pass
        time.sleep(0.2)
    return st


def test_server(tmp):
    print("== -l server and -w web (-u on a pty)")
    dev = FakeDevice(os.path.join(tmp, "dev"))
    lport, wport = free_port(), free_port()
    p = start([VECTIS, "-u", dev.link, "-l", "127.0.0.1:%d" % lport,
               "-w", "127.0.0.1:%d" % wport, "-t", "300", "-n"])
    try:
        check(wait_listen(lport) and wait_listen(wport), "listeners come up")
        st = wait_status(wport, "link", "online")
        check(st.get("link") == "online" and st.get("mode") == "serial", "/status: online, serial")

        # The page: every element its script touches exists, or render() throws
        # and the page stops updating.
        with urllib.request.urlopen("http://127.0.0.1:%d/" % wport, timeout=5) as r:
            page = r.read().decode()
        ids = set(re.findall(r"(?:\$|bulb)\('([\w-]+)'", page))
        missing = sorted(i for i in ids if "id='%s'" % i not in page)
        check(ids and not missing, "web page: script ids exist (missing %r)" % missing)
        check("needle" not in page and "gauge" not in page, "web page: no arrow gauge")
        check("document.title=" in page, "web page: the tab title follows the state")
        check("LAST" in page and ".btn:disabled.engaged{filter:none}" in page,
              "web page: LAST line, the engaged button stays lit")
        check("rel='icon'" in page and "LAMP[" in page, "web page: lamp tab icon")
        check(page.count("stroke=%27%236e5320%27") == 2 and "stroke-width" not in page,
              "web page: tab icon stroke #6e5320, width 1 (brass)")
        check("--foot:#a08a69" in page and "color:var(--foot)" in page and "#958061" not in page,
              "web page: footer uses --foot #a08a69 (brass)")
        check("prefers-reduced-motion:reduce" in page, "web page: reduced motion respected")
        check("OpenIPC Vectis 38x38" in page and "UART 38x38" not in page,
              "web page: names the OpenIPC Vectis 38x38 board")
        check('"' not in page, "web page: single quotes only")
        check("fetch('/" not in page and "fetch(path" not in page and "location.origin" in page,
              "web page: fetch() with absolute URLs (a page opened as http://user:pass@host/)")

        # Raw mode: LF and CRLF become CR on the way to the device.
        c = socket.create_connection(("127.0.0.1", lport))
        time.sleep(0.2)
        c.sendall(b"ls\nab\r\n")
        got = dev.read(1.0, until=b"ab\r")
        check(got == b"ls\rab\r", "raw: LF/CRLF -> CR (got %r)" % got)
        dev.write(b"hello\n\xff")
        got = recv_until(c, b"\xff")
        check(got == b"hello\n\xff", "raw: device bytes pass unchanged (got %r)" % got)
        c.close()
        time.sleep(0.3)

        # Telnet / RFC 2217 mode.
        c = socket.create_connection(("127.0.0.1", lport))
        c.sendall(bytes([IAC, WILL, 0, IAC, DO, 0, IAC, WILL, 3, IAC, DO, 3, IAC, WILL, COMPORT]))
        neg = recv_until(c, None, 0.5)
        check(bytes([IAC, DO, COMPORT]) in neg or bytes([IAC, WILL, COMPORT]) in neg,
              "telnet: COM-PORT-OPTION negotiated")

        c.sendall(bytes([IAC, IAC, 0x41, 0x0A]))
        got = dev.read(1.0, until=b"\n")
        check(got == b"\xffA\n", "telnet: IAC IAC -> 0xFF, binary-safe LF (got %r)" % got)
        dev.write(b"\xff\x01")
        got = recv_until(c, b"\x01")
        check(got == b"\xff\xff\x01", "telnet: 0xFF from the device is doubled (got %r)" % got)

        def ask(sub, payload):
            c.sendall(bytes([IAC, SB, COMPORT, sub]) + payload + bytes([IAC, SE]))
            r = telnet_subneg(recv_until(c, bytes([IAC, SE])))
            return r[-1] if r else (None, b"")

        sub, val = ask(0, b"")
        check(sub == 100 and val.startswith(b"vectis "), "SIGNATURE: %r" % val)
        sub, val = ask(1, struct.pack(">I", 9600))
        check(sub == 101 and val == struct.pack(">I", 9600),
              "SET-BAUDRATE 9600: reply %r, no trailing bytes" % val)
        sub, val = ask(1, struct.pack(">I", 0))
        check(val == struct.pack(">I", 9600), "SET-BAUDRATE 0 reports 9600")
        # A pty keeps CS8 and no parity whatever is asked: the reply must
        # report the port's real setting, as RFC 2217 wants.
        sub, val = ask(2, bytes([7]))
        check(sub == 102 and val in (bytes([7]), bytes([8])), "SET-DATASIZE 7 -> %r" % val)
        sub, val = ask(2, bytes([0]))
        check(sub == 102 and val in (bytes([7]), bytes([8])), "SET-DATASIZE query -> %r" % val)
        sub, val = ask(3, bytes([3]))
        check(sub == 103 and val in (bytes([3]), bytes([1])), "SET-PARITY even -> %r" % val)
        ask(2, bytes([8]))
        ask(3, bytes([1]))
        ask(1, struct.pack(">I", 115200))
        sub, val = ask(5, bytes([0]))
        check(sub == 105 and val == bytes([1]), "SET-CONTROL flow query -> no flow")
        sub, val = ask(5, bytes([9]))
        check(sub == 105 and val == bytes([9]), "SET-CONTROL DTR off acknowledged")
        sub, val = ask(5, bytes([8]))
        check(sub == 105 and val == bytes([8]), "SET-CONTROL DTR on acknowledged")
        sub, val = ask(5, bytes([12]))
        check(sub == 105 and val == bytes([12]), "SET-CONTROL RTS off acknowledged")
        sub, val = ask(5, bytes([11]))
        check(sub == 105 and val == bytes([11]), "SET-CONTROL RTS on acknowledged")
        sub, val = ask(5, bytes([10]))
        check(sub == 105 and val in (bytes([11]), bytes([12])), "SET-CONTROL RTS query -> %r" % val)
        sub, val = ask(5, bytes([7]))
        check(sub == 105 and val in (bytes([8]), bytes([9])), "SET-CONTROL DTR query -> %r" % val)
        sub, val = ask(12, bytes([1]))
        check(sub == 112 and val == bytes([1]), "PURGE-DATA acknowledged")
        c.sendall(bytes([IAC, BRK]) + b"x")
        got = dev.read(1.0, until=b"x")
        check(got.endswith(b"x"), "BRK does not eat the next byte")
        c.close()
        time.sleep(0.3)

        # Web power API.
        st = http(wport, "/power/disable")
        check(st["power"] == "off", "/power/disable -> off")
        st = http(wport, "/power/enable")
        check(st["power"] == "on", "/power/enable -> on")
        t0 = time.time()
        st = http(wport, "/power/reset")
        took = time.time() - t0
        check(st["power"] == "on" and st["event"]["kind"] == "reset" and took >= 0.25,
              "/power/reset answers after the pulse (%.2f s, %s)" % (took, st["event"]["kind"]))

        # Device loss: no busy loop, one log line, back when it returns.
        dev.unplug()
        st = wait_status(wport, "link", "offline")
        check(st.get("link") == "offline", "unplugged: /status offline")
        share = cpu_share(p.pid, 2.0)
        check(share < 0.05, "unplugged: idle CPU (%.1f%%)" % (share * 100))
        dev.plug()
        st = wait_status(wport, "link", "online", 12.0)
        check(st.get("link") == "online", "replugged: back online")
        share = cpu_share(p.pid, 1.0)
        check(share < 0.05, "online: idle CPU (%.1f%%)" % (share * 100))
    finally:
        log = stop(p)
    check(log.count("Serial device error/hangup") == 1 and log.count("Reconnecting") == 1,
          "the outage is logged once")


def test_client(tmp):
    print("== RFC 2217 client (-h/-p) against the -l server")
    dev = FakeDevice(os.path.join(tmp, "dev2"))
    lport = free_port()
    srv = start([VECTIS, "-u", dev.link, "-l", "127.0.0.1:%d" % lport, "-n"])
    cli = None
    try:
        check(wait_listen(lport), "server up")
        cli = start([VECTIS, "-h", "127.0.0.1", "-p", str(lport), "-n"], stdin=subprocess.PIPE)
        time.sleep(0.5)
        cli.stdin.write(b"x\xffy\n")
        cli.stdin.flush()
        got = dev.read(1.5, until=b"\n")
        check(got == b"x\xffy\n", "client -> server -> device, 0xFF intact (got %r)" % got)
        dev.write(b"ok\xff\n")
        time.sleep(0.5)
        cli.stdin.close()   # stdin EOF: keeps forwarding, no busy loop
        share = cpu_share(cli.pid, 1.5)
        check(share < 0.05, "client after stdin EOF: idle CPU (%.1f%%)" % (share * 100))
        out = stop(cli)
        cli = None
        check("ok" in out, "device output reaches the client")

        # The web terminal of an RFC 2217 client: 0xFF escaped on the way.
        wport = free_port()
        cli = start([VECTIS, "-h", "127.0.0.1", "-p", str(lport), "-n",
                     "-w", "127.0.0.1:%d" % wport])
        st = wait_status(wport, "link", "online")
        if st.get("terminals") is not None:
            ws = Ws(wport)
            ws.data(0.5)
            ws.send(b"t\xffu\r")
            got = dev.read(1.5, until=b"\r")
            check(got == b"t\xffu\r", "client's web terminal -> device, 0xFF intact (%r)" % got)
            dev.write(b"v\xffw\n")
            got = ws.data(2.0, until=b"w\n")
            check(got == b"v\xffw\n", "device -> client's web terminal, 0xFF once (%r)" % got)
            ws.close()
    finally:
        if cli is not None:
            stop(cli)
        stop(srv)


def test_stdin_eof(tmp):
    print("== -u with stdin at EOF")
    dev = FakeDevice(os.path.join(tmp, "dev3"))
    p = start([VECTIS, "-u", dev.link, "-n"], stdin=subprocess.PIPE)
    try:
        p.stdin.write(b"cmd\n")
        p.stdin.close()
        got = dev.read(1.0, until=b"\n")
        check(got == b"cmd\n", "piped stdin reaches the device (got %r)" % got)
        share = cpu_share(p.pid, 1.5)
        check(share < 0.05, "stdin EOF: idle CPU (%.1f%%)" % (share * 100))
        dev.write(b"reply\n")
        time.sleep(0.3)
    finally:
        out = stop(p)
    check("reply" in out, "device output still forwarded after stdin EOF")


def start_quiet(args, logpath, **kw):
    """vectis with stdout thrown away (large device output must not fill a
    pipe nobody reads) and stderr in a file."""
    log = open(logpath, "w+b")
    return subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                            stderr=log, **kw), log


def stop_quiet(p, log):
    if p.poll() is None:
        p.send_signal(signal.SIGTERM)
        try:
            p.wait(5)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()
    log.seek(0)
    text = log.read().decode(errors="replace")
    log.close()
    return text


def test_web_terminal(tmp):
    print("== web terminal (/ws/terminal)")
    dev = FakeDevice(os.path.join(tmp, "dev-wt"))
    wport = free_port()
    p, log = start_quiet([VECTIS, "-u", dev.link, "-w", "127.0.0.1:%d" % wport, "-n"],
                         os.path.join(tmp, "wt.log"))
    text = ""
    try:
        check(wait_listen(wport), "web up")
        st = wait_status(wport, "link", "online")
        page = http_raw(wport, "/")[2].decode()
        if st.get("terminals", 0) is None:
            # make WEBTERM=0: nothing of the terminal is there.
            ws = Ws(wport)
            check(ws.status == 404, "WEBTERM=0: no /ws/terminal (%d)" % ws.status)
            ws.close()
            check(http_raw(wport, "/xterm-5.5.0.js")[0] == 404, "WEBTERM=0: no xterm.js")
            check("xterm" not in page and "terminal" not in page.lower(),
                  "WEBTERM=0: the page has no terminal")
            return
        check(st.get("terminals") == 0, "/status: terminals 0")
        check("/xterm-5.5.0.js" in page and "/xterm-5.5.0.css" in page and "/ws/terminal" in page,
              "web page: loads the built-in xterm.js, opens /ws/terminal")
        check("cdn" not in page.lower(), "web page: nothing from a CDN")
        for asset, head, ctype in (("/xterm-5.5.0.js", b"!function", "text/javascript"),
                                   ("/xterm-5.5.0.css", b"/**", "text/css")):
            code, hdr, body = http_raw(wport, asset)
            ok = code == 200 and hdr.get("Content-Encoding") == "gzip" and \
                (hdr.get("Content-Type") or "").startswith(ctype)
            try:
                plain = gzip.decompress(body)
            except OSError:
                plain = b""
            check(ok and plain.startswith(head) and len(plain) > 1000,
                  "%s: gzip, %s, %d bytes unpacked" % (asset, ctype, len(plain)))
        js = gzip.decompress(http_raw(wport, "/xterm-5.5.0.js")[2])
        check(b"FitAddon" in js and b"sourceMappingURL" not in js,
              "xterm.js carries the fit addon, no source map link")

        dev.write(b"U-Boot 2016.11\r\n")
        time.sleep(0.3)
        a = Ws(wport)
        check(a.status == 101 and a.accept_ok, "handshake: 101 with the right Sec-WebSocket-Accept")
        got = a.data(2.0, until=b"2016.11\r\n")
        check(got == b"U-Boot 2016.11\r\n", "history: output from before the page opened (%r)" % got)
        check(http(wport, "/status").get("terminals") == 1, "/status: terminals 1")
        dev.write(b"login: ")
        got = a.data(2.0, until=b"login: ")
        check(got == b"login: ", "live output reaches the terminal (%r)" % got)
        a.send(b"root\r")
        got = dev.read(1.0, until=b"\r")
        check(got == b"root\r", "typed bytes reach the device as they are (%r)" % got)
        seq = (http(wport, "/status").get("event") or {}).get("seq")
        a.send(b"\x10\x02\xff\n")
        got = dev.read(1.0, until=b"\n")
        check(got == b"\x10\x02\xff\n", "Ctrl+P, Ctrl+B, 0xFF, LF pass as bytes (%r)" % got)
        check((http(wport, "/status").get("event") or {}).get("seq") == seq,
              "Ctrl+P from the terminal is no reset")

        b = Ws(wport)
        b.data(1.0)                                # its history
        dev.write(b"both\n")
        check(a.data(2.0, until=b"both\n") == b"both\n" and b.data(2.0, until=b"both\n") == b"both\n",
              "two terminals: both see the output")
        b.send(b"from-b\r")
        got = dev.read(1.0, until=b"\r")
        check(got == b"from-b\r", "two terminals: the second one types too (%r)" % got)
        a.send(b'{"resize":{"cols":120,"rows":40}}', opcode=1)
        got = dev.read(0.5)
        check(got == b"", "text frames (resize) do not reach the device (%r)" % got)
        a.send(b"fr", fin=False)
        a.send(b"ag\r", opcode=0)
        got = dev.read(1.0, until=b"\r")
        check(got == b"frag\r", "fragmented binary frame reassembled (%r)" % got)
        a.send(b"pp", opcode=9)
        fr = a.frames(1.0, until=None)
        check((10, b"pp") in fr, "ping answered with pong")
        b.send(struct.pack(">H", 1000), opcode=8)
        fr = b.frames(1.0, until=b"")
        check(any(op == 8 for op, _ in fr), "close answered with close")
        b.close()
        a.close()
        time.sleep(0.3)
        check(http(wport, "/status").get("terminals") == 0, "/status: terminals back to 0")

        # The history keeps the last 64 KB.
        big = bytes((i * 131 + i // 256) & 0xFF for i in range(100000))
        for i in range(0, len(big), 4096):
            dev.write(big[i:i + 4096])
        time.sleep(0.5)
        c = Ws(wport)
        got = c.data(3.0, until=big[-64:])
        check(len(got) == 65536 and got == big[-65536:],
              "history: the last 65536 bytes, in order (%d bytes)" % len(got))
        c.close()

        # A terminal that does not read is dropped; the rest is not held up.
        stalled = socket.socket()
        stalled.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
        stalled.connect(("127.0.0.1", wport))
        key = base64.b64encode(os.urandom(16)).decode()
        stalled.sendall(("GET /ws/terminal HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\n"
                         "Connection: Upgrade\r\nSec-WebSocket-Key: %s\r\n"
                         "Sec-WebSocket-Version: 13\r\n\r\n" % key).encode())
        time.sleep(0.3)
        chunk = b"x" * 4095 + b"\n"
        t0 = time.time()
        for _ in range(512):                       # 2 MB
            dev.write(chunk)
        took = time.time() - t0
        end = time.time() + 5
        while time.time() < end and http(wport, "/status").get("terminals") != 0:
            time.sleep(0.1)
        check(http(wport, "/status").get("terminals") == 0,
              "a stalled terminal is dropped (2 MB in %.2f s)" % took)
        stalled.close()
        d = Ws(wport)
        d.data(1.0)
        dev.write(b"still here\n")
        check(d.data(2.0, until=b"here\n").endswith(b"still here\n"), "a new terminal works after that")
        share = cpu_share(p.pid, 1.0)
        check(share < 0.05, "terminal open, idle CPU (%.1f%%)" % (share * 100))
        d.close()
    finally:
        text = stop_quiet(p, log)
    check("terminal opened" in text and "too slow, dropped" in text,
          "the log names opened and dropped terminals")


def test_web_password(tmp):
    print("== web password (-W, VECTIS_WEB_PASSWORD)")
    dev = FakeDevice(os.path.join(tmp, "dev-pw"))
    env = dict(os.environ, VECTIS_WEB_PASSWORD="env:pw")

    def run(extra, env):
        wport = free_port()
        p, log = start_quiet([VECTIS, "-u", dev.link, "-w", "127.0.0.1:%d" % wport, "-n"] + extra,
                             os.path.join(tmp, "pw.log"), env=env)
        check(wait_listen(wport), "web up")
        return p, log, wport

    p, log, wport = run([], env)
    try:
        code, hdr, _ = http_raw(wport, "/")
        check(code == 401 and 'Basic realm="vectis"' in (hdr.get("WWW-Authenticate") or ""),
              "no password: 401 with WWW-Authenticate Basic")
        check(http_raw(wport, "/status")[0] == 401 and http_raw(wport, "/power/reset")[0] == 401,
              "no password: /status and /power/* closed too")
        check(http_raw(wport, "/", "admin:wrong")[0] == 401, "wrong password: 401")
        check(http_raw(wport, "/", "admin:env:pw")[0] == 200, "VECTIS_WEB_PASSWORD, user admin: 200")
        check(http_raw(wport, "/status", ":env:pw")[0] == 200, "empty user name: 200")
        check(http_raw(wport, "/", "x:env:p")[0] == 401 and http_raw(wport, "/", "x:env:pw2")[0] == 401,
              "a prefix or a longer password is refused")
        if json.loads(http_raw(wport, "/status", ":env:pw")[2]).get("terminals") is not None:
            ws = Ws(wport)
            check(ws.status == 401, "terminal without the password: 401 (%d)" % ws.status)
            ws.close()
            ws = Ws(wport, auth="u:env:pw")
            check(ws.status == 101, "terminal with the password: 101 (%d)" % ws.status)
            ws.close()
    finally:
        text = stop_quiet(p, log)
    check("protected by a password" in text and "env:pw" not in text,
          "the log says the page is protected, not the password")

    p, log, wport = run(["-W", "arg"], env)
    try:
        check(http_raw(wport, "/", ":env:pw")[0] == 401 and http_raw(wport, "/", ":arg")[0] == 200,
              "-W wins over VECTIS_WEB_PASSWORD")
    finally:
        stop_quiet(p, log)

    p, log, wport = run([], dict(os.environ, VECTIS_WEB_PASSWORD=""))
    try:
        check(http_raw(wport, "/")[0] == 200, "empty VECTIS_WEB_PASSWORD: open")
    finally:
        text = stop_quiet(p, log)
    check("open, no password" in text, "the log says the page is open")

    r = subprocess.run([VECTIS, "-u", dev.link, "-W", "x"], stdin=subprocess.DEVNULL,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=5)
    check(r.returncode == 1 and b"give -w too" in r.stdout, "-W without -w: refused")


class FakeSysfs:
    """A sysfs tree that says which USB adapter a pty is (VECTIS_SYSFS_ROOT)."""

    ADAPTERS = {"ch340": ("1a86", "7523", None, "1-2"),
                "ch340-next-port": ("1a86", "7523", None, "1-3"),
                "ftdi": ("0403", "6001", "A50285BI", "1-4"),
                "ftdi-moved": ("0403", "6001", "A50285BI", "1-5")}

    def __init__(self, root):
        self.root = root
        os.makedirs(os.path.join(root, "dev", "char"))
        for vid, pid, serial, port in self.ADAPTERS.values():
            usb = os.path.join(root, "devices", "usb1", port)
            os.makedirs(os.path.join(usb, port + ":1.0", "ttyUSB0", "tty", "ttyUSB0"))
            for name, value in (("idVendor", vid), ("idProduct", pid), ("serial", serial)):
                if value:
                    with open(os.path.join(usb, name), "w") as f:
                        f.write(value + "\n")

    def point(self, path, adapter):
        """Make the character device behind path be this adapter."""
        st = os.stat(path)
        port = self.ADAPTERS[adapter][3]
        link = os.path.join(self.root, "dev", "char",
                            "%d:%d" % (os.major(st.st_rdev), os.minor(st.st_rdev)))
        tmp = link + ".new"
        os.symlink(os.path.join(self.root, "devices", "usb1", port, port + ":1.0",
                                "ttyUSB0", "tty", "ttyUSB0"), tmp)
        os.replace(tmp, link)


def test_identity(tmp):
    sysfs = FakeSysfs(os.path.join(tmp, "sys"))
    env = dict(os.environ, VECTIS_SYSFS_ROOT=sysfs.root)

    def reconnect_on(master, name, adapter):
        """Unplug, let vectis see it, give the same name to adapter."""
        sysfs.point(name, adapter)
        os.close(master)
        check(wait_status(wport, "link", "offline").get("link") == "offline",
              "identity: link offline after the unplug")
        master, slave = pty.openpty()
        tty.setraw(master)
        same = os.ttyname(slave) == name
        os.close(slave)
        check(same, "identity: the new pty has the old name")
        return master

    # A plain path, as /dev/ttyUSB0: the pty number comes back with the next
    # pty, like the name of a re-enumerated adapter.
    for first, other, same_adapter in (("ch340", "ftdi", "ch340"),
                                       ("ch340", "ch340-next-port", "ch340"),
                                       ("ftdi", "ch340", "ftdi-moved")):
        master, slave = pty.openpty()
        tty.setraw(master)
        name = os.ttyname(slave)
        os.close(slave)
        sysfs.point(name, first)
        wport = free_port()
        p = start([VECTIS, "-u", name, "-w", "127.0.0.1:%d" % wport], env=env)
        check(wait_status(wport, "link", "online").get("link") == "online",
              "identity: %s online" % first)
        master = reconnect_on(master, name, other)
        time.sleep(7)   # over two retry periods
        check(http(wport, "/status").get("link") == "offline",
              "identity: %s on the name of %s is not opened" % (other, first))
        sysfs.point(name, same_adapter)
        check(wait_status(wport, "link", "online").get("link") == "online",
              "identity: %s accepted back" % same_adapter)
        out = stop(p)
        os.close(master)
        check("is USB" in out, "identity: the adapter is logged")
        check(out.count("not opening it") == 1, "identity: the mismatch is logged once")

    # A symlink follows the device, so it is not checked.
    dev = FakeDevice(os.path.join(tmp, "by-path-port"))
    sysfs.point(dev.link, "ch340")
    wport = free_port()
    p = start([VECTIS, "-u", dev.link, "-w", "127.0.0.1:%d" % wport], env=env)
    check(wait_status(wport, "link", "online").get("link") == "online",
          "identity: symlink online")
    dev.unplug()
    check(wait_status(wport, "link", "offline").get("link") == "offline",
          "identity: symlink offline after the unplug")
    dev.plug()
    sysfs.point(dev.link, "ftdi")
    check(wait_status(wport, "link", "online").get("link") == "online",
          "identity: a symlink is not checked")
    out = stop(p)
    check("not opening it" not in out, "identity: no mismatch through a symlink")


def test_usb_cli():
    print("== -U command line")
    # vectis has -U unless built with USB=0; --version lists the chips then.
    r = subprocess.run([VECTIS, "--version"], capture_output=True, text=True)
    if "USB chips (-U)" not in r.stdout:
        r = subprocess.run([VECTIS, "-U", "1-1"], capture_output=True, text=True)
        check(r.returncode == 1 and "no USB support" in r.stderr, "vectis (USB=0): -U refused")
        return
    check("ch34x" in r.stdout, "vectis --version lists ch34x among the USB chips")
    b, name = VECTIS, "vectis"
    r = subprocess.run([b, "-U", "nonsense"], capture_output=True, text=True)
    check(r.returncode == 1 and "Invalid -U spec" in r.stderr, "%s: bad -U spec" % name)
    r = subprocess.run([b, "-U", "usb:zz:1"], capture_output=True, text=True)
    check(r.returncode == 1 and "Invalid -U spec" in r.stderr, "%s: bad VID:PID" % name)
    r = subprocess.run([b, "-U", "usb:0000:7523@1-1"], capture_output=True, text=True)
    check(r.returncode == 1 and "Invalid -U spec" in r.stderr, "%s: VID 0000 refused" % name)
    r = subprocess.run([b, "-U", "9-9.9", "-b", "10000"], capture_output=True, text=True)
    check(r.returncode == 1 and "Unsupported baud rate" in r.stderr,
          "%s: unsupported baud refused before opening" % name)
    r = subprocess.run([b, "-U", "1-1", "-u", "/dev/null"], capture_output=True, text=True)
    check(r.returncode == 1, "%s: -U with -u refused" % name)
    r = subprocess.run([b, "-U", "9-9.9"], capture_output=True, text=True,
                       stdin=subprocess.DEVNULL)
    check(r.returncode == 1 and "no USB device at 9-9.9" in r.stderr,
          "%s: absent device reported" % name)
    wport = free_port()
    p = start([b, "-U", "usb:1a86:7523@9-9.9", "-w", "127.0.0.1:%d" % wport])
    try:
        st = wait_status(wport, "mode", "usb")
        check(st.get("target") == "usb:1a86:7523@9-9.9",
              "%s: /status target %r" % (name, st.get("target")))
    finally:
        stop(p)
    wport = free_port()
    p = start([b, "-U", "9-9.9", "-w", "127.0.0.1:%d" % wport])
    try:
        st = wait_status(wport, "mode", "usb")
        check(st.get("mode") == "usb" and st.get("link") == "offline",
              "%s: daemon without the device serves /status offline" % name)
        share = cpu_share(p.pid, 1.0)
        check(share < 0.05, "%s: waiting for the device, idle CPU" % name)
    finally:
        log = stop(p)
    check(log.count("no USB device") == 1, "%s: the outage is logged once" % name)


class FdServer:
    """The app side of -U fd:@NAME: an abstract unix socket that sends one
    fd (SCM_RIGHTS) per connection, or with fd None just a byte.  copies=2
    sends the fd twice in one message; stall=True accepts and sends nothing."""

    def __init__(self, name, fd, copies=1, stall=False):
        self.fd, self.copies, self.stall, self.accepted = fd, copies, stall, 0
        self.held = []
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.bind("\0" + name)
        self.sock.listen(4)
        self.sock.settimeout(0.2)
        self.running = True
        import threading
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def serve(self):
        while self.running:
            try:
                c, _ = self.sock.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self.accepted += 1
            if self.stall:
                self.held.append(c)
                continue
            anc = [] if self.fd is None else \
                [(socket.SOL_SOCKET, socket.SCM_RIGHTS, struct.pack("i" * self.copies,
                                                                    *[self.fd] * self.copies))]
            try:
                c.sendmsg([b"\x01"], anc)
            finally:
                c.close()

    def close(self):
        self.running = False
        self.thread.join()
        self.sock.close()
        for c in self.held:
            c.close()


def device_descriptor(vid, pid):
    # bLength, bDescriptorType, bcdUSB, class/subclass/protocol, bMaxPacketSize0,
    # idVendor, idProduct, bcdDevice, iManufacturer/iProduct/iSerial, bNumConfigurations
    return struct.pack("<BBHBBBBHHHBBBB", 18, 1, 0x0110, 0xff, 0, 0, 8, vid, pid, 0x0264,
                       0, 2, 0, 1)


def test_usb_fd(tmp):
    print("== -U fd:@NAME (an fd from an Android app)")
    r = subprocess.run([VECTIS, "--version"], capture_output=True, text=True)
    if "USB chips (-U)" not in r.stdout:
        return
    for spec in ("fd:", "fd:@", "fd:name", "fd:@" + "n" * 28):
        r = subprocess.run([VECTIS, "-U", spec], capture_output=True, text=True)
        check(r.returncode == 1 and "Invalid -U spec" in r.stderr, "-U %s refused" % spec[:12])
    name = "vectis-test-%d" % os.getpid()
    r = subprocess.run([VECTIS, "-U", "fd:@" + name], capture_output=True, text=True,
                       stdin=subprocess.DEVNULL)
    check(r.returncode == 1 and ("fd:@%s: connect:" % name) in r.stderr,
          "-U fd: without the app: connect error")

    def run_with(fd, expect, what, **kw):
        srv = FdServer(name, fd, **kw)
        t = time.time()
        try:
            r = subprocess.run([VECTIS, "-U", "fd:@" + name], capture_output=True, text=True,
                               stdin=subprocess.DEVNULL, timeout=10)
        finally:
            srv.close()
        check(r.returncode == 1 and expect in r.stderr and srv.accepted == 1,
              "-U fd: %s (%r)" % (what, r.stderr.strip()[-90:]))
        return time.time() - t

    run_with(None, "no fd received", "a connection without an fd")
    took = run_with(None, "timed out", "a stalled app times out", stall=True)
    check(took < 5, "-U fd: the stall ends after about 2 s (%.1f s)" % took)
    files = {}
    for key, data in (("junk", b"not a usbfs fd"), ("other", device_descriptor(0x1234, 0x5678)),
                      ("ch340", device_descriptor(0x1a86, 0x7523) + bytes(9))):
        path = os.path.join(tmp, "usbfs-" + key)
        with open(path, "wb") as f:
            f.write(data)
        files[key] = os.open(path, os.O_RDONLY)
    try:
        run_with(files["junk"], "no USB device descriptor", "a non-usbfs fd refused")
        run_with(files["junk"], "expected one fd, got 2", "two fds in one message refused",
                 copies=2)
        run_with(files["other"], "1234:5678 is not a supported chip", "an unknown chip refused")
        # The descriptor names a CH340, so the fd gets as far as claiming the
        # interface, which a plain file cannot do: the fd went through.
        run_with(files["ch340"], "fd:@%s: claim interface 0" % name,
                 "a CH340 fd reaches the interface claim")

        # As a daemon: every re-open is a new connection with a fresh fd.
        srv = FdServer(name, files["ch340"])
        wport = free_port()
        p = start([VECTIS, "-U", "fd:@" + name, "-w", "127.0.0.1:%d" % wport])
        try:
            st = wait_status(wport, "mode", "usb")
            check(st.get("target") == "fd:@" + name and st.get("link") == "offline",
                  "-U fd: daemon /status target %r, link %r" % (st.get("target"), st.get("link")))
            end = time.time() + 8
            while srv.accepted < 2 and time.time() < end:
                time.sleep(0.1)
            check(srv.accepted >= 2, "-U fd: daemon asks the app again on re-open (%d)"
                  % srv.accepted)
            check(cpu_share(p.pid, 1.0) < 0.05, "-U fd: daemon retries with idle CPU")
        finally:
            log = stop(p)
            srv.close()
        check(log.count("claim interface") == 1, "-U fd: the failure is logged once")

        # Extra fds are closed, not collected: the fd count stays flat over retries.
        srv = FdServer(name, files["junk"], copies=3)
        wport = free_port()
        p = start([VECTIS, "-U", "fd:@" + name, "-w", "127.0.0.1:%d" % wport])
        try:
            wait_status(wport, "mode", "usb")
            end = time.time() + 8
            while srv.accepted < 2 and time.time() < end:
                time.sleep(0.1)
            before, n0 = len(os.listdir("/proc/%d/fd" % p.pid)), srv.accepted
            end = time.time() + 15
            while srv.accepted < n0 + 2 and time.time() < end:
                time.sleep(0.1)
            after = len(os.listdir("/proc/%d/fd" % p.pid))
            check(srv.accepted >= n0 + 2 and after <= before,
                  "-U fd: no fd leak over %d re-opens (%d -> %d open fds)"
                  % (srv.accepted - n0, before, after))
        finally:
            stop(p)
            srv.close()
    finally:
        for fd in files.values():
            os.close(fd)


def main():
    if not os.access(VECTIS, os.X_OK):
        sys.exit("no %s; build first (make)" % VECTIS)
    with tempfile.TemporaryDirectory() as tmp:
        test_server(tmp)
        test_client(tmp)
        test_web_terminal(tmp)
        test_web_password(tmp)
        test_stdin_eof(tmp)
        test_identity(tmp)
        test_usb_fd(tmp)
    test_usb_cli()
    print()
    if failures:
        print("%d FAILED:" % len(failures))
        for f in failures:
            print("  " + f)
        sys.exit(1)
    print("all passed")


if __name__ == "__main__":
    main()
