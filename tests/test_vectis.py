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
# SIGNATURE, BRK), the -w web API, the RFC 2217 client (-h/-p) against
# the -l server, stdin EOF and device loss without a busy loop, and the
# -U command line of the USB builds.
#
# Usage: tests/test_vectis.py [DIR]   (DIR holds vectis and vectis-*; default .)
# Python 3 standard library only; Linux.
#

import json
import os
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
import urllib.request

BIN_DIR = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else ".")
VECTIS = os.path.join(BIN_DIR, "vectis")
USB_BINS = sorted(os.path.join(BIN_DIR, f) for f in os.listdir(BIN_DIR)
                  if f.startswith("vectis-") and f != "vectis-bootrom"
                  and os.access(os.path.join(BIN_DIR, f), os.X_OK))

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
        check("ok" in out, "device output reaches the client")
    finally:
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


def test_usb_cli():
    print("== -U command line")
    r = subprocess.run([VECTIS, "-U", "1-1"], capture_output=True, text=True)
    check(r.returncode == 1 and "no USB support" in r.stderr, "vectis: -U refused")
    for b in USB_BINS:
        name = os.path.basename(b)
        r = subprocess.run([b, "--version"], capture_output=True, text=True)
        check("USB chips (-U)" in r.stdout, "%s --version lists the chips" % name)
        r = subprocess.run([b, "-U", "nonsense"], capture_output=True, text=True)
        check(r.returncode == 1 and "Invalid -U spec" in r.stderr, "%s: bad -U spec" % name)
        r = subprocess.run([b, "-U", "usb:zz:1"], capture_output=True, text=True)
        check(r.returncode == 1 and "Invalid -U spec" in r.stderr, "%s: bad VID:PID" % name)
        r = subprocess.run([b, "-U", "1-1", "-u", "/dev/null"], capture_output=True, text=True)
        check(r.returncode == 1, "%s: -U with -u refused" % name)
        r = subprocess.run([b, "-U", "9-9.9"], capture_output=True, text=True,
                           stdin=subprocess.DEVNULL)
        check(r.returncode == 1 and "no USB device at 9-9.9" in r.stderr,
              "%s: absent device reported" % name)
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


def main():
    if not os.access(VECTIS, os.X_OK):
        sys.exit("no %s; build first (make)" % VECTIS)
    with tempfile.TemporaryDirectory() as tmp:
        test_server(tmp)
        test_client(tmp)
        test_stdin_eof(tmp)
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
