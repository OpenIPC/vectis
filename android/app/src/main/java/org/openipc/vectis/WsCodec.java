/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * WsCodec.java — the WebSocket (RFC 6455) of the terminal screen, client
 * side: the handshake request, masked frames for vectis, and a parser for
 * the frames vectis sends (the history arrives as one frame of up to 64 KB).
 * No Android classes: tested on the JVM (LogicTest).
 */
package org.openipc.vectis;

import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

final class WsCodec {
    static final int CONT = 0x0, TEXT = 0x1, BINARY = 0x2, CLOSE = 0x8, PING = 0x9, PONG = 0xA;
    /** Larger frames than vectis ever sends: the connection is wrong, drop it. */
    static final int MAX_FRAME = 1 << 20;

    private WsCodec() {
    }

    /** The upgrade request; no Origin, so vectis needs no token (README). */
    static byte[] request(int port, String key, String password) {
        StringBuilder r = new StringBuilder()
                .append("GET /ws/terminal HTTP/1.1\r\n")
                .append("Host: 127.0.0.1:").append(port).append("\r\n")
                .append("Upgrade: websocket\r\nConnection: Upgrade\r\n")
                .append("Sec-WebSocket-Key: ").append(key).append("\r\n")
                .append("Sec-WebSocket-Version: 13\r\n");
        String auth = Http.basic(password);
        if (auth != null)
            r.append("Authorization: ").append(auth).append("\r\n");
        return r.append("\r\n").toString().getBytes(StandardCharsets.US_ASCII);
    }

    /** A client frame: FIN, the opcode, masked with mask (4 bytes) as the RFC requires. */
    static byte[] frame(int opcode, byte[] data, int off, int len, byte[] mask) {
        int hl = len < 126 ? 2 : len < 65536 ? 4 : 10;
        byte[] f = new byte[hl + 4 + len];
        f[0] = (byte) (0x80 | opcode);
        if (len < 126) {
            f[1] = (byte) (0x80 | len);
        } else if (len < 65536) {
            f[1] = (byte) (0x80 | 126);
            f[2] = (byte) (len >> 8);
            f[3] = (byte) len;
        } else {
            f[1] = (byte) (0x80 | 127);
            for (int i = 0; i < 8; i++)
                f[2 + i] = (byte) ((long) len >> (56 - 8 * i));
        }
        System.arraycopy(mask, 0, f, hl, 4);
        for (int i = 0; i < len; i++)
            f[hl + 4 + i] = (byte) (data[off + i] ^ mask[i & 3]);
        return f;
    }

    static final class Frame {
        final int opcode;
        final byte[] data;

        Frame(int opcode, byte[] data) {
            this.opcode = opcode;
            this.data = data;
        }
    }

    /** Collects bytes from vectis and cuts them into frames. */
    static final class Parser {
        private byte[] buf = new byte[4096];
        private int len;

        /** The frames completed by these bytes; throws on a frame that cannot be right. */
        List<Frame> feed(byte[] b, int off, int n) {
            if (len + n > buf.length) {
                byte[] nb = new byte[Math.max(buf.length * 2, len + n)];
                System.arraycopy(buf, 0, nb, 0, len);
                buf = nb;
            }
            System.arraycopy(b, off, buf, len, n);
            len += n;
            List<Frame> out = new ArrayList<>();
            int p = 0;
            while (len - p >= 2) {
                int op = buf[p] & 0x0f;
                boolean masked = (buf[p + 1] & 0x80) != 0;
                long size = buf[p + 1] & 0x7f;
                int hl = 2;
                if (size == 126) {
                    if (len - p < 4)
                        break;
                    size = ((buf[p + 2] & 0xff) << 8) | (buf[p + 3] & 0xff);
                    hl = 4;
                } else if (size == 127) {
                    if (len - p < 10)
                        break;
                    size = 0;
                    for (int i = 0; i < 8; i++)
                        size = (size << 8) | (buf[p + 2 + i] & 0xff);
                    hl = 10;
                }
                if (size > MAX_FRAME)
                    throw new IllegalStateException("frame of " + size + " bytes");
                int ml = masked ? 4 : 0;
                if (len - p < hl + ml + size)
                    break;
                byte[] d = new byte[(int) size];
                System.arraycopy(buf, p + hl + ml, d, 0, d.length);
                if (masked)
                    for (int i = 0; i < d.length; i++)
                        d[i] ^= buf[p + hl + (i & 3)];
                out.add(new Frame(op, d));
                p += hl + ml + (int) size;
            }
            System.arraycopy(buf, p, buf, 0, len - p);
            len -= p;
            return out;
        }
    }
}
