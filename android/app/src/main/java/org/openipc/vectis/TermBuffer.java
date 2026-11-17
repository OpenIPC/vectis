/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * TermBuffer.java — the text of the app's terminal screen: UART bytes in,
 * plain lines out.  UTF-8 is decoded across reads; escape sequences
 * (colours, cursor moves, titles) are dropped, except "erase to the end of
 * the line"; CR returns to the start of the line, so a progress line
 * rewrites itself; backspace and tabs move the column.  Keeps the last
 * maxLines lines.  No Android classes: tested on the JVM (LogicTest).
 */
package org.openipc.vectis;

import java.nio.ByteBuffer;
import java.nio.CharBuffer;
import java.nio.charset.CharsetDecoder;
import java.nio.charset.CodingErrorAction;
import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;
import java.util.Iterator;

final class TermBuffer {
    private static final int NONE = 0, ESC = 1, CSI = 2, OSC = 3, OSC_ESC = 4;

    private final int maxLines;
    private final ArrayDeque<String> lines = new ArrayDeque<>();
    private final StringBuilder cur = new StringBuilder();
    private int col;
    private int state = NONE;
    private final CharsetDecoder utf8 = StandardCharsets.UTF_8.newDecoder()
            .onMalformedInput(CodingErrorAction.REPLACE)
            .onUnmappableCharacter(CodingErrorAction.REPLACE);
    private final ByteBuffer partial = ByteBuffer.allocate(8);   // an unfinished UTF-8 sequence

    TermBuffer(int maxLines) {
        this.maxLines = maxLines;
    }

    synchronized void feed(byte[] b, int off, int len) {
        partial.flip();
        ByteBuffer in = ByteBuffer.allocate(partial.remaining() + len);
        in.put(partial).put(b, off, len).flip();
        partial.clear();
        CharBuffer out = CharBuffer.allocate(in.remaining() + 1);
        utf8.decode(in, out, false);
        if (in.remaining() <= partial.capacity())
            partial.put(in);
        out.flip();
        while (out.hasRemaining())
            put(out.get());
    }

    private void put(char ch) {
        switch (state) {
            case ESC:
                state = ch == '[' ? CSI : ch == ']' ? OSC : NONE;
                return;
            case CSI:
                if (ch >= 0x40 && ch <= 0x7e) {
                    if (ch == 'K')
                        cur.setLength(col);      // erase to the end of the line
                    state = NONE;
                }
                return;
            case OSC:
                if (ch == 7)
                    state = NONE;
                else if (ch == 27)
                    state = OSC_ESC;
                return;
            case OSC_ESC:
                state = ch == '\\' ? NONE : OSC;
                return;
            default:
                break;
        }
        if (ch == 27) {
            state = ESC;
        } else if (ch == '\n') {
            lines.addLast(cur.toString());
            while (lines.size() > maxLines)
                lines.removeFirst();
            cur.setLength(0);
            col = 0;
        } else if (ch == '\r') {
            col = 0;
        } else if (ch == '\b') {
            if (col > 0)
                col--;
        } else if (ch == '\t') {
            do
                put(' ');
            while (col % 8 != 0);
        } else if (ch >= 0x20 && ch != 0x7f) {
            if (col < cur.length())
                cur.setCharAt(col, ch);
            else
                cur.append(ch);
            col++;
        }
    }

    /** The last n lines, the one being written included. */
    synchronized String text(int n) {
        StringBuilder t = new StringBuilder();
        int skip = Math.max(0, lines.size() + 1 - n);
        Iterator<String> it = lines.iterator();
        for (int i = 0; it.hasNext(); i++) {
            String l = it.next();
            if (i >= skip)
                t.append(l).append('\n');
        }
        return t.append(cur).toString();
    }

    synchronized void clear() {
        lines.clear();
        cur.setLength(0);
        col = 0;
        state = NONE;
        partial.clear();
        utf8.reset();
    }
}
