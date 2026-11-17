/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * TermBuffer.java — the text of the app's terminal screen: UART bytes in,
 * plain lines out.  UTF-8 is decoded across reads; escape sequences
 * (colours, cursor moves, titles) are dropped, except "erase to the end of
 * the line"; CR returns to the start of the line, so a progress line
 * rewrites itself; backspace and tabs move the column.  Keeps the last
 * maxLines lines; a line wraps at MAX_COLS.  One column per character: a
 * character outside the BMP shows as U+FFFD; C1 controls act as their ESC forms.  No Android classes: tested on the JVM (LogicTest).
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
    /** A longer line wraps, so output without newlines stays bounded. */
    static final int MAX_COLS = 1024;
    private static final int NONE = 0, ESC = 1, CSI = 2, OSC = 3, OSC_ESC = 4, ESC_INTER = 5;

    private final int maxLines;
    private final ArrayDeque<String> lines = new ArrayDeque<>();
    private final StringBuilder cur = new StringBuilder();
    private int col;
    private int state = NONE;
    private final StringBuilder params = new StringBuilder();   // of the CSI sequence
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
        while (out.hasRemaining()) {
            char ch = out.get();
            if (Character.isHighSurrogate(ch)) {
                // One column per character: a character outside the BMP (an emoji) shows as
                // U+FFFD, so backspace and wrapping never split a surrogate pair
                if (out.hasRemaining() && Character.isLowSurrogate(out.get(out.position())))
                    out.get();
                else if (!out.hasRemaining())
                    continue;       // its low half comes with the next read: not kept, rare on a UART
                ch = '\ufffd';
            } else if (Character.isLowSurrogate(ch)) {
                continue;
            }
            put(ch);
        }
    }

    private void put(char ch) {
        if (state != NONE && (ch == 0x18 || ch == 0x1a)) {
            state = NONE;               // CAN, SUB: cancel a sequence in progress
            return;
        }
        switch (state) {
            case ESC:
                params.setLength(0);
                // ESC ( B and the like: intermediates 0x20-0x2F, then one final byte
                // ESC ] (OSC) and ESC P/X/^/_ (DCS, SOS, PM, APC): strings up to BEL, ESC \ or ST
                state = ch == '[' ? CSI
                        : ch == ']' || ch == 'P' || ch == 'X' || ch == '^' || ch == '_' ? OSC
                        : ch >= 0x20 && ch <= 0x2f ? ESC_INTER : NONE;
                return;
            case ESC_INTER:
                if (ch >= 0x30 && ch <= 0x7e)
                    state = NONE;
                return;
            case CSI:
                if (ch >= 0x40 && ch <= 0x7e) {
                    if (ch == 'K')
                        eraseLine(params.toString());
                    state = NONE;
                } else if (params.length() < 32) {
                    params.append(ch);
                }
                return;
            case OSC:
                if (ch == 7 || ch == 0x9c)
                    state = NONE;       // BEL or ST (U+009C)
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
        } else if (ch >= 0x80 && ch <= 0x9f) {
            // C1 controls, as UTF-8: CSI and the string introducers as their ESC forms
            params.setLength(0);
            state = ch == 0x9b ? CSI
                    : ch == 0x9d || ch == 0x90 || ch == 0x98 || ch == 0x9e || ch == 0x9f ? OSC : NONE;
        } else if (ch == '\n') {
            newline();
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
            if (col >= MAX_COLS)
                newline();
            while (cur.length() < col)
                cur.append(' ');            // after an erase that left the cursor further on
            if (col < cur.length())
                cur.setCharAt(col, ch);
            else
                cur.append(ch);
            col++;
        }
    }

    private void newline() {
        lines.addLast(cur.toString());
        while (lines.size() > maxLines)
            lines.removeFirst();
        cur.setLength(0);
        col = 0;
    }

    /** CSI K: 0 (or none) from the cursor to the end, 1 from the start to the cursor, 2 all. */
    private void eraseLine(String p) {
        if (p.isEmpty() || p.equals("0")) {
            if (col < cur.length())
                cur.setLength(col);
        } else if (p.equals("1")) {
            for (int i = 0; i <= col && i < cur.length(); i++)
                cur.setCharAt(i, ' ');
        } else if (p.equals("2")) {
            cur.setLength(0);
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
        params.setLength(0);
        partial.clear();
        utf8.reset();
    }
}
