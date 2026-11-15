/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * LogBuffer.java — the last lines of vectis' messages in memory, each with
 * the time it arrived, for the app screen.
 */
package org.openipc.vectis;

import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.Date;
import java.util.Locale;

public final class LogBuffer {
    private final int capacity;
    private final ArrayDeque<String> lines = new ArrayDeque<>();
    private final SimpleDateFormat stamp = new SimpleDateFormat("MM-dd HH:mm:ss", Locale.US);

    public LogBuffer(int capacity) {
        this.capacity = capacity;
    }

    public synchronized void add(String line) {
        if (lines.size() >= capacity)
            lines.removeFirst();
        lines.addLast(stamp.format(new Date()) + " " + line);
    }

    /** The last n lines, oldest first. */
    public synchronized String tail(int n) {
        StringBuilder sb = new StringBuilder();
        int skip = lines.size() - n;
        for (String l : lines)
            if (skip-- <= 0)
                sb.append(l).append('\n');
        return sb.toString();
    }

    public synchronized int size() {
        return lines.size();
    }
}
