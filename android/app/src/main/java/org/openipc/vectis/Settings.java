/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * Settings.java — the few settings of the app, in SharedPreferences: the
 * baud rate and the two TCP ports (defaults as in the README examples),
 * start on boot, and whether the user wants vectis running (Stop clears it).
 */
package org.openipc.vectis;

import android.content.Context;
import android.content.SharedPreferences;

final class Settings {
    static final int DEFAULT_BAUD = 115200;
    static final int DEFAULT_CONSOLE = 2021;
    static final int DEFAULT_WEB = 2081;
    /** The baud rates vectis accepts (baud_table in vectis.c). */
    static final int[] BAUDS = {300, 1200, 2400, 4800, 9600, 19200, 38400, 57600,
                                115200, 230400, 460800, 921600};
    private static final String BAUD = "baud", CONSOLE = "console_port", WEB = "web_port",
            BOOT = "start_on_boot", RUNNING = "running";

    private Settings() {}

    /** Ports below this need root to listen on. */
    static final int MIN_PORT = 1024;

    /** True if vectis can run with these: a rate of BAUDS, two different ports 1024..65535. */
    static boolean valid(int baud, int console, int web) {
        boolean known = false;
        for (int b : BAUDS)
            known |= b == baud;
        return known && console >= MIN_PORT && console <= 65535 && web >= MIN_PORT && web <= 65535
                && console != web;
    }

    private static SharedPreferences prefs(Context c) {
        return c.getSharedPreferences("vectis", Context.MODE_PRIVATE);
    }

    static int baud(Context c) {
        return prefs(c).getInt(BAUD, DEFAULT_BAUD);
    }

    static int consolePort(Context c) {
        return prefs(c).getInt(CONSOLE, DEFAULT_CONSOLE);
    }

    static int webPort(Context c) {
        return prefs(c).getInt(WEB, DEFAULT_WEB);
    }

    static boolean startOnBoot(Context c) {
        return prefs(c).getBoolean(BOOT, true);
    }

    /** True until the user presses Stop: then neither boot nor a plug-in starts vectis. */
    static boolean running(Context c) {
        return prefs(c).getBoolean(RUNNING, true);
    }

    static void setPorts(Context c, int baud, int console, int web) {
        prefs(c).edit().putInt(BAUD, baud).putInt(CONSOLE, console).putInt(WEB, web).apply();
    }

    static void setStartOnBoot(Context c, boolean on) {
        prefs(c).edit().putBoolean(BOOT, on).apply();
    }

    static void setRunning(Context c, boolean on) {
        prefs(c).edit().putBoolean(RUNNING, on).apply();
    }
}
