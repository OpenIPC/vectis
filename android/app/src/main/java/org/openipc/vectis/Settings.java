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
    private static final String BAUD = "baud", CONSOLE = "console_port", WEB = "web_port",
            BOOT = "start_on_boot", RUNNING = "running";

    private Settings() {}

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
