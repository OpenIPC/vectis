/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * Settings.java — the few settings of the app, in SharedPreferences: the
 * baud rate, the two TCP ports (web 8085 as Decoder and Microsnander, which
 * camex servers expect), the web page password (empty: none), start on boot,
 * and whether the user wants vectis running (Stop clears it).  A value saved
 * before keeps winning over a new default.
 */
package org.openipc.vectis;

import android.content.Context;
import android.content.SharedPreferences;

final class Settings {
    static final int DEFAULT_BAUD = 115200;
    static final int DEFAULT_CONSOLE = 2021;
    static final int DEFAULT_WEB = 8085;
    /** The baud rates vectis accepts (baud_table in vectis.c). */
    static final int[] BAUDS = {300, 1200, 2400, 4800, 9600, 19200, 38400, 57600,
                                115200, 230400, 460800, 921600};
    private static final String BAUD = "baud", CONSOLE = "console_port", WEB = "web_port",
            BOOT = "start_on_boot", RUNNING = "running", PASSWORD = "web_password";

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
        return stored(prefs(c).getAll(), WEB, DEFAULT_WEB);
    }

    /** A saved int, or the default when none was saved: a new default never overwrites. */
    static int stored(java.util.Map<String, ?> saved, String key, int def) {
        Object v = saved == null ? null : saved.get(key);
        return v instanceof Integer ? (Integer) v : def;
    }

    /** vectis takes a web password of at most this many bytes (UTF-8). */
    static final int MAX_PASSWORD = 255;

    static boolean validPassword(String p) {
        return p == null || p.getBytes(java.nio.charset.StandardCharsets.UTF_8).length <= MAX_PASSWORD;
    }

    /** The web page password; "" means the page is open. */
    static String password(Context c) {
        String p = prefs(c).getString(PASSWORD, "");
        return p == null ? "" : p;
    }

    static void setPassword(Context c, String password) {
        prefs(c).edit().putString(PASSWORD, password == null ? "" : password).apply();
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
