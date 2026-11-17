/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * Http.java — the app's own requests to vectis on 127.0.0.1: /status for
 * the screen and /power/* for the terminal's buttons, with the web page
 * password (HTTP Basic, empty user name) when one is set.
 */
package org.openipc.vectis;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import org.json.JSONException;
import org.json.JSONObject;

final class Http {
    private Http() {
    }

    /** The Authorization header value for a password, null for none. */
    static String basic(String password) {
        if (password == null || password.isEmpty())
            return null;
        return "Basic " + base64((":" + password).getBytes(StandardCharsets.UTF_8));
    }

    private static final char[] B64 =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/".toCharArray();

    /** Base64 with padding (java.util.Base64 only from Android 8.0). */
    static String base64(byte[] in) {
        StringBuilder o = new StringBuilder();
        for (int i = 0; i < in.length; i += 3) {
            int v = (in[i] & 0xff) << 16;
            if (i + 1 < in.length)
                v |= (in[i + 1] & 0xff) << 8;
            if (i + 2 < in.length)
                v |= in[i + 2] & 0xff;
            o.append(B64[(v >> 18) & 63]).append(B64[(v >> 12) & 63]);
            o.append(i + 1 < in.length ? B64[(v >> 6) & 63] : '=');
            o.append(i + 2 < in.length ? B64[v & 63] : '=');
        }
        return o.toString();
    }

    /** GET (or POST) http://127.0.0.1:PORT/PATH and parse the JSON reply. */
    static JSONObject json(int port, String path, String password, boolean post)
            throws IOException, JSONException {
        HttpURLConnection c = (HttpURLConnection) new URL("http://127.0.0.1:" + port + path)
                .openConnection();
        c.setConnectTimeout(800);
        c.setReadTimeout(post ? 65000 : 800);   // /power/reset answers after the pulse (-t)
        String auth = basic(password);
        if (auth != null)
            c.setRequestProperty("Authorization", auth);
        if (post) {
            c.setRequestMethod("POST");
            c.setFixedLengthStreamingMode(0);
            c.setDoOutput(true);
        }
        try (InputStream in = c.getResponseCode() < 400 ? c.getInputStream() : c.getErrorStream()) {
            ByteArrayOutputStream b = new ByteArrayOutputStream();
            byte[] buf = new byte[2048];
            int n;
            while (in != null && (n = in.read(buf)) > 0 && b.size() < 65536)
                b.write(buf, 0, n);
            return new JSONObject(new String(b.toByteArray(), StandardCharsets.UTF_8));
        } finally {
            c.disconnect();
        }
    }
}
