/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * Http.java — the app's own requests to vectis on its local socket
 * (--web-local @org.openipc.vectis.web): /status for the screen and
 * /power/* for the terminal's buttons.  The socket takes only our own UID
 * and no password, and it is there with remote access off too.  HTTP/1.1,
 * one request per connection: vectis closes it after the reply.
 */
package org.openipc.vectis;

import android.net.LocalSocket;
import android.net.LocalSocketAddress;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
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

    /**
     * Connects s (made and published by the caller, so a pause can close it meanwhile) to
     * vectis' --web-local, and checks that the peer is our own uid: an abstract name belongs to
     * whoever binds it first, and another app holding it must not get the terminal's input.
     */
    static void connect(LocalSocket s) throws IOException {
        s.connect(new LocalSocketAddress(VectisService.WEB_SOCKET, LocalSocketAddress.Namespace.ABSTRACT));
        if (s.getPeerCredentials().getUid() != android.os.Process.myUid())
            throw new IOException("@" + VectisService.WEB_SOCKET + " is not vectis");
    }

    /** A reply: the status code and the body. */
    static final class Reply {
        final int code;
        final String body;

        Reply(int code, String body) {
            this.code = code;
            this.body = body;
        }
    }

    /** The status code and body of a raw HTTP/1.x reply; code 0 if it is not one. */
    static Reply parse(byte[] raw, int len) {
        String r = new String(raw, 0, len, StandardCharsets.UTF_8);
        int sp = r.indexOf(' ');
        int code = 0;
        if (r.startsWith("HTTP/1.") && sp > 0 && r.length() >= sp + 4) {
            try {
                code = Integer.parseInt(r.substring(sp + 1, sp + 4));
            } catch (NumberFormatException e) {
                code = 0;
            }
        }
        int b = r.indexOf("\r\n\r\n");
        return new Reply(code, b < 0 ? "" : r.substring(b + 4));
    }

    /** One request on the local socket; the reply read until vectis closes the connection. */
    static Reply request(String method, String path, int timeoutMs) throws IOException {
        try (LocalSocket s = new LocalSocket()) {
            connect(s);
            s.setSoTimeout(timeoutMs);
            OutputStream o = s.getOutputStream();
            o.write((method + " " + path + " HTTP/1.1\r\nHost: localhost\r\nContent-Length: 0\r\n"
                    + "Connection: close\r\n\r\n").getBytes(StandardCharsets.US_ASCII));
            o.flush();
            InputStream in = s.getInputStream();
            ByteArrayOutputStream b = new ByteArrayOutputStream();
            byte[] buf = new byte[2048];
            int n;
            while ((n = in.read(buf)) > 0 && b.size() < 65536)
                b.write(buf, 0, n);
            byte[] all = b.toByteArray();
            return parse(all, all.length);
        }
    }

    /** POST PATH: null when vectis did it, else what went wrong (its "error", or the HTTP status). */
    static String post(String path) {
        try {
            Reply r = request("POST", path, 65000);   // /power/reset answers after the pulse (-t)
            if (r.code > 0 && r.code < 400)
                return null;
            try {
                String e = new JSONObject(r.body).optString("error", "");
                return e.isEmpty() ? "HTTP " + r.code : e;
            } catch (JSONException e) {
                return "HTTP " + r.code;
            }
        } catch (IOException e) {
            return e.getMessage() != null ? e.getMessage() : "no answer";
        }
    }

    /** GET PATH and parse the JSON reply (the screen's /status). */
    static JSONObject json(String path) throws IOException, JSONException {
        Reply r = request("GET", path, 800);
        if (r.code != 200)
            throw new IOException("HTTP " + r.code);
        return new JSONObject(r.body);
    }
}
