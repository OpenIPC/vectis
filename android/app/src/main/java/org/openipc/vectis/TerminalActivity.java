/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * TerminalActivity.java — the UART terminal on the box's own screen.  On a
 * TV box the console is mostly read and only short confirmations are typed,
 * and the system WebView of Android 5-7 may be too old for xterm.js, so this
 * is a native screen: the output as plain text (TermBuffer), an input line
 * with Enter, keys for Ctrl+C, y and n, and the power buttons.  Works with a
 * TV remote (D-pad).
 *
 * It is one more client of vectis' /ws/terminal on 127.0.0.1 (WsCodec), with
 * the web page password when one is set: the history (64 KB) and the input
 * are shared with the browsers.  The connection is reopened every 2 s while
 * the screen is shown.
 */
package org.openipc.vectis;

import android.app.Activity;
import android.graphics.Typeface;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.text.InputType;
import android.view.Gravity;
import android.view.KeyEvent;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.security.SecureRandom;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class TerminalActivity extends Activity {
    private static final int SHOWN_LINES = 300;
    private static final int RETRY_MS = 2000;
    private static final int REDRAW_MS = 100;

    private final Handler handler = new Handler(Looper.getMainLooper());
    private final TermBuffer term = new TermBuffer(1000);
    private final SecureRandom random = new SecureRandom();
    private ExecutorService writer;
    private TextView out, state;
    private ScrollView scroll;
    private EditText input;
    private volatile boolean shown;
    private volatile Socket socket;
    private volatile OutputStream sink;
    private boolean redrawPosted;

    private final Runnable redraw = new Runnable() {
        @Override
        public void run() {
            redrawPosted = false;
            out.setText(term.text(SHOWN_LINES));
            scroll.post(() -> scroll.fullScroll(View.FOCUS_DOWN));
        }
    };

    @Override
    protected void onCreate(Bundle saved) {
        super.onCreate(saved);
        int pad = Brass.dp(this, 12);
        LinearLayout page = new LinearLayout(this);
        page.setOrientation(LinearLayout.VERTICAL);
        page.setPadding(pad, pad, pad, pad);

        LinearLayout top = new LinearLayout(this);
        top.setOrientation(LinearLayout.HORIZONTAL);
        top.setGravity(Gravity.CENTER_VERTICAL);
        top.addView(Brass.plate(this, getString(R.string.term_title)));
        View gap = new View(this);
        top.addView(gap, new LinearLayout.LayoutParams(0, 1, 1));
        state = Brass.plate(this, getString(R.string.term_connecting));
        top.addView(state);
        page.addView(top);

        out = Brass.display(this);
        out.setBackground(null);          // the scroll view carries the display
        out.setTextSize(12);
        out.setTextColor(Brass.color(this, R.color.ink));
        out.setTypeface(Typeface.MONOSPACE);
        scroll = new ScrollView(this);
        scroll.setBackgroundResource(R.drawable.display);
        scroll.addView(out);
        LinearLayout.LayoutParams slp = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, 0, 1);
        slp.setMargins(0, pad, 0, pad);
        page.addView(scroll, slp);

        LinearLayout line = new LinearLayout(this);
        line.setOrientation(LinearLayout.HORIZONTAL);
        line.setGravity(Gravity.CENTER_VERTICAL);
        input = new EditText(this);
        input.setSingleLine(true);
        input.setTypeface(Typeface.MONOSPACE);
        input.setHint(R.string.term_hint);
        input.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_FLAG_NO_SUGGESTIONS);
        input.setImeOptions(EditorInfo.IME_ACTION_SEND);
        input.setOnEditorActionListener((v, id, ev) -> {
            boolean enter = ev != null && ev.getKeyCode() == KeyEvent.KEYCODE_ENTER;
            if (id == EditorInfo.IME_ACTION_SEND || enter) {
                if (ev == null || ev.getAction() == KeyEvent.ACTION_DOWN)
                    enter();
                return true;
            }
            return false;
        });
        line.addView(input, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1));
        key(line, R.string.term_enter, v -> enter());
        page.addView(line);

        LinearLayout keys = new LinearLayout(this);
        keys.setOrientation(LinearLayout.HORIZONTAL);
        key(keys, R.string.term_ctrl_c, v -> send(new byte[] {3}));
        key(keys, R.string.term_yes, v -> send("y\r"));
        key(keys, R.string.term_no, v -> send("n\r"));
        key(keys, R.string.term_reset, v -> power("reset"));
        key(keys, R.string.term_on, v -> power("enable"));
        key(keys, R.string.term_off, v -> power("disable"));
        page.addView(keys);

        setContentView(page);
        // Mostly read: the keyboard comes up only when the input line is clicked
        scroll.setFocusable(true);
        scroll.requestFocus();
    }

    private void key(LinearLayout row, int text, View.OnClickListener l) {
        Button b = Brass.button(this, text);
        b.setOnClickListener(l);
        row.addView(b, Brass.margins(this, 0, LinearLayout.LayoutParams.WRAP_CONTENT, 1));
    }

    @Override
    protected void onResume() {
        super.onResume();
        shown = true;
        writer = Executors.newSingleThreadExecutor();
        new Thread(this::run, "vectis-terminal").start();
    }

    @Override
    protected void onPause() {
        shown = false;
        closeSocket();
        writer.shutdownNow();
        super.onPause();
    }

    private void closeSocket() {
        Socket s = socket;
        if (s != null) {
            try {
                s.close();
            } catch (IOException ignored) {
                // closing anyway
            }
        }
    }

    /** The text of the input line, then CR (Enter on a serial console). */
    private void enter() {
        String t = input.getText().toString();
        input.setText("");
        send(t + "\r");
    }

    private void send(String s) {
        send(s.getBytes(StandardCharsets.UTF_8));
    }

    private void send(byte[] data) {
        OutputStream o = sink;
        if (o == null || writer.isShutdown())
            return;
        byte[] mask = new byte[4];
        random.nextBytes(mask);
        byte[] f = WsCodec.frame(WsCodec.BINARY, data, 0, data.length, mask);
        writer.execute(() -> write(o, f));
    }

    private void write(OutputStream o, byte[] f) {
        try {
            synchronized (o) {
                o.write(f);
                o.flush();
            }
        } catch (IOException e) {
            closeSocket();   // the reader sees it and reconnects
        }
    }

    private void power(String what) {
        int port = Settings.webPort(this);
        String pw = Settings.password(this);
        writer.execute(() -> {
            try {
                Http.json(port, "/power/" + what, pw, true);
            } catch (Exception ignored) {
                // the state line tells whether vectis is there
            }
        });
    }

    private void setState(int text) {
        handler.post(() -> state.setText(text));
    }

    private void dirty() {
        handler.post(() -> {
            if (!redrawPosted) {
                redrawPosted = true;
                handler.postDelayed(redraw, REDRAW_MS);
            }
        });
    }

    /** Connect, read until the connection ends, retry while the screen is shown. */
    private void run() {
        while (shown) {
            int port = Settings.webPort(this);
            String pw = Settings.password(this);
            try (Socket s = new Socket()) {
                socket = s;
                s.connect(new InetSocketAddress("127.0.0.1", port), RETRY_MS);
                s.setTcpNoDelay(true);
                byte[] k = new byte[16];
                random.nextBytes(k);
                OutputStream o = s.getOutputStream();
                o.write(WsCodec.request(port, Http.base64(k), pw));
                o.flush();
                InputStream in = s.getInputStream();
                String head = readHead(in);
                if (head.startsWith("HTTP/1.1 401")) {
                    setState(R.string.term_password);
                } else if (head.startsWith("HTTP/1.1 101")) {
                    term.clear();             // the history follows
                    sink = o;
                    setState(R.string.term_connected);
                    pump(in, o);
                }
            } catch (IOException | RuntimeException e) {
                // vectis not up yet, or the connection dropped
            }
            sink = null;
            socket = null;
            if (!shown)
                break;
            setState(R.string.term_reconnecting);
            try {
                Thread.sleep(RETRY_MS);
            } catch (InterruptedException e) {
                break;
            }
        }
    }

    private static String readHead(InputStream in) throws IOException {
        StringBuilder h = new StringBuilder();
        int c;
        while ((c = in.read()) >= 0 && h.length() < 4096) {
            h.append((char) c);
            if (h.length() >= 4 && h.lastIndexOf("\r\n\r\n") == h.length() - 4)
                break;
        }
        return h.toString();
    }

    private void pump(InputStream in, OutputStream o) throws IOException {
        WsCodec.Parser p = new WsCodec.Parser();
        byte[] buf = new byte[8192];
        int n;
        while (shown && (n = in.read(buf)) > 0) {
            for (WsCodec.Frame f : p.feed(buf, 0, n)) {
                if (f.opcode == WsCodec.BINARY || f.opcode == WsCodec.CONT) {
                    term.feed(f.data, 0, f.data.length);
                    dirty();
                } else if (f.opcode == WsCodec.PING) {
                    byte[] mask = new byte[4];
                    random.nextBytes(mask);
                    write(o, WsCodec.frame(WsCodec.PONG, f.data, 0, f.data.length, mask));
                } else if (f.opcode == WsCodec.CLOSE) {
                    return;
                }
            }
        }
    }
}
