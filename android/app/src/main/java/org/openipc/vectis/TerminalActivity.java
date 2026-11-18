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
import android.widget.Toast;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.security.SecureRandom;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

public class TerminalActivity extends Activity {
    /** On screen: the recent output, a bound for weak TV boxes; the page keeps 5000 lines. */
    private static final int SHOWN_LINES = 300;
    private static final int RETRY_MS = 2000;
    private static final int REDRAW_MS = 100;

    private final Handler handler = new Handler(Looper.getMainLooper());
    private final TermBuffer term = new TermBuffer(1000);
    private final SecureRandom random = new SecureRandom();
    /** Frames for vectis, in order; the power buttons have their own thread. */
    private ExecutorService writer, powerer;
    private TextView out, state;
    private ScrollView scroll;
    private EditText input;
    /** The current reader; an older one (before a pause) sees it changed and ends. */
    private volatile Object generation;
    /** The connection keys go to, null while there is none. */
    private volatile Conn conn;
    private boolean redrawPosted;

    /** One WebSocket connection: its socket, the stream to write to it and its writer. */
    private static final class Conn {
        final Socket socket;
        final OutputStream out;
        final ExecutorService writer;
        private final AtomicBoolean closing = new AtomicBoolean();

        Conn(Socket socket, OutputStream out, ExecutorService writer) {
            this.socket = socket;
            this.out = out;
            this.writer = writer;
        }

        void close() {
            try {
                socket.close();
            } catch (IOException ignored) {
                // closing anyway
            }
        }

        /**
         * On leaving the screen: frames already accepted (Enter cleared the input) go out
         * first, then the socket closes; a peer that stops reading holds it 2 s at most.
         * The writer takes nothing new.  Safe to call twice (onPause and the reader).
         */
        void closeAfterWrites() {
            if (!closing.compareAndSet(false, true))
                return;
            try {
                writer.execute(this::close);
            } catch (RejectedExecutionException e) {
                close();
            }
            writer.shutdown();
            Thread t = new Thread(() -> {
                try {
                    if (!writer.awaitTermination(2, TimeUnit.SECONDS))
                        close();
                } catch (InterruptedException e) {
                    close();
                }
            }, "vectis-terminal-close");
            t.setDaemon(true);
            t.start();
        }
    }

    private final Runnable redraw = new Runnable() {
        @Override
        public void run() {
            redrawPosted = false;
            // Follow the output only if the reader is at the bottom: scrolling back stays put
            boolean atBottom = scroll.getScrollY() + scroll.getHeight() >= out.getBottom() - Brass.dp(
                    TerminalActivity.this, 24);
            out.setText(term.text(SHOWN_LINES));
            // scrollTo, not fullScroll: that one also moves the focus off the input and the keys
            if (atBottom)
                scroll.post(() -> scroll.scrollTo(0, Math.max(0, out.getBottom() - scroll.getHeight())));
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
        key(keys, R.string.term_reset, v -> power("reset", R.string.term_reset));
        key(keys, R.string.term_on, v -> power("enable", R.string.term_on));
        key(keys, R.string.term_off, v -> power("disable", R.string.term_off));
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

    private Thread reader;
    private volatile Socket connecting;   // the reader's socket before the handshake ends

    @Override
    protected void onResume() {
        super.onResume();
        Object gen = new Object();
        generation = gen;
        writer = Executors.newSingleThreadExecutor();
        powerer = Executors.newSingleThreadExecutor();
        reader = new Thread(() -> run(gen), "vectis-terminal");
        reader.start();
    }

    @Override
    protected void onPause() {
        generation = null;
        reader.interrupt();
        Conn c = conn;
        conn = null;                    // nothing more is sent to it, before it is closed
        if (c != null) {
            // Wake the reader without closing: the writer closes after the accepted frames
            try {
                c.socket.shutdownInput();
            } catch (IOException ignored) {
                // closed already
            }
            c.closeAfterWrites();
        } else {
            Socket s = connecting;      // still connecting: nothing was sent to it
            if (s != null) {
                try {
                    s.close();
                } catch (IOException ignored) {
                    // closing anyway
                }
            }
        }
        writer.shutdown();
        powerer.shutdownNow();
        super.onPause();
    }

    /** The text of the input line, then CR (Enter on a serial console). */
    private void enter() {
        String t = input.getText().toString();
        if (send(t + "\r"))
            input.setText("");          // kept while not connected, to be sent again
    }

    private boolean send(String s) {
        return send(s.getBytes(StandardCharsets.UTF_8));
    }

    /** False, with a note on the screen, when there is no connection to send to. */
    private boolean send(byte[] data) {
        Conn c = conn;
        if (c == null || writer.isShutdown()) {
            Toast.makeText(this, R.string.term_not_connected, Toast.LENGTH_SHORT).show();
            return false;
        }
        byte[] mask = new byte[4];
        random.nextBytes(mask);
        byte[] f = WsCodec.frame(WsCodec.BINARY, data, 0, data.length, mask);
        writer.execute(() -> write(c, f));
        return true;
    }

    /** Write a frame; a failure closes this connection only, its reader then reconnects. */
    private static void write(Conn c, byte[] f) {
        try {
            synchronized (c.out) {
                c.out.write(f);
                c.out.flush();
            }
        } catch (IOException e) {
            c.close();
        }
    }

    private void power(String what, int label) {
        int port = Settings.webPort(this);
        String pw = Settings.password(this);
        powerer.execute(() -> {
            String err = Http.post(port, "/power/" + what, pw);
            if (err != null)
                handler.post(() -> Toast.makeText(this, getString(R.string.term_failed,
                        getString(label), err), Toast.LENGTH_LONG).show());
        });
    }

    /** The label, unless a newer reader (after a pause) has taken over meanwhile. */
    private void setState(Object gen, int text) {
        handler.post(() -> {
            if (generation == gen)
                state.setText(text);
        });
    }

    private void dirty() {
        handler.post(() -> {
            if (!redrawPosted) {
                redrawPosted = true;
                handler.postDelayed(redraw, REDRAW_MS);
            }
        });
    }

    /** Connect, read until the connection ends, retry while this generation is current. */
    private void run(Object gen) {
        while (generation == gen) {
            int port = Settings.webPort(this);
            String pw = Settings.password(this);
            boolean refused = false;
            Conn mine = null;
            Socket s = new Socket();
            try {
                connecting = s;
                if (generation != gen)
                    break;
                s.connect(new InetSocketAddress("127.0.0.1", port), RETRY_MS);
                s.setTcpNoDelay(true);
                byte[] k = new byte[16];
                random.nextBytes(k);
                OutputStream o = s.getOutputStream();
                o.write(WsCodec.request(port, Http.base64(k), pw));
                o.flush();
                InputStream in = s.getInputStream();
                s.setSoTimeout(2 * RETRY_MS);           // a listener that never answers: retry
                String head = readHead(in);
                s.setSoTimeout(0);                      // a quiet UART is not a dead connection
                if (head.startsWith("HTTP/1.1 401")) {
                    refused = true;
                    setState(gen, R.string.term_password);   // kept until the next attempt succeeds
                } else if (head.startsWith("HTTP/1.1 101") && generation == gen) {
                    term.clear();                       // the history follows, if there is one
                    dirty();
                    mine = new Conn(s, o, writer);
                    conn = mine;
                    if (generation != gen) {            // paused meanwhile: not ours to publish
                        conn = null;
                        break;
                    }
                    setState(gen, R.string.term_connected);
                    pump(in, mine, gen);
                }
            } catch (IOException | RuntimeException e) {
                // vectis not up yet, or the connection dropped
            }
            if (mine != null && conn == mine)
                conn = null;
            if (mine == null || generation == gen) {
                try {
                    s.close();
                } catch (IOException ignored) {
                    // closed
                }
            } else {
                mine.closeAfterWrites();        // paused: the accepted frames go out first
            }
            if (generation != gen)
                break;
            if (!refused)
                setState(gen, R.string.term_reconnecting);
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

    private void pump(InputStream in, Conn c, Object gen) throws IOException {
        WsCodec.Parser p = new WsCodec.Parser();
        byte[] buf = new byte[8192];
        int n;
        while (generation == gen && (n = in.read(buf)) > 0) {
            for (WsCodec.Frame f : p.feed(buf, 0, n)) {
                if (f.opcode == WsCodec.BINARY || f.opcode == WsCodec.CONT) {
                    term.feed(f.data, 0, f.data.length);
                    dirty();
                } else if (f.opcode == WsCodec.PING) {
                    byte[] mask = new byte[4];
                    random.nextBytes(mask);
                    write(c, WsCodec.frame(WsCodec.PONG, f.data, 0, f.data.length, mask));
                } else if (f.opcode == WsCodec.CLOSE) {
                    return;
                }
            }
        }
    }
}
