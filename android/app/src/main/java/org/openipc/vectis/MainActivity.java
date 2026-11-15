/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * MainActivity.java — the status screen: the adapter, power (RTS+DTR) from
 * vectis' /status, the addresses with the console and web ports, Start /
 * Stop, USB access, the web console, the settings and Start on boot.
 * Built from platform views in code: works with a TV remote (D-pad); the
 * brass look of brand (Brass.java, res/values).
 *
 * The lamp and the display follow the brass state rule: green = powered,
 * red = power off, amber = waiting for the adapter or for access, dim =
 * stopped.
 */
package org.openipc.vectis;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.ActivityNotFoundException;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Typeface;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.text.InputType;
import android.view.Gravity;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Locale;
import org.json.JSONObject;

public class MainActivity extends Activity {
    private static final String ACTION_PERMISSION = "org.openipc.vectis.PERMISSION";
    private static final int REFRESH_MS = 1000;
    private static final int LOG_SHOWN = 8;

    private final Handler handler = new Handler(Looper.getMainLooper());
    private View lamp;
    private TextView display, logView;
    private Button start, stop, access, web;
    /** The last /status of vectis, read off the UI thread; null if none. */
    private volatile JSONObject status;
    private volatile boolean polling;

    private final BroadcastReceiver permission = new BroadcastReceiver() {
        @Override
        public void onReceive(Context c, Intent i) {
            if (i.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false))
                VectisService.send(c, VectisService.ACTION_START);
            update();
        }
    };

    private final Runnable refresh = new Runnable() {
        @Override
        public void run() {
            update();
            handler.postDelayed(this, REFRESH_MS);
        }
    };

    @Override
    protected void onCreate(Bundle state) {
        super.onCreate(state);
        int pad = Brass.dp(this, 16);
        LinearLayout page = new LinearLayout(this);
        page.setOrientation(LinearLayout.VERTICAL);
        page.setPadding(pad, pad, pad, pad);

        page.addView(Brass.title(this, getString(R.string.app_name)));
        TextView sub = new TextView(this);
        sub.setText(R.string.subtitle);
        sub.setTextColor(Brass.color(this, R.color.dim));
        sub.setGravity(Gravity.CENTER_HORIZONTAL);
        page.addView(sub);
        TextView plate = Brass.plate(this, "vectis " + version());
        LinearLayout.LayoutParams plp = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT);
        plp.gravity = Gravity.CENTER_HORIZONTAL;
        plp.setMargins(0, pad / 2, 0, pad);
        page.addView(plate, plp);

        LinearLayout col = Brass.panel(this);
        page.addView(col);

        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        row.setGravity(Gravity.CENTER_VERTICAL);
        lamp = new View(this);
        int ls = Brass.dp(this, 28);
        LinearLayout.LayoutParams llp = new LinearLayout.LayoutParams(ls, ls);
        llp.setMargins(Brass.dp(this, 6), 0, Brass.dp(this, 6), 0);
        row.addView(lamp, llp);
        display = Brass.display(this);
        display.setTextSize(15);
        row.addView(display, Brass.margins(this, 0, LinearLayout.LayoutParams.WRAP_CONTENT, 1));
        col.addView(row);

        start = button(col, R.string.btn_start, v -> {
            Settings.setRunning(this, true);
            VectisService.send(this, VectisService.ACTION_START);
        });
        stop = button(col, R.string.btn_stop, v -> {
            Settings.setRunning(this, false);
            VectisService.send(this, VectisService.ACTION_STOP);
        });
        access = button(col, R.string.btn_access, v -> askAccess());
        web = button(col, R.string.btn_web, v -> openWeb());
        button(col, R.string.btn_settings, v -> showSettings());

        CheckBox boot = new CheckBox(this);
        boot.setText(R.string.start_on_boot);
        boot.setBackgroundResource(R.drawable.focus_ring);
        boot.setChecked(Settings.startOnBoot(this));
        boot.setOnCheckedChangeListener((b, on) -> Settings.setStartOnBoot(this, on));
        col.addView(boot, Brass.margins(this, LinearLayout.LayoutParams.WRAP_CONTENT,
                LinearLayout.LayoutParams.WRAP_CONTENT, 0));

        logView = Brass.display(this);
        logView.setTextSize(11);
        LinearLayout.LayoutParams lgp = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
        lgp.setMargins(0, pad, 0, 0);
        page.addView(logView, lgp);

        ScrollView scroll = new ScrollView(this);
        scroll.addView(page);
        setContentView(scroll);
        if (Build.VERSION.SDK_INT >= 33)
            requestPermissions(new String[] {"android.permission.POST_NOTIFICATIONS"}, 1);
        // Registered for the life of the screen: the permission dialog pauses it
        register(new IntentFilter(ACTION_PERMISSION));
        if (Settings.running(this))
            VectisService.send(this, VectisService.ACTION_START);
    }

    @Override
    protected void onDestroy() {
        unregisterReceiver(permission);
        super.onDestroy();
    }

    private Button button(LinearLayout col, int text, View.OnClickListener l) {
        Button b = Brass.button(this, text);
        b.setOnClickListener(l);
        col.addView(b, Brass.margins(this, LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT, 0));
        return b;
    }

    @Override
    protected void onResume() {
        super.onResume();
        // Access granted meanwhile (the dialog, "use by default"): hand it to the service
        VectisService s = VectisService.instance;
        UsbDevice d = VectisService.findAdapter(this);
        if (s != null && d != null && !s.hasAccess() &&
                ((UsbManager) getSystemService(USB_SERVICE)).hasPermission(d))
            VectisService.send(this, VectisService.ACTION_START);
        polling = true;
        new Thread(this::poll, "vectis-status").start();
        handler.post(refresh);
    }

    @Override
    protected void onPause() {
        handler.removeCallbacks(refresh);
        polling = false;
        super.onPause();
    }

    /** The USB permission answer: our own broadcast, never from other apps. */
    @SuppressLint("UnspecifiedRegisterReceiverFlag")   // no flags before 13; the intent names our package
    private void register(IntentFilter f) {
        if (Build.VERSION.SDK_INT >= 33)
            registerReceiver(permission, f, Context.RECEIVER_NOT_EXPORTED);
        else
            registerReceiver(permission, f);
    }

    private String version() {
        try {
            return getPackageManager().getPackageInfo(getPackageName(), 0).versionName;
        } catch (Exception e) {
            return "?";
        }
    }

    private void askAccess() {
        UsbDevice d = VectisService.findAdapter(this);
        if (d == null)
            return;
        UsbManager um = (UsbManager) getSystemService(USB_SERVICE);
        um.requestPermission(d, Compat.usbPermission(this,
                new Intent(ACTION_PERMISSION).setPackage(getPackageName())));
    }

    private void openWeb() {
        String url = "http://127.0.0.1:" + Settings.webPort(this) + "/";
        try {
            startActivity(new Intent(Intent.ACTION_VIEW, Uri.parse(url)));
        } catch (ActivityNotFoundException e) {
            List<String> a = VectisService.addresses();
            String there = "http://" + (a.isEmpty() ? "127.0.0.1" : a.get(0)) + ":" + Settings.webPort(this) + "/";
            Toast.makeText(this, getString(R.string.no_browser, there), Toast.LENGTH_LONG).show();
        }
    }

    private EditText field(LinearLayout box, int label, int value) {
        TextView t = new TextView(this);
        t.setText(label);
        t.setTextColor(Brass.color(this, R.color.dim));
        box.addView(t);
        EditText e = new EditText(this);
        e.setInputType(InputType.TYPE_CLASS_NUMBER);
        e.setText(String.format(Locale.US, "%d", value));
        e.setTypeface(Typeface.MONOSPACE);
        box.addView(e);
        return e;
    }

    private void showSettings() {
        LinearLayout box = new LinearLayout(this);
        box.setOrientation(LinearLayout.VERTICAL);
        int pad = Brass.dp(this, 20);
        box.setPadding(pad, pad / 2, pad, 0);
        EditText baud = field(box, R.string.settings_baud, Settings.baud(this));
        EditText console = field(box, R.string.settings_console, Settings.consolePort(this));
        EditText webPort = field(box, R.string.settings_web, Settings.webPort(this));
        new AlertDialog.Builder(this)
                .setTitle(R.string.settings_title)
                .setView(box)
                .setPositiveButton(R.string.settings_save, (d, w) -> {
                    int b = number(baud), c = number(console), p = number(webPort);
                    if (b < 300 || b > 921600 || c < 1 || c > 65535 || p < 1 || p > 65535 || c == p) {
                        Toast.makeText(this, R.string.settings_bad, Toast.LENGTH_LONG).show();
                        return;
                    }
                    Settings.setPorts(this, b, c, p);
                    if (VectisService.instance != null)
                        VectisService.send(this, VectisService.ACTION_RESTART);
                })
                .setNegativeButton(R.string.settings_cancel, null)
                .show();
    }

    private static int number(EditText e) {
        try {
            return Integer.parseInt(e.getText().toString().trim());
        } catch (NumberFormatException x) {
            return -1;
        }
    }

    /** Reads vectis' /status once a second while the screen is shown. */
    private void poll() {
        while (polling) {
            JSONObject st = null;
            VectisService s = VectisService.instance;
            if (s != null && s.isRunning()) {
                try {
                    URL u = new URL("http://127.0.0.1:" + Settings.webPort(this) + "/status");
                    HttpURLConnection c = (HttpURLConnection) u.openConnection();
                    c.setConnectTimeout(800);
                    c.setReadTimeout(800);
                    try (InputStream in = c.getInputStream()) {
                        ByteArrayOutputStream b = new ByteArrayOutputStream();
                        byte[] buf = new byte[2048];
                        int n;
                        while ((n = in.read(buf)) > 0 && b.size() < 65536)
                            b.write(buf, 0, n);
                        st = new JSONObject(new String(b.toByteArray(), StandardCharsets.UTF_8));
                    } finally {
                        c.disconnect();
                    }
                } catch (Exception ignored) {
                    // vectis not listening yet
                }
            }
            status = st;
            try {
                Thread.sleep(REFRESH_MS);
            } catch (InterruptedException e) {
                return;
            }
        }
    }

    private void update() {
        VectisService s = VectisService.instance;
        UsbDevice d = s != null ? s.adapter() : VectisService.findAdapter(this);
        JSONObject st = status;
        StringBuilder t = new StringBuilder();
        int color;

        String adapter = d == null ? getString(R.string.adapter_none)
                : s != null && s.hasAccess() ? getString(R.string.adapter_ok, VectisService.describe(d))
                : getString(R.string.adapter_no_access, VectisService.describe(d));
        line(t, R.string.label_adapter, adapter);

        String power;
        if (s == null) {
            power = getString(R.string.stopped);
            color = R.color.dim;
        } else if (!s.isRunning()) {
            power = getString(R.string.restarting, s.lastExit());
            color = R.color.red;
        } else if (st == null || !"online".equals(st.optString("link"))) {
            power = getString(R.string.power_unknown);
            color = R.color.amber;
        } else {
            boolean on = "on".equals(st.optString("power"));
            int rts = st.optInt("rts", -1), dtr = st.optInt("dtr", -1);
            power = getString(on ? R.string.power_on : R.string.power_off, rts, dtr);
            color = on ? R.color.green : R.color.red;
        }
        line(t, R.string.label_power, power);

        List<String> a = VectisService.addresses();
        String host = a.isEmpty() ? null : a.get(0);
        int cp = Settings.consolePort(this), wp = Settings.webPort(this);
        StringBuilder con = new StringBuilder(), webs = new StringBuilder();
        for (String ip : a) {
            con.append(con.length() > 0 ? "\n           " : "").append(ip).append(':').append(cp);
            webs.append(webs.length() > 0 ? "\n           " : "").append("http://").append(ip).append(':').append(wp).append('/');
        }
        line(t, R.string.label_console, host == null ? getString(R.string.no_address) : con.toString());
        line(t, R.string.label_web, host == null ? getString(R.string.no_address) : webs.toString());

        display.setText(t.toString().trim());
        display.setTextColor(Brass.color(this, color == R.color.dim ? R.color.dim : R.color.ink));
        lamp.setBackground(Brass.lamp(this, color));
        start.setEnabled(s == null);
        stop.setEnabled(s != null);
        access.setVisibility(d != null && (s == null || !s.hasAccess()) ? View.VISIBLE : View.GONE);
        web.setEnabled(s != null && s.isRunning());
        logView.setText(s != null ? s.log.tail(LOG_SHOWN).trim() : "");
        logView.setVisibility(s != null ? View.VISIBLE : View.GONE);
    }

    private void line(StringBuilder t, int label, String value) {
        String l = getString(label);
        t.append(l);
        for (int i = l.length(); i < 10; i++)
            t.append(' ');
        t.append(' ').append(value).append('\n');
    }
}
