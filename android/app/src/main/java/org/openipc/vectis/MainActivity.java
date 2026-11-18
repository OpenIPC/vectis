/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * MainActivity.java — the status screen: the adapter, power (RTS+DTR) from
 * vectis' /status, the addresses with the console and web ports and whether
 * the web page has a password, Start / Stop, USB access, the terminal
 * (TerminalActivity), the web console, the settings and Start on boot; the
 * camex tunnel (#97): its state, the VPN consent, and a camex link opened
 * with the app, which fills the tunnel settings.
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
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import org.json.JSONObject;
import org.openipc.camex.tunnel.CamexTunnel;
import org.openipc.camex.tunnel.ConnectLink;
import org.openipc.camex.tunnel.Field;

public class MainActivity extends Activity {
    private static final String ACTION_PERMISSION = "org.openipc.vectis.PERMISSION";
    private static final int REFRESH_MS = 1000;
    private static final int LOG_SHOWN = 8;
    private static final int ASK_VPN = 2;

    private final Handler handler = new Handler(Looper.getMainLooper());
    private View lamp;
    private TextView display, logView;
    private Button start, stop, access, consent, terminal, web;
    /** The last /status of vectis, read off the UI thread; null if none. */
    private volatile JSONObject status;
    private volatile Object pollToken;

    private final BroadcastReceiver permission = new BroadcastReceiver() {
        @Override
        public void onReceive(Context c, Intent i) {
            // Not the extra (any app could send this action before 13): what UsbManager says
            if (Settings.running(c) && hasAccess(c))
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
        consent = button(col, R.string.btn_consent, v -> askConsent());
        terminal = button(col, R.string.btn_terminal,
                v -> startActivity(new Intent(this, TerminalActivity.class)));
        web = button(col, R.string.btn_web, v -> openWeb());
        button(col, R.string.btn_settings, v -> showSettings(null));

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
        if (state == null)
            openLink(getIntent());
    }

    @Override
    protected void onNewIntent(Intent i) {
        super.onNewIntent(i);
        openLink(i);
    }

    /** A camex link the app was opened with: the settings, the tunnel filled from it. */
    private void openLink(Intent i) {
        if (i != null && Intent.ACTION_VIEW.equals(i.getAction()) && i.getData() != null)
            showSettings(i.getData().toString());
    }

    /** The system's VPN dialog, once; the service starts the tunnel after OK. */
    private void askConsent() {
        Intent i = CamexTunnel.consentIntent(this);
        if (i == null) {
            restartService(false);
            return;
        }
        try {
            startActivityForResult(i, ASK_VPN);
        } catch (ActivityNotFoundException e) {
            Toast.makeText(this, R.string.tunnel_consent_refused, Toast.LENGTH_LONG).show();
        }
    }

    @Override
    protected void onActivityResult(int request, int result, Intent data) {
        super.onActivityResult(request, result, data);
        if (request != ASK_VPN)
            return;
        if (result == RESULT_OK)
            restartService(false);
        else
            Toast.makeText(this, R.string.tunnel_consent_refused, Toast.LENGTH_LONG).show();
        update();
    }

    /** The new settings to the service: vectis restarted if its own changed, the tunnel either way. */
    private void restartService(boolean vectis) {
        if (VectisService.instance == null)
            return;                       // stopped: Start takes the settings
        VectisService.send(this, vectis ? VectisService.ACTION_RESTART : VectisService.ACTION_START);
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
        if (s != null && !s.hasAccess() && Settings.running(this) && hasAccess(this))
            VectisService.send(this, VectisService.ACTION_START);
        // One poller per resume: an older one (still sleeping) sees its token replaced and ends
        Object token = new Object();
        pollToken = token;
        new Thread(() -> poll(token), "vectis-status").start();
        handler.post(refresh);
    }

    @Override
    protected void onPause() {
        handler.removeCallbacks(refresh);
        pollToken = null;
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

    /** True if a supported adapter is plugged in and UsbManager lets us open it. */
    static boolean hasAccess(Context c) {
        UsbDevice d = VectisService.findAdapter(c);
        return d != null && ((UsbManager) c.getSystemService(USB_SERVICE)).hasPermission(d);
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
        EditText e = field(box, label, String.format(Locale.US, "%d", value));
        e.setInputType(InputType.TYPE_CLASS_NUMBER);
        return e;
    }

    private EditText field(LinearLayout box, int label, String value) {
        TextView t = new TextView(this);
        t.setText(label);
        t.setTextColor(Brass.color(this, R.color.dim));
        box.addView(t);
        EditText e = new EditText(this);
        e.setText(value);
        e.setTypeface(Typeface.MONOSPACE);
        box.addView(e);
        return e;
    }

    private TextView heading(LinearLayout box, int text) {
        TextView t = new TextView(this);
        t.setText(text);
        t.setTextColor(Brass.color(this, R.color.ink));
        t.setTypeface(Typeface.DEFAULT_BOLD);
        t.setPadding(0, Brass.dp(this, 12), 0, 0);
        box.addView(t);
        return t;
    }

    /** The settings; a camex link (or null) fills the tunnel's fields. */
    private void showSettings(String link) {
        LinearLayout box = new LinearLayout(this);
        box.setOrientation(LinearLayout.VERTICAL);
        int pad = Brass.dp(this, 20);
        box.setPadding(pad, pad / 2, pad, 0);
        heading(box, R.string.settings_serial);
        EditText baud = field(box, R.string.settings_baud, Settings.baud(this));

        heading(box, R.string.settings_network);
        EditText console = field(box, R.string.settings_console, Settings.consolePort(this));
        EditText webPort = field(box, R.string.settings_web, Settings.webPort(this));
        // The camex tunnel: on or off, a link that fills it, the fields of the tunnel library
        CheckBox tunnelOn = new CheckBox(this);
        tunnelOn.setText(R.string.tunnel_on);
        tunnelOn.setBackgroundResource(R.drawable.focus_ring);
        tunnelOn.setChecked(Settings.tunnelEnabled(this));
        box.addView(tunnelOn);
        TextView note = new TextView(this);
        note.setText(R.string.tunnel_note);
        note.setTextColor(Brass.color(this, R.color.dim));
        note.setTextSize(12);
        box.addView(note);
        EditText linkField = new EditText(this);
        linkField.setHint(R.string.tunnel_link_hint);
        linkField.setSingleLine(true);
        linkField.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_URI);
        box.addView(linkField);
        Button apply = Brass.button(this, R.string.tunnel_link_apply);
        box.addView(apply);
        TextView linkErrors = new TextView(this);
        linkErrors.setTextColor(Brass.color(this, R.color.red));
        linkErrors.setVisibility(View.GONE);
        box.addView(linkErrors);
        Map<String, String> saved = Settings.tunnel(this);
        Map<String, EditText> texts = new LinkedHashMap<>();
        Map<String, RadioGroup> choices = new HashMap<>();
        for (Field f : CamexTunnel.fields()) {
            if (CamexTunnel.WEB_PORT.equals(f.key) || f.kind == Field.Kind.FLAG)
                continue;                 // the web port is the one above
            TextView label = new TextView(this);
            label.setText(f.required ? f.label : getString(R.string.tunnel_optional, f.label));
            label.setTextColor(Brass.color(this, R.color.dim));
            box.addView(label);
            String value = saved.get(f.key) != null && !saved.get(f.key).isEmpty() ? saved.get(f.key) : f.initial;
            if (f.kind == Field.Kind.CHOICE) {
                RadioGroup g = new RadioGroup(this);
                g.setOrientation(RadioGroup.HORIZONTAL);
                for (String c : f.choices) {
                    RadioButton r = new RadioButton(this);
                    r.setText(c);
                    r.setId(View.generateViewId());
                    g.addView(r);
                    if (c.equals(value))
                        g.check(r.getId());
                }
                choices.put(f.key, g);
                box.addView(g);
            } else {
                EditText e = new EditText(this);
                e.setSingleLine(true);
                e.setTypeface(Typeface.MONOSPACE);
                // shown, not dotted: typed with a TV remote, read back before Save
                e.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_VISIBLE_PASSWORD);
                e.setText(value == null ? "" : value);
                texts.put(f.key, e);
                box.addView(e);
            }
        }
        // a link is asking for the tunnel: it turns it on
        if (link != null && fill(link, texts, choices, linkErrors))
            tunnelOn.setChecked(true);
        apply.setOnClickListener(v -> {
            if (fill(linkField.getText().toString().trim(), texts, choices, linkErrors))
                tunnelOn.setChecked(true);
        });

        // Last, as in Decoder and Microsnander; shown as typed: a lab tool, set with a remote
        EditText password = field(box, R.string.settings_password, Settings.password(this));
        password.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_VISIBLE_PASSWORD);
        password.setHint(R.string.settings_password_hint);

        ScrollView sv = new ScrollView(this);
        sv.addView(box);
        AlertDialog d = new AlertDialog.Builder(this)
                .setTitle(R.string.settings_title)
                .setView(sv)
                .setPositiveButton(R.string.settings_save, null)
                .setNegativeButton(R.string.settings_cancel, null)
                .show();
        // Save checks the fields first: the dialog stays with what is wrong
        d.getButton(AlertDialog.BUTTON_POSITIVE).setOnClickListener(v -> {
            int b = number(baud), c = number(console), p = number(webPort);
            if (!Settings.valid(b, c, p)) {
                Toast.makeText(this, R.string.settings_bad, Toast.LENGTH_LONG).show();
                return;
            }
            String pw = password.getText().toString();
            if (!Settings.validPassword(pw)) {
                Toast.makeText(this, R.string.settings_password_long, Toast.LENGTH_LONG).show();
                password.requestFocus();
                return;
            }
            Map<String, String> tv = new HashMap<>();
            for (Map.Entry<String, EditText> e : texts.entrySet())
                tv.put(e.getKey(), e.getValue().getText().toString().trim());
            for (Map.Entry<String, RadioGroup> e : choices.entrySet()) {
                View r = e.getValue().findViewById(e.getValue().getCheckedRadioButtonId());
                tv.put(e.getKey(), r instanceof RadioButton ? ((RadioButton) r).getText().toString() : "");
            }
            if (tunnelOn.isChecked()) {
                tv.put(CamexTunnel.WEB_PORT, String.valueOf(p));
                String[] bad = CamexTunnel.check(tv);
                tv.remove(CamexTunnel.WEB_PORT);
                if (bad != null) {
                    EditText e = texts.get(bad[0]);
                    if (e != null) {
                        e.setError(bad[1]);
                        e.requestFocus();
                    } else {
                        linkErrors.setText(getString(R.string.tunnel_link_field, bad[0], bad[1]));
                        linkErrors.setVisibility(View.VISIBLE);
                    }
                    return;
                }
            }
            boolean vectis = b != Settings.baud(this) || c != Settings.consolePort(this)
                    || p != Settings.webPort(this) || !pw.equals(Settings.password(this));
            Settings.setPorts(this, b, c, p);
            Settings.setPassword(this, pw);
            Settings.setTunnel(this, tunnelOn.isChecked(), tv);
            d.dismiss();
            if (tunnelOn.isChecked() && CamexTunnel.consentIntent(this) != null) {
                if (vectis)
                    restartService(true);
                askConsent();             // the tunnel starts after the answer
            } else {
                restartService(vectis);
            }
            update();
        });
    }

    /** The values of a camex link into the fields; its malformed fields are listed. False: not a link. */
    private boolean fill(String link, Map<String, EditText> texts, Map<String, RadioGroup> choices, TextView errors) {
        ConnectLink l = ConnectLink.parse(link);
        if (l == null) {
            errors.setText(R.string.tunnel_link_bad);
            errors.setVisibility(View.VISIBLE);
            return false;
        }
        for (Map.Entry<String, String> e : l.values.entrySet()) {
            EditText t = texts.get(e.getKey());
            if (t != null)
                t.setText(e.getValue());
            RadioGroup g = choices.get(e.getKey());
            for (int i = 0; g != null && i < g.getChildCount(); i++) {
                RadioButton r = (RadioButton) g.getChildAt(i);
                if (r.getText().toString().equalsIgnoreCase(e.getValue()))
                    g.check(r.getId());
            }
        }
        StringBuilder m = new StringBuilder();
        for (Map.Entry<String, String> e : l.errors.entrySet())
            m.append(getString(R.string.tunnel_link_field, e.getKey(), e.getValue())).append('\n');
        errors.setText(m.toString().trim());
        errors.setVisibility(m.length() > 0 ? View.VISIBLE : View.GONE);
        return true;
    }

    private static int number(EditText e) {
        try {
            return Integer.parseInt(e.getText().toString().trim());
        } catch (NumberFormatException x) {
            return -1;
        }
    }

    /** Reads vectis' /status once a second while the screen is shown. */
    private void poll(Object token) {
        while (pollToken == token) {
            JSONObject st = null;
            VectisService s = VectisService.instance;
            if (s != null && s.isRunning()) {
                try {
                    st = Http.json(Settings.webPort(this), "/status", Settings.password(this));
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
        } else if (!s.isRunning() && s.lastExit().isEmpty()) {
            power = getString(R.string.starting);
            color = R.color.amber;
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
        line(t, R.string.label_page, getString(Settings.password(this).isEmpty()
                ? R.string.web_open : R.string.web_protected));
        String tunnel = s != null ? VectisService.tunnelStatus : "";
        if (!tunnel.isEmpty())
            line(t, R.string.label_tunnel, tunnel);

        display.setText(t.toString().trim());
        display.setTextColor(Brass.color(this, color == R.color.dim ? R.color.dim : R.color.ink));
        lamp.setBackground(Brass.lamp(this, color));
        start.setEnabled(s == null || !s.isWanted());
        stop.setEnabled(s != null);
        access.setVisibility(d != null && (s == null || !s.hasAccess()) ? View.VISIBLE : View.GONE);
        consent.setVisibility(s != null && Settings.tunnelEnabled(this)
                && CamexTunnel.state() == CamexTunnel.State.NEEDS_CONSENT ? View.VISIBLE : View.GONE);
        web.setEnabled(s != null && s.isRunning());
        terminal.setEnabled(s != null && s.isRunning());
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
