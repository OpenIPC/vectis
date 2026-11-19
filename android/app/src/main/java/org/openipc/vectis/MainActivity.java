/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * MainActivity.java — the status screen: a lamp board (ADAPTER, POWER from
 * vectis' /status, VECTIS, the CONSOLE and WEB addresses with a lock for the
 * page password, CAMEX when the tunnel is on), Start / Stop, Terminal
 * (TerminalActivity), Settings, USB access and the VPN consent when needed,
 * Start on boot and Show log; the settings window (VECTIS: remote access,
 * ports, the page password; CAMEX: the tunnel, #97) and a camex link opened
 * with the app, which fills the tunnel settings.
 * Built from platform views in code: works with a TV remote (D-pad); the
 * brass look of brand 1.3.0 (Brass.java, res/values).
 *
 * Each row has its own lamp, by the brass state rule: green = works, amber =
 * waiting (starting, no access, connecting), red = an error only, dim =
 * stopped or off.  The port rows: green = serving, dim = OFF, red = FAILED.
 */
package org.openipc.vectis;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.Dialog;
import android.content.ActivityNotFoundException;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Typeface;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
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
    /** The log window: this many lines high (brass 1.3.1), the whole buffer inside. */
    private static final int LOG_ROWS = 12;
    private static final int ASK_VPN = 2;

    private final Handler handler = new Handler(Looper.getMainLooper());
    private Brass.Row rAdapter, rPower, rVectis, rConsole, rWeb, rCamex;
    private LinearLayout accessRow;
    private Brass.LogWindow logWindow;
    private Button start, stop, access, consent, terminal;
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

        page.addView(Brass.titleRow(this, getString(R.string.app_name), version()));
        TextView sub = new TextView(this);
        sub.setText(R.string.subtitle);
        sub.setTextColor(Brass.color(this, R.color.dim));
        sub.setGravity(Gravity.CENTER_HORIZONTAL);
        LinearLayout.LayoutParams sp = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
        sp.setMargins(0, 0, 0, pad);
        page.addView(sub, sp);

        LinearLayout col = Brass.panel(this);
        page.addView(col);

        LinearLayout board = new LinearLayout(this);
        board.setOrientation(LinearLayout.VERTICAL);
        board.setBackgroundResource(R.drawable.display);
        int bp = Brass.dp(this, 12);
        board.setPadding(bp, bp - Brass.dp(this, 3), bp, bp - Brass.dp(this, 3));
        rAdapter = new Brass.Row(this, getString(R.string.label_adapter));
        rPower = new Brass.Row(this, getString(R.string.label_power));
        rVectis = new Brass.Row(this, getString(R.string.label_vectis));
        rConsole = new Brass.Row(this, getString(R.string.label_console));
        rWeb = new Brass.Row(this, getString(R.string.label_web));
        rCamex = new Brass.Row(this, getString(R.string.label_camex));
        for (Brass.Row r : new Brass.Row[] {rAdapter, rPower, rVectis, rConsole, rWeb, rCamex})
            board.addView(r.view);
        rWeb.value.setCompoundDrawablePadding(Brass.dp(this, 10));
        LinearLayout.LayoutParams bl = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
        bl.setMargins(Brass.dp(this, 6), 0, Brass.dp(this, 6), Brass.dp(this, 6));
        col.addView(board, bl);

        // Buttons two in a row; the access row only when one of its buttons is needed
        LinearLayout r1 = row(col), r2 = row(col);
        accessRow = row(col);
        start = button(r1, R.string.btn_start, v -> {
            Settings.setRunning(this, true);
            VectisService.send(this, VectisService.ACTION_START);
        });
        stop = button(r1, R.string.btn_stop, v -> {
            Settings.setRunning(this, false);
            VectisService.send(this, VectisService.ACTION_STOP);
        });
        terminal = button(r2, R.string.btn_terminal,
                v -> startActivity(new Intent(this, TerminalActivity.class)));
        button(r2, R.string.btn_settings, v -> showSettings(null));
        access = button(accessRow, R.string.btn_access, v -> askAccess());
        consent = button(accessRow, R.string.btn_consent, v -> askConsent());

        // Check boxes in one row, as wide as their text
        LinearLayout checks = new LinearLayout(this);
        checks.setOrientation(LinearLayout.HORIZONTAL);
        CheckBox boot = new CheckBox(this);
        boot.setText(R.string.start_on_boot);
        Brass.check(boot);
        boot.setChecked(Settings.startOnBoot(this));
        boot.setOnCheckedChangeListener((b, on) -> Settings.setStartOnBoot(this, on));
        checks.addView(boot, Brass.margins(this, LinearLayout.LayoutParams.WRAP_CONTENT,
                LinearLayout.LayoutParams.WRAP_CONTENT, 0));
        CheckBox showLog = new CheckBox(this);
        showLog.setText(R.string.show_log);
        Brass.check(showLog);
        showLog.setChecked(Settings.showLog(this));
        showLog.setOnCheckedChangeListener((b, on) -> {
            Settings.setShowLog(this, on);
            update();
            if (on)
                logWindow.shown();        // opened: at the end, however long the log is
        });
        checks.addView(showLog, Brass.margins(this, LinearLayout.LayoutParams.WRAP_CONTENT,
                LinearLayout.LayoutParams.WRAP_CONTENT, 0));
        col.addView(checks);

        // The log (Show log, brass 1.3.1): Brass.LogWindow, as in pusher
        logWindow = new Brass.LogWindow(this, LOG_ROWS);
        LinearLayout.LayoutParams lgp = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, logWindow.height);
        lgp.setMargins(0, pad, 0, 0);
        page.addView(logWindow.view, lgp);

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

    /**
     * A camex link the app was opened with: the settings, the tunnel filled from it. Used once:
     * not when the app is reopened from recents with the intent that started it, and cleared.
     */
    private void openLink(Intent i) {
        if (i == null || !Intent.ACTION_VIEW.equals(i.getAction()) || i.getData() == null)
            return;
        String link = linkToOpen(i.getAction(), i.getData().toString(), i.getFlags());
        setIntent(new Intent(this, MainActivity.class));
        if (link != null)
            showSettings(link);
    }

    /** The link of a VIEW intent to open, or null: none, or reopened from recents. */
    static String linkToOpen(String action, String data, int flags) {
        if (!Intent.ACTION_VIEW.equals(action) || data == null || data.isEmpty())
            return null;
        return (flags & Intent.FLAG_ACTIVITY_LAUNCHED_FROM_HISTORY) != 0 ? null : data;
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

    /** A brass button sharing its row with the others, all the same width. */
    private Button button(LinearLayout row, int text, View.OnClickListener l) {
        Button b = Brass.button(this, text);
        b.setOnClickListener(l);
        row.addView(b, Brass.margins(this, 0, LinearLayout.LayoutParams.WRAP_CONTENT, 1));
        return b;
    }

    /** A row for two brass buttons across the full width. */
    private LinearLayout row(LinearLayout col) {
        LinearLayout r = new LinearLayout(this);
        r.setOrientation(LinearLayout.HORIZONTAL);
        col.addView(r, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT));
        return r;
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

    /** The settings; a camex link (or null) fills the tunnel's fields. */
    private void showSettings(String link) {
        LinearLayout box = new LinearLayout(this);
        box.setOrientation(LinearLayout.VERTICAL);
        int pad = Brass.dp(this, 20);
        box.setPadding(pad, pad / 2, pad, 0);
        // VECTIS: the app's own settings; remote access first, the page password last
        Brass.section(box, this, getString(R.string.section_vectis), false);
        CheckBox remoteOn = new CheckBox(this);
        remoteOn.setText(R.string.remote_access);
        remoteOn.setBackgroundResource(R.drawable.focus_ring);
        remoteOn.setChecked(Settings.remoteAccess(this));
        box.addView(remoteOn);
        EditText baud = field(box, R.string.settings_baud, Settings.baud(this));
        EditText console = field(box, R.string.settings_console, Settings.consolePort(this));
        EditText webPort = field(box, R.string.settings_web, Settings.webPort(this));
        // shown as typed: a lab tool, set with a remote
        EditText password = field(box, R.string.settings_password, Settings.password(this));
        password.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_VISIBLE_PASSWORD);
        password.setHint(R.string.settings_password_hint);

        // CAMEX: the tunnel on or off and the fields of the tunnel library; a camex link
        // opened with the app fills them (no link field here)
        Brass.section(box, this, getString(R.string.section_camex), true);
        CheckBox tunnelOn = new CheckBox(this);
        tunnelOn.setText(R.string.tunnel_on);
        tunnelOn.setBackgroundResource(R.drawable.focus_ring);
        tunnelOn.setChecked(Settings.tunnelEnabled(this));
        box.addView(tunnelOn);
        TextView linkErrors = new TextView(this);
        linkErrors.setTextColor(Brass.color(this, R.color.red));
        linkErrors.setVisibility(View.GONE);
        box.addView(linkErrors);
        Map<String, String> saved = Settings.tunnel(this);
        Map<String, EditText> texts = new LinkedHashMap<>();
        Map<String, RadioGroup> choices = new HashMap<>();
        // Order, labels, kinds, hints and the default client name: the library's, as in every app
        for (Field f : CamexTunnel.fields(this, Settings.APP)) {
            if (CamexTunnel.WEB_PORT.equals(f.key) || f.kind == Field.Kind.FLAG)
                continue;                 // the web port is the one above
            TextView label = new TextView(this);
            label.setText(f.label);
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
                boolean secret = f.kind == Field.Kind.SECRET;
                // the key hidden, with Show the key under it; the rest shown as typed
                e.setInputType(InputType.TYPE_CLASS_TEXT | (secret ? InputType.TYPE_TEXT_VARIATION_PASSWORD
                        : InputType.TYPE_TEXT_VARIATION_VISIBLE_PASSWORD));
                e.setTypeface(Typeface.MONOSPACE);   // setInputType resets it for a password
                if (!f.hint.isEmpty())
                    e.setHint(f.hint);
                e.setText(value == null ? "" : value);
                texts.put(f.key, e);
                box.addView(e);
                if (secret) {
                    CheckBox show = new CheckBox(this);
                    show.setText(Field.SHOW_SECRET);
                    show.setBackgroundResource(R.drawable.focus_ring);
                    show.setOnCheckedChangeListener((b, on) -> {
                        int at = e.getSelectionEnd();
                        e.setInputType(InputType.TYPE_CLASS_TEXT | (on ? InputType.TYPE_TEXT_VARIATION_VISIBLE_PASSWORD
                                : InputType.TYPE_TEXT_VARIATION_PASSWORD));
                        e.setTypeface(Typeface.MONOSPACE);
                        e.setSelection(Math.max(0, Math.min(at, e.length())));
                    });
                    box.addView(show);
                }
            }
        }
        // a link is asking for the tunnel: it turns it on
        if (link != null && fill(link, texts, choices, linkErrors))
            tunnelOn.setChecked(true);

        // One brass window: the panel with its frame, a serif title, brass Cancel and Save
        LinearLayout frame = Brass.panel(this);
        TextView title = Brass.title(this, getString(R.string.settings_title));
        title.setTextSize(20);
        frame.addView(title);
        ScrollView sv = new ScrollView(this);
        sv.addView(box);
        frame.addView(sv, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, 0, 1));
        LinearLayout actions = row(frame);
        Button cancel = button(actions, R.string.settings_cancel, null);
        Button save = button(actions, R.string.settings_save, null);
        Dialog d = new Dialog(this, android.R.style.Theme_Material_Dialog_NoActionBar);
        d.setContentView(frame);
        if (d.getWindow() != null) {
            d.getWindow().setBackgroundDrawable(new android.graphics.drawable.ColorDrawable(0));
            android.util.DisplayMetrics dm = getResources().getDisplayMetrics();
            d.getWindow().setLayout(Math.min((int) (dm.widthPixels * 0.94), Brass.dp(this, 640)),
                    (int) (dm.heightPixels * 0.9));
        }
        styleFields(box);
        d.show();
        cancel.setOnClickListener(v -> d.dismiss());
        // Save checks the fields first: the window stays with what is wrong
        save.setOnClickListener(v -> {
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
            String name = tv.get(CamexTunnel.NAME);
            if (name == null || name.isEmpty())   // emptied: back to the default name
                tv.put(CamexTunnel.NAME, CamexTunnel.defaultName(this, Settings.APP));
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
            boolean remote = remoteOn.isChecked();
            boolean vectis = b != Settings.baud(this) || c != Settings.consolePort(this)
                    || p != Settings.webPort(this) || !pw.equals(Settings.password(this))
                    || remote != Settings.remoteAccess(this);
            Settings.setPorts(this, b, c, p);
            Settings.setPassword(this, pw);
            Settings.setRemoteAccess(this, remote);
            Settings.setTunnel(this, tunnelOn.isChecked(), tv);
            d.dismiss();
            // The service first, either way: without the consent it records that and retries,
            // so a declined dialog still leaves the TUNNEL line, the button and the retries
            restartService(vectis);
            if (tunnelOn.isChecked() && CamexTunnel.consentIntent(this) != null)
                askConsent();             // the tunnel starts after the answer
            update();
        });
    }

    /**
     * The values of a camex link into the fields. A link with malformed fields changes nothing:
     * they are listed and it is false, as for no link at all.
     */
    private boolean fill(String link, Map<String, EditText> texts, Map<String, RadioGroup> choices, TextView errors) {
        ConnectLink l = ConnectLink.parse(link);
        if (l == null) {
            errors.setText(R.string.tunnel_link_bad);
            errors.setVisibility(View.VISIBLE);
            return false;
        }
        if (!l.errors.isEmpty()) {
            StringBuilder m = new StringBuilder();
            for (Map.Entry<String, String> e : l.errors.entrySet())
                m.append(getString(R.string.tunnel_link_field, e.getKey(), e.getValue())).append('\n');
            errors.setText(m.toString().trim());
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
        errors.setVisibility(View.GONE);
        return true;
    }

    /** Fields, labels and check boxes of the settings window in the brass palette. */
    private void styleFields(android.view.ViewGroup g) {
        android.content.res.ColorStateList amber =
                android.content.res.ColorStateList.valueOf(Brass.color(this, R.color.amber));
        for (int i = 0; i < g.getChildCount(); i++) {
            View v = g.getChildAt(i);
            if (v instanceof EditText) {
                EditText e = (EditText) v;
                e.setTextColor(Brass.color(this, R.color.ink));
                e.setHintTextColor(0x99b09873);   // dim at 60 %
                e.setBackgroundTintList(amber);
            } else if (v instanceof CheckBox || v instanceof RadioButton) {
                ((TextView) v).setTextColor(Brass.color(this, R.color.ink));
                Brass.check((android.widget.CompoundButton) v);
            } else if (v instanceof android.view.ViewGroup) {
                styleFields((android.view.ViewGroup) v);
            }
        }
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
                    st = Http.json("/status");
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
        boolean running = s != null && s.isRunning();

        if (d == null)
            rAdapter.set(R.color.dim, getString(R.string.adapter_none));
        else if (s != null && s.hasAccess())
            rAdapter.set(R.color.green, VectisService.chip(d) + " · " + Settings.baud(this));
        else
            rAdapter.set(R.color.amber, getString(R.string.adapter_no_access, VectisService.chip(d)));

        if (s == null)
            rPower.set(R.color.dim, getString(R.string.dash));
        else if (!running || st == null || !"online".equals(st.optString("link")))
            rPower.set(R.color.amber, getString(R.string.power_unknown));
        else {
            boolean on = "on".equals(st.optString("power"));
            rPower.set(on ? R.color.green : R.color.dim, getString(on ? R.string.power_on : R.string.power_off,
                    st.optInt("rts", -1), st.optInt("dtr", -1)));
        }

        boolean failed = s != null && !running && !s.lastExit().isEmpty();
        if (s == null)
            rVectis.set(R.color.dim, getString(R.string.stopped));
        else if (failed)
            rVectis.set(R.color.red, getString(R.string.restarting, s.lastExit()));
        else if (!running)
            rVectis.set(R.color.amber, getString(R.string.starting));
        else
            rVectis.set(R.color.green, getString(R.string.running));

        // The port rows: green serving, dim OFF (stopped, remote access off), red FAILED
        boolean remote = Settings.remoteAccess(this);
        List<String> a = VectisService.addresses();
        String host = a.isEmpty() ? null : a.get(0);
        port(rConsole, remote, running, failed, host, Settings.consolePort(this));
        port(rWeb, remote, running, failed, host, VectisService.webPort(this));
        rWeb.value.setCompoundDrawablesWithIntrinsicBounds(0, 0, !remote ? 0
                : Settings.password(this).isEmpty() ? R.drawable.lock_open : R.drawable.lock_closed, 0);

        boolean tun = Settings.tunnelEnabled(this);
        rCamex.view.setVisibility(tun ? View.VISIBLE : View.GONE);
        if (tun) {
            CamexTunnel.State ts = s != null ? VectisService.tunnelNow : CamexTunnel.State.STOPPED;
            int c = ts == CamexTunnel.State.CONNECTED ? R.color.green : ts == CamexTunnel.State.FAILED ? R.color.red
                    : ts == CamexTunnel.State.STOPPED ? R.color.dim : R.color.amber;
            String addr = VectisService.tunnelAddress;
            rCamex.set(c, ts.name().replace('_', ' ')
                    + (ts == CamexTunnel.State.CONNECTED && !addr.isEmpty() ? " · " + addr : ""));
        }

        start.setEnabled(s == null || !s.isWanted());
        stop.setEnabled(s != null);
        access.setVisibility(d != null && (s == null || !s.hasAccess()) ? View.VISIBLE : View.GONE);
        consent.setVisibility(s != null && tun
                && CamexTunnel.state() == CamexTunnel.State.NEEDS_CONSENT ? View.VISIBLE : View.GONE);
        accessRow.setVisibility(access.getVisibility() == View.VISIBLE || consent.getVisibility() == View.VISIBLE
                ? View.VISIBLE : View.GONE);
        terminal.setEnabled(running);
        LogBuffer.Snapshot log = s != null ? s.log.snapshot() : null;
        logWindow.setText(log != null ? log.text.trim() : "", log != null ? log.removed : 0);
        logWindow.view.setVisibility(s != null && Settings.showLog(this) ? View.VISIBLE : View.GONE);
    }

    /** A port row: the address when vectis serves it, OFF or FAILED otherwise. */
    private void port(Brass.Row r, boolean remote, boolean running, boolean failed, String host, int port) {
        if (!remote || (!running && !failed))
            r.set(R.color.dim, getString(R.string.port_off));
        else if (failed)
            r.set(R.color.red, getString(R.string.port_failed));
        else
            r.set(R.color.green, host == null ? getString(R.string.no_address) : host + ":" + port);
    }
}
