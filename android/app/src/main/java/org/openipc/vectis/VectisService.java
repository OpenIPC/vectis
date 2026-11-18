/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * VectisService.java — foreground service that owns the USB-UART adapter
 * and runs vectis on it
 *
 * Notes:
 *   - vectis is the C program of the repository root, built as an
 *     executable named libvectis.so (build-native.sh) and installed in
 *     nativeLibraryDir; it runs here as a child process:
 *     vectis -U fd:@org.openipc.vectis.usb -b BAUD -l PORT -w PORT, with
 *     the web page password, if one is set, in VECTIS_WEB_PASSWORD (not on
 *     the command line, where ps would show it).
 *   - Only root can open /dev/bus/usb, so the service opens the adapter with
 *     UsbManager and serves a usbfs fd on the abstract socket of -U: every
 *     connection from our own uid gets the adapter opened anew (vectis
 *     connects again on each re-open), never a copy of an fd another vectis
 *     used, whose leftover URBs would point into that old process.  Without
 *     access the connection is closed and vectis keeps retrying quietly.
 *     Other uids get nothing: the fd controls the adapter.
 *   - Unplugging closes the adapter; plugging it back in reopens it, and
 *     vectis picks it up on its next retry: vectis itself keeps running, so
 *     the TCP ports stay up.  vectis exiting (a crash) is restarted after 3 s.
 *   - RTS/DTR are left as they are when vectis or the service stops, as with
 *     -U on Linux: the target keeps its power.
 *   - vectis' messages (stderr) go to logcat and the screen; device output
 *     (stdout) is read and dropped, so the pipe never fills.
 *   - With the camex tunnel on (#97), the box joins the camex server of the
 *     settings (org.openipc.camex.tunnel, bundles/camex) while the service
 *     runs: the server's nginx opens the page by the client's name on the web
 *     port.  It reconnects by itself and shares this service's notification;
 *     without the VPN consent it says so and tries again every minute.
 */
package org.openipc.vectis;

import android.annotation.SuppressLint;
import android.app.Notification;
import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.net.LocalServerSocket;
import android.net.LocalSocket;
import android.net.LocalSocketAddress;
import android.os.Build;
import android.os.Handler;
import android.os.IBinder;
import android.os.Looper;
import android.os.ParcelFileDescriptor;
import android.os.PowerManager;
import android.text.TextUtils;
import android.util.Log;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileDescriptor;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import org.openipc.camex.tunnel.CamexTunnel;

public class VectisService extends Service {
    public static final String ACTION_START = "org.openipc.vectis.START";
    public static final String ACTION_STOP = "org.openipc.vectis.STOP";
    public static final String ACTION_RESTART = "org.openipc.vectis.RESTART";
    public static final String ACTION_USB = "org.openipc.vectis.USB";
    /** The abstract socket of -U fd:@NAME (at most 27 characters). */
    static final String SOCKET = "org.openipc.vectis.usb";
    static final String TAG = "vectis";
    private static final String CHANNEL_ID = "vectis";
    private static final int NOTIFY_ID = 1;
    private static final int LOG_LINES = 200;
    private static final int RESTART_DELAY_MS = 3000;
    private static final int STOP_TIMEOUT_MS = 3000;
    /** A tunnel without the consent (or refused) is started again this often. */
    private static final int TUNNEL_RETRY_MS = 60000;
    /** A socket name still taken (the old one not released yet) is tried again this often, this many times. */
    private static final int BIND_RETRY_MS = 1000, BIND_TRIES = 10;
    /** VID:PID pairs vectis drives (ch34x.c, res/xml/device_filter.xml). */
    private static final int[][] IDS = {
        {0x1a86, 0x5523}, {0x1a86, 0x7522}, {0x1a86, 0x7523}, {0x2184, 0x0057}, {0x4348, 0x5523}, {0x9986, 0x7523},
    };

    /** The running service, for the activity; null if none. */
    static volatile VectisService instance;
    /** What the screen and the notification say about the tunnel; "" when it is off. */
    static volatile String tunnelStatus = "";
    /** The notification's text about vectis (the tunnel's line is added to it). */
    private static volatile String notifyText = "";

    final LogBuffer log = new LogBuffer(LOG_LINES);
    private final Object lock = new Object();
    private UsbDevice device;             // guarded by lock: the adapter found, if any
    private UsbDeviceConnection conn;     // guarded by lock: open while we have access
    private Process process;              // guarded by lock
    private boolean wanted;               // guarded by lock: keep vectis running
    private int generation;               // guarded by lock: which runLoop is the current one
    private volatile LocalServerSocket server;
    private int bindTries;                // main thread: failed binds since the last good one
    private PowerManager.WakeLock wakeLock;
    private volatile String lastExit = "";
    private boolean askedAccess;          // guarded by this: the access dialog was shown in this run
    private int settingsSeq;              // guarded by lock: bumped by every settings change
    private final Handler main = new Handler(Looper.getMainLooper());
    private Map<String, String> tunnelValues;   // main thread: the values the tunnel was started with

    private final BroadcastReceiver usbEvents = new BroadcastReceiver() {
        @Override
        @SuppressWarnings("deprecation")   // getParcelableExtra(String): the only one before 13
        public void onReceive(Context c, Intent i) {
            UsbDevice d = i.getParcelableExtra(UsbManager.EXTRA_DEVICE);
            if (UsbManager.ACTION_USB_DEVICE_DETACHED.equals(i.getAction())) {
                synchronized (lock) {
                    if (device == null || d == null || !device.getDeviceName().equals(d.getDeviceName()))
                        return;
                }
                closeAdapter();
                line("app: adapter unplugged");
            } else {
                if (d == null || !supported(d))
                    return;               // a keyboard, a flash drive: not ours
                synchronized (VectisService.this) {
                    askedAccess = false;  // our adapter plugged in: one more chance to ask
                }
                openAdapter();
            }
            updateNotification();
        }
    };

    static void send(Context c, String action) {
        Intent i = new Intent(c, VectisService.class).setAction(action);
        if (action.equals(ACTION_START))
            Compat.startForegroundService(c, i);
        else
            c.startService(i);
    }

    static File binary(Context c) {
        return new File(c.getApplicationInfo().nativeLibraryDir, "libvectis.so");
    }

    static boolean supported(UsbDevice d) {
        for (int[] id : IDS)
            if (d.getVendorId() == id[0] && d.getProductId() == id[1])
                return true;
        return false;
    }

    /** The first supported adapter plugged in, or null. */
    static UsbDevice findAdapter(Context c) {
        UsbManager um = (UsbManager) c.getSystemService(Context.USB_SERVICE);
        if (um == null)
            return null;
        for (UsbDevice d : um.getDeviceList().values())
            if (supported(d))
                return d;
        return null;
    }

    @Override
    public void onCreate() {
        super.onCreate();
        instance = this;
        PowerManager pm = (PowerManager) getSystemService(POWER_SERVICE);
        wakeLock = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "vectis:serve");
        wakeLock.setReferenceCounted(false);
        IntentFilter f = new IntentFilter(UsbManager.ACTION_USB_DEVICE_ATTACHED);
        f.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
        registerReceiver(usbEvents, f);
        // the tunnel's foreground service posts this service's notification: one for both
        CamexTunnel.setNotification(NOTIFY_ID, (ctx, st, addr, err) -> notification(ctx));
        CamexTunnel.addListener(tunnelState);
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        String action = intent != null && intent.getAction() != null ? intent.getAction() : ACTION_START;
        if (action.equals(ACTION_STOP)) {
            stopVectis();                 // first: the tunnel's last state then posts no notification
            stopTunnel();
            dropNotificationLater(getApplicationContext());
            Compat.stopForeground(this);
            stopSelf();
            return START_NOT_STICKY;
        }
        if (action.equals(ACTION_USB) && !Settings.running(this)) {
            // An access answer that arrives after Stop: nothing to start
            if (!isWanted())
                stopSelf();
            return START_NOT_STICKY;
        }
        goForeground(getString(R.string.notify_starting));
        if (action.equals(ACTION_RESTART)) {
            restartVectis();
        } else {
            openAdapter();
            startVectis();
        }
        startTunnel();
        updateNotification();
        return START_STICKY;
    }

    @Override
    public void onDestroy() {
        unregisterReceiver(usbEvents);
        CamexTunnel.removeListener(tunnelState);
        stopVectis();
        stopTunnel();
        dropNotificationLater(getApplicationContext());
        instance = null;
        super.onDestroy();
    }

    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }

    // ── state for the screen ──────────────────────────────────────

    /** True while vectis should be running (Start pressed or started on boot). */
    boolean isWanted() {
        synchronized (lock) {
            return wanted;
        }
    }

    boolean isRunning() {
        synchronized (lock) {
            return process != null;
        }
    }

    /** The adapter for the screen: null if none is plugged in. */
    UsbDevice adapter() {
        synchronized (lock) {
            return device;
        }
    }

    boolean hasAccess() {
        synchronized (lock) {
            return conn != null;
        }
    }

    String lastExit() {
        return lastExit;
    }

    // ── the adapter ───────────────────────────────────────────────

    /**
     * Finds the adapter and opens it if we have access; safe to call again.
     * Synchronized as a whole: it runs on the main thread (plug-in events) and
     * on the fd thread (reopen()), and two openDevice() at once would leave one
     * connection open with nobody to close it.
     */
    synchronized void openAdapter() {
        UsbManager um = (UsbManager) getSystemService(USB_SERVICE);
        UsbDevice d = findAdapter(this);
        synchronized (lock) {
            if (conn != null && device != null && d != null && device.getDeviceName().equals(d.getDeviceName()))
                return;
        }
        closeAdapter();
        if (d == null)
            return;
        synchronized (lock) {
            device = d;
        }
        if (!um.hasPermission(d)) {
            askAccess(um, d);
            return;
        }
        UsbDeviceConnection c = um.openDevice(d);
        if (c == null) {
            line("app: cannot open " + describe(d));
            return;
        }
        synchronized (lock) {
            conn = c;
        }
        line("app: adapter " + describe(d));
    }

    /**
     * vectis asks for the adapter while we hold none: if one is plugged in and
     * we have access (an earlier openDevice() failed, say), open it now.
     * Never asks for access: that is left to starts and plug-in events.
     */
    private boolean reopen() {
        UsbDevice d = findAdapter(this);
        if (d == null || !((UsbManager) getSystemService(USB_SERVICE)).hasPermission(d))
            return false;
        openAdapter();
        return true;
    }

    /**
     * Without access, ask once per service start.  Android 10 grants access
     * to the "use by default" app only on a plug-in event, and an adapter
     * plugged in before boot gets none, so after a reboot the system dialog
     * comes up on the screen: one OK with the remote.  The answer comes back
     * as ACTION_USB (onStartCommand opens the adapter then).
     */
    private void askAccess(UsbManager um, UsbDevice d) {
        if (askedAccess) {
            line("app: no access to " + describe(d) + " yet: open the app or plug the adapter in again");
            return;
        }
        askedAccess = true;
        line("app: asking for access to " + describe(d));
        um.requestPermission(d, Compat.usbPermissionService(this,
                new Intent(this, VectisService.class).setAction(ACTION_USB)));
    }

    private void closeAdapter() {
        UsbDeviceConnection c;
        synchronized (lock) {
            c = conn;
            conn = null;
            device = null;
        }
        if (c != null)
            c.close();
    }

    static String describe(UsbDevice d) {
        return String.format(Locale.US, "%04x:%04x %s", d.getVendorId(), d.getProductId(), d.getDeviceName());
    }

    /** Serves the adapter's fd to vectis (-U fd:@SOCKET) until the socket is closed. */
    private void serve(LocalServerSocket s) {
        int me = android.os.Process.myUid();
        while (true) {
            LocalSocket c;
            try {
                c = s.accept();
            } catch (IOException e) {
                return;                   // closed by stopVectis()
            }
            if (server != s) {            // stopVectis() woke us: this socket is done
                try {
                    c.close();
                } catch (IOException ignored) {
                    // nothing to do
                }
                return;
            }
            try {
                if (c.getPeerCredentials().getUid() != me)
                    continue;             // not ours: no fd, closed below
                UsbDevice d;
                synchronized (lock) {
                    d = conn != null ? device : null;
                }
                if (d == null && reopen()) {
                    synchronized (lock) {
                        d = conn != null ? device : null;
                    }
                }
                if (d == null)
                    continue;             // no adapter: vectis retries
                // A newly opened file each time, never a copy of one a previous
                // vectis used: its URBs stay in that file (they point into the
                // old process) until the file is closed.  Our copy is closed at
                // once, so the file lives exactly as long as this vectis.
                UsbDeviceConnection fresh = ((UsbManager) getSystemService(USB_SERVICE)).openDevice(d);
                if (fresh == null)
                    continue;
                try (ParcelFileDescriptor p = ParcelFileDescriptor.fromFd(fresh.getFileDescriptor())) {
                    c.setFileDescriptorsForSend(new FileDescriptor[] {p.getFileDescriptor()});
                    OutputStream out = c.getOutputStream();
                    out.write(1);
                    out.flush();
                } finally {
                    fresh.close();
                }
            } catch (IOException e) {
                Log.w(TAG, "fd handover: " + e);
            } finally {
                try {
                    c.close();
                } catch (IOException ignored) {
                    // nothing to do
                }
            }
        }
    }

    // ── vectis ────────────────────────────────────────────────────

    @SuppressLint("WakelockTimeout")   // vectis serves until Stop: no natural timeout
    private void startVectis() {
        int gen;
        synchronized (lock) {
            if (wanted)
                return;
            wanted = true;
            gen = ++generation;
        }
        try {
            server = new LocalServerSocket(SOCKET);
        } catch (IOException e) {
            synchronized (lock) {
                wanted = false;
            }
            // The previous socket may not have let go of the name yet (#101): again in a
            // second, a few times, unless Stop came meanwhile
            if (++bindTries <= BIND_TRIES) {
                line("app: the socket @" + SOCKET + " is still taken (" + e.getMessage() + "), again in 1 s");
                main.postDelayed(() -> {
                    if (Settings.running(this) && !isWanted() && instance == this) {
                        startVectis();
                        startTunnel();
                        updateNotification();
                    }
                }, BIND_RETRY_MS);
            } else {
                line("app: cannot open the socket @" + SOCKET + ": " + e.getMessage());
            }
            return;
        }
        bindTries = 0;
        LocalServerSocket s = server;
        new Thread(() -> serve(s), "vectis-fd").start();
        if (wakeLock != null)
            wakeLock.acquire();           // held while vectis serves, released by stopVectis()
        new Thread(() -> runLoop(gen), "vectis-run").start();
    }

    /** Runs vectis, restarts it while wanted; ends when a newer start took over. */
    private void runLoop(int gen) {
        while (true) {
            synchronized (lock) {
                if (!wanted || gen != generation)
                    break;
            }
            runOnce(gen);
            synchronized (lock) {
                if (!wanted || gen != generation)
                    break;
                try {
                    lock.wait(RESTART_DELAY_MS);
                } catch (InterruptedException e) {
                    break;
                }
            }
        }
    }

    /**
     * The vectis command line.  The ports listen on every interface, as
     * -l PORT and -w PORT do on Linux: the console, the power switch and the
     * web terminal are meant to be reached from the lab network (README; the
     * web page takes an optional password, the console none).
     */
    static List<String> command(String bin, int baud, int console, int web) {
        List<String> cmd = new ArrayList<>();
        cmd.add(bin);
        cmd.add("-U");
        cmd.add("fd:@" + SOCKET);
        cmd.add("-b");
        cmd.add(Integer.toString(baud));
        cmd.add("-l");
        cmd.add(Integer.toString(console));
        cmd.add("-w");
        cmd.add(Integer.toString(web));
        return cmd;
    }

    /**
     * The password for vectis: set when there is one, removed otherwise, so
     * an empty one never reaches vectis (an empty value would mean "open"
     * anyway, but nothing from the app's own environment may leak in).
     */
    static void environment(java.util.Map<String, String> env, String password) {
        if (password == null || password.isEmpty())
            env.remove("VECTIS_WEB_PASSWORD");
        else
            env.put("VECTIS_WEB_PASSWORD", password);
    }

    private void runOnce(int gen) {
        int seq;
        synchronized (lock) {
            seq = settingsSeq;
        }
        List<String> cmd = command(binary(this).getPath(), Settings.baud(this),
                Settings.consolePort(this), Settings.webPort(this));
        Process p;
        try {
            ProcessBuilder pb = new ProcessBuilder(cmd);
            environment(pb.environment(), Settings.password(this));
            p = pb.start();
        } catch (IOException | RuntimeException e) {   // RuntimeException: a bad environment value
            line("app: cannot start vectis: " + e.getMessage());
            return;
        }
        synchronized (lock) {
            process = p;
            // Stopped, a newer run loop, or settings saved while it was starting
            if (!wanted || gen != generation || seq != settingsSeq)
                p.destroy();
        }
        line("app: started " + TextUtils.join(" ", cmd));
        updateNotification();
        Thread drain = new Thread(() -> discard(p.getInputStream()), "vectis-stdout");
        drain.start();
        try (BufferedReader r = new BufferedReader(
                new InputStreamReader(p.getErrorStream(), StandardCharsets.UTF_8))) {
            String l;
            while ((l = r.readLine()) != null)
                line(l);
        } catch (IOException ignored) {
            // the process is gone
        }
        int code;
        try {
            code = p.waitFor();
        } catch (InterruptedException e) {
            p.destroy();
            code = -1;
        }
        synchronized (lock) {
            if (process == p)
                process = null;
        }
        lastExit = "vectis exited with code " + code;
        line("app: " + lastExit);
        updateNotification();
    }

    /** Device output on stdout: read and dropped (the screen shows status lines only). */
    private static void discard(InputStream in) {
        byte[] buf = new byte[4096];
        try {
            while (in.read(buf) >= 0) {
                // dropped
            }
        } catch (IOException ignored) {
            // the process is gone
        }
    }

    /** Stops vectis and starts it again with the current settings. */
    private void restartVectis() {
        Process p;
        boolean running;
        synchronized (lock) {
            settingsSeq++;                // a child started with older settings ends itself (runOnce)
            p = process;
            running = wanted;
        }
        if (!running) {
            openAdapter();
            startVectis();
            return;
        }
        line("app: restarting vectis with the new settings");
        if (p != null)
            p.destroy();                  // runLoop starts it again
        synchronized (lock) {
            lock.notifyAll();
        }
    }

    private void stopVectis() {
        Process p;
        LocalServerSocket s;
        synchronized (lock) {
            wanted = false;
            p = process;
            s = server;
            server = null;
            lock.notifyAll();
        }
        if (p != null) {
            p.destroy();
            try {
                if (!Compat.waitFor(p, STOP_TIMEOUT_MS) && Build.VERSION.SDK_INT >= 26)
                    p.destroyForcibly();
            } catch (InterruptedException ignored) {
                // leaving anyway
            }
        }
        if (s != null) {
            // close() alone may leave serve() blocked in accept() and the name taken until it
            // returns (#101): a connection wakes it first, and it sees it is no longer the server
            try (LocalSocket w = new LocalSocket()) {
                w.connect(new LocalSocketAddress(SOCKET));
            } catch (IOException ignored) {
                // nothing listens: nothing to wake
            }
            try {
                s.close();
            } catch (IOException ignored) {
                // closed
            }
        }
        closeAdapter();
        if (wakeLock != null && wakeLock.isHeld())
            wakeLock.release();
    }

    // ── notification and log ─────────────────────────────────────

    // ── the camex tunnel ─────────────────────────────────────────

    /** The tunnel of the settings, when it is on; a start refused now is tried again later. */
    private void startTunnel() {
        main.removeCallbacks(retryTunnel);
        // Settings.running too: a start that arrives after Stop (queued, or a sticky restart)
        // must not bring the tunnel back
        if (!Settings.tunnelEnabled(this) || !Settings.running(this) || !isWanted()) {
            stopTunnel();
            return;
        }
        Map<String, String> v = Settings.tunnel(this);
        if (CamexTunnel.running() && v.equals(tunnelValues))
            return;                       // up with these values: a vectis restart does not cut it
        tunnelValues = v;
        if (!CamexTunnel.start(this, v))
            main.postDelayed(retryTunnel, TUNNEL_RETRY_MS);   // no consent yet: adb may give it meanwhile
    }

    private void stopTunnel() {
        main.removeCallbacks(retryTunnel);
        tunnelValues = null;
        if (CamexTunnel.running() || CamexTunnel.state() != CamexTunnel.State.STOPPED)
            CamexTunnel.stop();
        showTunnel(CamexTunnel.State.STOPPED, "", "");
    }

    private final Runnable retryTunnel = () -> {
        CamexTunnel.State s = CamexTunnel.state();
        if (s == CamexTunnel.State.NEEDS_CONSENT || s == CamexTunnel.State.FAILED || s == CamexTunnel.State.STOPPED)
            startTunnel();
    };

    private final CamexTunnel.Listener tunnelState = (st, addr, err) -> {
        showTunnel(st, addr, err);
        if ((st == CamexTunnel.State.NEEDS_CONSENT || st == CamexTunnel.State.FAILED)
                && Settings.tunnelEnabled(this) && Settings.running(this) && isWanted()) {
            main.removeCallbacks(retryTunnel);   // consent revoked while it ran, or refused
            main.postDelayed(retryTunnel, TUNNEL_RETRY_MS);
        }
    };

    private void showTunnel(CamexTunnel.State st, String addr, String err) {
        String t;
        if (!Settings.tunnelEnabled(this))
            t = "";
        else if (st == CamexTunnel.State.CONNECTED)
            t = getString(R.string.tunnel_connected, "http://" + addr + ":" + Settings.webPort(this) + "/");
        else if (st == CamexTunnel.State.CONNECTING)
            t = getString(R.string.tunnel_connecting) + (err.isEmpty() ? "" : " (" + err + ")");
        else if (st == CamexTunnel.State.NEEDS_CONSENT)
            t = getString(R.string.tunnel_needs_consent);
        else if (st == CamexTunnel.State.FAILED)
            t = getString(R.string.tunnel_failed, err);
        else
            t = getString(R.string.tunnel_stopped);
        if (!t.equals(tunnelStatus)) {
            tunnelStatus = t;
            if (!t.isEmpty())
                line("app: camex tunnel: " + t);
        }
        if (instance != null && isWanted())
            goForeground(notifyText);     // not while stopping: that would post it again
    }

    /**
     * The tunnel's service leaves the shared notification behind when it stops (it is the
     * app's, so it detaches it): once it has, and unless vectis runs again, remove it.
     */
    private static void dropNotificationLater(Context app) {
        Handler h = new Handler(Looper.getMainLooper());
        Runnable drop = () -> {
            VectisService s = instance;
            if (s == null || !s.isWanted())
                ((android.app.NotificationManager) app.getSystemService(NOTIFICATION_SERVICE)).cancel(NOTIFY_ID);
        };
        h.postDelayed(drop, 1000);
        h.postDelayed(drop, 3000);
    }

    /** The one notification of vectis and its tunnel. */
    static Notification notification(Context c) {
        String text = notifyText.isEmpty() ? c.getString(R.string.notify_starting) : notifyText;
        if (!tunnelStatus.isEmpty())
            text += "\n" + c.getString(R.string.label_tunnel).trim() + " " + tunnelStatus;
        return Compat.ongoing(c, CHANNEL_ID, c.getString(R.string.app_name), text,
                R.drawable.ic_notify, new Intent(c, MainActivity.class));
    }

    private void goForeground(String text) {
        notifyText = text;
        Notification n = notification(this);
        if (Build.VERSION.SDK_INT >= 34)
            startForeground(NOTIFY_ID, n, android.content.pm.ServiceInfo.FOREGROUND_SERVICE_TYPE_SPECIAL_USE);
        else
            startForeground(NOTIFY_ID, n);
    }

    private void updateNotification() {
        if (instance == null)
            return;
        String text;
        if (!isRunning())
            text = getString(R.string.notify_starting);
        else if (hasAccess())
            text = getString(R.string.notify_serving, Settings.consolePort(this), Settings.webPort(this),
                    getString(Settings.password(this).isEmpty() ? R.string.web_open : R.string.web_protected));
        else
            text = getString(R.string.notify_no_adapter);
        goForeground(text);
    }

    /** A line for the screen; the app's own also to logcat (vectis logs there itself). */
    void line(String l) {
        if (l.startsWith("app: "))
            Log.i(TAG, l);
        log.add(l);
    }

    /** IPv4 addresses of the device, loopback left out; empty if none. */
    static List<String> addresses() {
        List<String> out = new ArrayList<>();
        try {
            for (NetworkInterface ni : Collections.list(NetworkInterface.getNetworkInterfaces())) {
                if (!ni.isUp() || ni.isLoopback())
                    continue;
                for (InetAddress a : Collections.list(ni.getInetAddresses()))
                    if (a instanceof Inet4Address)
                        out.add(a.getHostAddress());
            }
        } catch (IOException | RuntimeException ignored) {
            // no network information
        }
        return out;
    }
}
