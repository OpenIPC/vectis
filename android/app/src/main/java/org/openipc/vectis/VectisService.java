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
 *     vectis -U fd:@org.openipc.vectis.usb -b BAUD -l PORT -w PORT.
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
import android.os.Build;
import android.os.IBinder;
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
    /** VID:PID pairs vectis drives (ch34x.c, res/xml/device_filter.xml). */
    private static final int[][] IDS = { {0x1a86, 0x7523}, {0x1a86, 0x7522}, {0x1a86, 0x5523} };

    /** The running service, for the activity; null if none. */
    static volatile VectisService instance;

    final LogBuffer log = new LogBuffer(LOG_LINES);
    private final Object lock = new Object();
    private UsbDevice device;             // guarded by lock: the adapter found, if any
    private UsbDeviceConnection conn;     // guarded by lock: open while we have access
    private Process process;              // guarded by lock
    private boolean wanted;               // guarded by lock: keep vectis running
    private int generation;               // guarded by lock: which runLoop is the current one
    private LocalServerSocket server;
    private PowerManager.WakeLock wakeLock;
    private volatile String lastExit = "";
    private boolean askedAccess;          // the access dialog was shown in this run

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
                askedAccess = false;      // a new plug-in: one more chance to ask
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
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        String action = intent != null && intent.getAction() != null ? intent.getAction() : ACTION_START;
        if (action.equals(ACTION_STOP)) {
            stopVectis();
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
        updateNotification();
        return START_STICKY;
    }

    @Override
    public void onDestroy() {
        unregisterReceiver(usbEvents);
        stopVectis();
        instance = null;
        super.onDestroy();
    }

    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }

    // ── state for the screen ──────────────────────────────────────

    private boolean isWanted() {
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

    /** Finds the adapter and opens it if we have access; safe to call again. */
    void openAdapter() {
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
            try {
                if (c.getPeerCredentials().getUid() != me)
                    continue;             // not ours: no fd, closed below
                UsbDevice d;
                synchronized (lock) {
                    d = conn != null ? device : null;
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
            line("app: cannot open the socket @" + SOCKET + ": " + e.getMessage());
            synchronized (lock) {
                wanted = false;
            }
            return;
        }
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
     * -l PORT and -w PORT do on Linux: the console and the power switch are
     * meant to be reached from the lab network (README, no authentication).
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

    private void runOnce(int gen) {
        List<String> cmd = command(binary(this).getPath(), Settings.baud(this),
                Settings.consolePort(this), Settings.webPort(this));
        Process p;
        try {
            p = new ProcessBuilder(cmd).start();
        } catch (IOException e) {
            line("app: cannot start vectis: " + e.getMessage());
            return;
        }
        synchronized (lock) {
            process = p;
            if (!wanted || gen != generation)
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
        synchronized (lock) {
            p = process;
        }
        if (p == null) {
            openAdapter();
            startVectis();
            return;
        }
        line("app: restarting vectis with the new settings");
        p.destroy();                      // runLoop starts it again
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

    private void goForeground(String text) {
        Notification n = Compat.ongoing(this, CHANNEL_ID, getString(R.string.app_name), text,
                R.drawable.ic_notify, new Intent(this, MainActivity.class));
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
            text = getString(R.string.notify_serving, Settings.consolePort(this), Settings.webPort(this));
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
