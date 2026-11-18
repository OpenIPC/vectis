/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * Compat.java — what differs between Android 5.0 (API 21, the app's
 * minimum) and the newer versions, in one place (after pusher).
 *
 * Notes:
 *   - Notification channels and startForegroundService exist from 8.0;
 *     STOP_FOREGROUND_REMOVE from 7.0; PendingIntent.FLAG_IMMUTABLE from 6.0,
 *     FLAG_MUTABLE from 12 (the USB permission answer needs extras filled in).
 *   - Process.destroy() sends SIGTERM from 8.0 and SIGKILL before: either is
 *     fine for vectis, which leaves RTS/DTR as they are on exit with -U.
 */
package org.openipc.vectis;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.os.Build;

final class Compat {
    private Compat() {}

    /** Starts a service that goes to the foreground at once. */
    static void startForegroundService(Context c, Intent i) {
        if (Build.VERSION.SDK_INT >= 26)
            c.startForegroundService(i);
        else
            c.startService(i);
    }

    /** An ongoing notification; creates its channel on 8.0+. */
    @SuppressWarnings("deprecation")   // Notification.Builder(Context): the only one before 8.0
    static Notification ongoing(Context c, String channel, String title, String text, int icon, Intent open) {
        int flags = Build.VERSION.SDK_INT >= 23 ? PendingIntent.FLAG_IMMUTABLE : 0;
        PendingIntent pi = PendingIntent.getActivity(c, 0, open, flags);
        Notification.Builder b;
        if (Build.VERSION.SDK_INT >= 26) {
            NotificationManager nm = (NotificationManager) c.getSystemService(Context.NOTIFICATION_SERVICE);
            nm.createNotificationChannel(new NotificationChannel(channel, title, NotificationManager.IMPORTANCE_LOW));
            b = new Notification.Builder(c, channel);
        } else {
            b = new Notification.Builder(c).setPriority(Notification.PRIORITY_LOW);
        }
        return b.setContentTitle(title)
                .setContentText(text)
                .setStyle(new Notification.BigTextStyle().bigText(text))   // the tunnel's line too
                .setSmallIcon(icon)
                .setContentIntent(pi)
                .setOngoing(true)
                .build();
    }

    @SuppressWarnings("deprecation")   // stopForeground(boolean): the only one before 7.0
    static void stopForeground(Service s) {
        if (Build.VERSION.SDK_INT >= 24)
            s.stopForeground(Service.STOP_FOREGROUND_REMOVE);
        else
            s.stopForeground(true);
    }

    /** The PendingIntent for UsbManager.requestPermission(): the system adds extras to it. */
    static PendingIntent usbPermission(Context c, Intent i) {
        int flags = Build.VERSION.SDK_INT >= 31 ? PendingIntent.FLAG_MUTABLE : 0;
        return PendingIntent.getBroadcast(c, 0, i, flags);
    }

    /** The same for an answer that goes straight to a (running) service. */
    static PendingIntent usbPermissionService(Context c, Intent i) {
        int flags = Build.VERSION.SDK_INT >= 31 ? PendingIntent.FLAG_MUTABLE : 0;
        return PendingIntent.getService(c, 1, i, flags);
    }

    /** Waits up to ms for the process to exit; true if it did. */
    static boolean waitFor(Process p, long ms) throws InterruptedException {
        long end = System.currentTimeMillis() + ms;
        while (true) {
            try {
                p.exitValue();
                return true;
            } catch (IllegalThreadStateException running) {
                if (System.currentTimeMillis() >= end)
                    return false;
                Thread.sleep(50);
            }
        }
    }
}
