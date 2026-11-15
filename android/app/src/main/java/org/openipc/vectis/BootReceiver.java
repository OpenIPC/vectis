/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * BootReceiver.java — starts vectis after boot when "Start on boot" is on
 * and the user has not stopped it.  Without access to the adapter yet the
 * service waits: Android grants it with the plug-in event of the adapter
 * once "use by default" was chosen (UsbAttachActivity).
 */
package org.openipc.vectis;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;

public class BootReceiver extends BroadcastReceiver {
    @Override
    public void onReceive(Context c, Intent intent) {
        if (!Intent.ACTION_BOOT_COMPLETED.equals(intent.getAction()))
            return;
        if (Settings.startOnBoot(c) && Settings.running(c))
            VectisService.send(c, VectisService.ACTION_START);
    }
}
