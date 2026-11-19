/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * BootReceiver.java — starts vectis after boot when "Start on boot" is on
 * and the user has not stopped it, and after an update of the app when the
 * service ran before it (Android stops the app to replace it).  Without access to the adapter yet the
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
        String a = intent.getAction();
        if (Intent.ACTION_BOOT_COMPLETED.equals(a)) {
            Settings.setServiceUp(c, false);  // a reboot ends the service: nothing to bring back
            if (Settings.startOnBoot(c) && Settings.running(c))
                VectisService.send(c, VectisService.ACTION_START);
        } else if (Intent.ACTION_MY_PACKAGE_REPLACED.equals(a)) {
            // The update killed a running service (no onDestroy): start it again, the tunnel too
            if (Settings.serviceUp(c) && Settings.running(c))
                VectisService.send(c, VectisService.ACTION_START);
        }
    }
}
