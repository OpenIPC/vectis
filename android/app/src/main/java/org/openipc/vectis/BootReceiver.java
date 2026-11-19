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
        boolean boot = Intent.ACTION_BOOT_COMPLETED.equals(a);
        if (!boot && !Intent.ACTION_MY_PACKAGE_REPLACED.equals(a))
            return;
        boolean start = shouldStart(boot, Settings.serviceUp(c), Settings.running(c), Settings.startOnBoot(c));
        if (boot)
            Settings.setServiceUp(c, false);  // a reboot ends the service: nothing to bring back
        if (start)
            VectisService.send(c, VectisService.ACTION_START);
    }

    /**
     * Whether to start the service.  At boot: Start on boot and not stopped.  After an update
     * (which killed a running service without onDestroy): it was up and not stopped; serviceUp
     * null is an install from before the mark (1.17.2 and older), where not stopped is all
     * there is to go by.
     */
    static boolean shouldStart(boolean boot, Boolean serviceUp, boolean running, boolean startOnBoot) {
        if (!running)
            return false;                 // Stop was pressed: nothing starts it but Start
        if (boot)
            return startOnBoot;
        return serviceUp == null || serviceUp;
    }
}
