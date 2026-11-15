/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * UsbAttachActivity.java — started by Android when a supported adapter is
 * plugged in (res/xml/device_filter.xml).  Choosing the app there grants it
 * access to the adapter; with "use by default" Android grants it on every
 * later plug-in and after a reboot without asking.  The activity has no
 * window of its own: it hands the adapter to the service and finishes.
 */
package org.openipc.vectis;

import android.app.Activity;
import android.os.Bundle;

public class UsbAttachActivity extends Activity {
    @Override
    protected void onCreate(Bundle state) {
        super.onCreate(state);
        if (Settings.running(this))
            VectisService.send(this, VectisService.ACTION_START);
        finish();
    }
}
