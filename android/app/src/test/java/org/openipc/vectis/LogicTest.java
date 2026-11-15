/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * LogicTest.java — the app's logic that needs no device: the settings
 * check against vectis' baud table, the vectis command line and the log
 * tail shown on the screen.  Run: cd android && gradle testDebugUnitTest
 */
package org.openipc.vectis;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import org.junit.Test;

public class LogicTest {
    @Test
    public void settingsTakeOnlyVectisRates() {
        for (int b : Settings.BAUDS)
            assertTrue("baud " + b, Settings.valid(b, 2021, 2081));
        assertFalse(Settings.valid(500, 2021, 2081));
        assertFalse(Settings.valid(1000000, 2021, 2081));
        assertFalse(Settings.valid(0, 2021, 2081));
    }

    @Test
    public void settingsNeedTwoDifferentPorts() {
        assertTrue(Settings.valid(115200, 1, 65535));
        assertFalse(Settings.valid(115200, 0, 2081));
        assertFalse(Settings.valid(115200, 2021, 65536));
        assertFalse(Settings.valid(115200, 2021, 2021));
        assertTrue(Settings.valid(Settings.DEFAULT_BAUD, Settings.DEFAULT_CONSOLE, Settings.DEFAULT_WEB));
    }

    @Test
    public void commandLine() {
        assertEquals(Arrays.asList("/lib/libvectis.so", "-U", "fd:@org.openipc.vectis.usb", "-b", "921600",
                                   "-l", "2022", "-w", "8080"),
                     VectisService.command("/lib/libvectis.so", 921600, 2022, 8080));
        // -U fd:@NAME takes 1 to 27 characters
        assertTrue(VectisService.SOCKET.length() <= 27);
    }

    @Test
    public void logTail() {
        LogBuffer log = new LogBuffer(3);
        assertEquals("", log.tail(5));
        for (int i = 1; i <= 5; i++)
            log.add("line " + i);
        assertEquals(3, log.size());
        String two = log.tail(2);
        assertEquals(2, two.split("\n").length);
        assertTrue(two.contains("line 4") && two.contains("line 5") && !two.contains("line 3"));
        String all = log.tail(10);
        assertTrue(all.contains("line 3") && !all.contains("line 2"));
    }
}
