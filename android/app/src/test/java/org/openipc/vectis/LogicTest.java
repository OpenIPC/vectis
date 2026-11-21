/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * LogicTest.java — the app's logic that needs no device: the settings
 * check against vectis' baud table, the vectis command line and its
 * password variable, the log tail shown on the screen, HTTP Basic, the
 * terminal's text (TermBuffer) and its WebSocket frames (WsCodec).
 * Run: cd android && gradle testDebugUnitTest
 */
package org.openipc.vectis;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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
        assertTrue(Settings.valid(115200, 1024, 65535));
        assertFalse(Settings.valid(115200, 0, 2081));
        assertFalse(Settings.valid(115200, 23, 2081));       // needs root on Android
        assertFalse(Settings.valid(115200, 2021, 80));
        assertFalse(Settings.valid(115200, 2021, 65536));
        assertFalse(Settings.valid(115200, 2021, 2021));
        assertTrue(Settings.valid(Settings.DEFAULT_BAUD, Settings.DEFAULT_CONSOLE, Settings.DEFAULT_WEB));
    }

    @Test
    public void commandLine() {
        assertEquals(Arrays.asList("/lib/libvectis.so", "-U", "fd:@org.openipc.vectis.usb", "-b", "921600",
                                   "--web-local", "@org.openipc.vectis.web", "-l", "2022", "-w", "8080"),
                     VectisService.command("/lib/libvectis.so", 921600, 2022, 8080, true));
        // remote access off: no network port at all, the app's own socket only
        assertEquals(Arrays.asList("/lib/libvectis.so", "-U", "fd:@org.openipc.vectis.usb", "-b", "921600",
                                   "--web-local", "@org.openipc.vectis.web"),
                     VectisService.command("/lib/libvectis.so", 921600, 2022, 8080, false));
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

    @Test
    public void aTakenWebPortMovesToTheNextFreeOne() {
        assertEquals(8085, VectisService.freePort(8085, 2021, p -> true));
        assertEquals(8086, VectisService.freePort(8085, 2021, p -> p != 8085));
        // never the console port
        assertEquals(8087, VectisService.freePort(8085, 8086, p -> p != 8085));
        // nothing free: the setting, vectis then says why
        assertEquals(8085, VectisService.freePort(8085, 2021, p -> false));
        assertEquals(65535, VectisService.freePort(65535, 2021, p -> p == 65535));
    }

    @Test
    public void localHttpReply() {
        byte[] raw = "HTTP/1.1 200 OK\r\nContent-Length: 2\r\n\r\n{}".getBytes(StandardCharsets.US_ASCII);
        Http.Reply r = Http.parse(raw, raw.length);
        assertEquals(200, r.code);
        assertEquals("{}", r.body);
        byte[] bad = "garbage".getBytes(StandardCharsets.US_ASCII);
        assertEquals(0, Http.parse(bad, bad.length).code);
    }

    @Test
    public void startAfterBootOrUpdate() {
        // boot: Start on boot and not stopped
        assertTrue(BootReceiver.shouldStart(true, true, true, true, true));
        assertFalse(BootReceiver.shouldStart(true, true, true, true, false));
        assertFalse(BootReceiver.shouldStart(true, true, true, false, true));
        // update: the service was up and not stopped
        assertTrue(BootReceiver.shouldStart(false, true, true, true, false));
        assertFalse(BootReceiver.shouldStart(false, false, true, true, true));
        assertFalse(BootReceiver.shouldStart(false, true, true, false, true));
        // update from an install without the mark: only one that was set up
        assertTrue(BootReceiver.shouldStart(false, null, true, true, false));
        assertFalse(BootReceiver.shouldStart(false, null, false, true, false));   // never opened
        assertFalse(BootReceiver.shouldStart(false, null, true, false, false));   // stopped
    }

    @Test
    public void logLinesGoneFromTheTop() {
        // counted by the buffer, not by the text: equal lines repeat
        LogBuffer log = new LogBuffer(3);
        log.add("x");
        log.add("x");
        assertEquals(0, log.snapshot().removed);
        log.add("x");
        log.add("x");
        log.add("y");
        LogBuffer.Snapshot s = log.snapshot();
        assertEquals(2, s.removed);
        assertEquals(3, s.text.split("\n").length);
        // and their text, line ends included
        assertEquals(0, Brass.LogWindow.charsOfLines("ab\ncd\n", 0));
        assertEquals(3, Brass.LogWindow.charsOfLines("ab\ncd\n", 1));
        assertEquals(6, Brass.LogWindow.charsOfLines("ab\ncd\n", 2));
        assertEquals(6, Brass.LogWindow.charsOfLines("ab\ncd\n", 5));
        assertEquals(5, Brass.LogWindow.charsOfLines("ab\ncd", 2));
    }

    @Test
    public void webPortDefaultIs8085() {
        assertEquals(8085, Settings.DEFAULT_WEB);   // camex servers and Decoder expect it
    }

    @Test
    public void aSavedWebPortIsKept() {
        Map<String, Object> saved = new HashMap<>();
        assertEquals(8085, Settings.stored(saved, "web_port", Settings.DEFAULT_WEB));
        saved.put("web_port", 2081);                 // saved by 1.13 or earlier
        assertEquals(2081, Settings.stored(saved, "web_port", Settings.DEFAULT_WEB));
        assertEquals(8085, Settings.stored(null, "web_port", Settings.DEFAULT_WEB));
    }

    @Test
    public void passwordFitsVectis() {
        assertTrue(Settings.validPassword(""));
        assertTrue(Settings.validPassword(new String(new char[255]).replace('\0', 'x')));
        assertFalse(Settings.validPassword(new String(new char[256]).replace('\0', 'x')));
        // bytes, not characters: 128 Cyrillic letters are 256 bytes
        assertFalse(Settings.validPassword(new String(new char[128]).replace('\0', 'ж')));
        assertFalse(Settings.validPassword("ab\0cd"));     // cannot go into the environment
    }

    @Test
    public void passwordGoesToTheEnvironmentOnlyWhenSet() {
        Map<String, String> env = new HashMap<>();
        env.put("VECTIS_WEB_PASSWORD", "left over");
        VectisService.environment(env, "");
        assertFalse(env.containsKey("VECTIS_WEB_PASSWORD"));     // empty: open, nothing passed
        VectisService.environment(env, null);
        assertFalse(env.containsKey("VECTIS_WEB_PASSWORD"));
        VectisService.environment(env, "s3cret:x");
        assertEquals("s3cret:x", env.get("VECTIS_WEB_PASSWORD"));
    }

    @Test
    public void basicAuthWithAnyUserName() {
        assertEquals(null, Http.basic(""));
        assertEquals(null, Http.basic(null));
        assertEquals("Basic OnNlY3JldA==", Http.basic("secret"));   // ":secret"
        assertEquals("", Http.base64(new byte[0]));
        assertEquals("YQ==", Http.base64("a".getBytes(StandardCharsets.UTF_8)));
        assertEquals("YWI=", Http.base64("ab".getBytes(StandardCharsets.UTF_8)));
        assertEquals("YWJj", Http.base64("abc".getBytes(StandardCharsets.UTF_8)));
    }

    private static void feed(TermBuffer t, String s) {
        byte[] b = s.getBytes(StandardCharsets.UTF_8);
        t.feed(b, 0, b.length);
    }

    @Test
    public void terminalTextDropsColoursKeepsLines() {
        TermBuffer t = new TermBuffer(100);
        feed(t, "U-Boot 2016.11\r\n\u001b[1;32mroot@openipc\u001b[0m:~# ls\r\n");
        assertEquals("U-Boot 2016.11\nroot@openipc:~# ls\n", t.text(10));
        feed(t, "\u001b]0;title\u0007x\ty");
        assertTrue(t.text(10).endsWith("x       y"));
    }

    @Test
    public void terminalCarriageReturnRewritesTheLine() {
        TermBuffer t = new TermBuffer(100);
        feed(t, "Writing 10%\rWriting 100%\r\n");
        assertEquals("Writing 100%\n", t.text(10));
        feed(t, "abcdef\r\u001b[Kxy\bz\n");
        assertTrue(t.text(10).endsWith("xz\n"));
    }

    @Test
    public void terminalEraseLineVariants() {
        TermBuffer t = new TermBuffer(100);
        feed(t, "abcdef\b\b\u001b[1K");                 // erase from the start to the cursor
        assertEquals("     f", t.text(10));
        t.clear();
        feed(t, "abcdef\b\b\u001b[2Kz");                // erase the whole line, the cursor stays
        assertEquals("    z", t.text(10));
        t.clear();
        feed(t, "abcdef\b\b\u001b[0K");
        assertEquals("abcd", t.text(10));
    }

    @Test
    public void terminalWrapsEndlessOutput() {
        TermBuffer t = new TermBuffer(5);
        byte[] b = new byte[10 * TermBuffer.MAX_COLS + 7];
        java.util.Arrays.fill(b, (byte) 'x');
        t.feed(b, 0, b.length);
        String all = t.text(100);
        assertTrue(all.length() <= 5 * (TermBuffer.MAX_COLS + 1) + TermBuffer.MAX_COLS);
        for (String l : all.split("\n"))
            assertTrue(l.length() <= TermBuffer.MAX_COLS);
        assertTrue(all.endsWith("\nxxxxxxx"));
    }

    @Test
    public void terminalDropsCharsetSelection() {
        TermBuffer t = new TermBuffer(100);
        feed(t, "ok\u001b(B\u001b[m done\u001b)0!");     // tput sgr0 and friends
        assertEquals("ok done!", t.text(10));
        t.clear();
        feed(t, "a\u001bPq#0;2;0;0;0\u001b\\b\u001b_apc");    // DCS, then an APC split across reads
        feed(t, " text\u0007c");
        assertEquals("abc", t.text(10));
    }

    @Test
    public void terminalOneColumnPerCharacter() {
        TermBuffer t = new TermBuffer(100);
        feed(t, "a\uD83D\uDE00b\bc");                  // an emoji, then backspace over b
        assertEquals("a\ufffdc", t.text(10));
        t.clear();
        byte[] b = new byte[TermBuffer.MAX_COLS - 1];
        java.util.Arrays.fill(b, (byte) 'x');
        t.feed(b, 0, b.length);
        feed(t, "\uD83D\uDE00\uD83D\uDE00");               // at the wrap boundary
        String[] l = t.text(10).split("\n");
        assertEquals(TermBuffer.MAX_COLS, l[0].length());
        assertEquals("\ufffd", l[1]);
    }

    @Test
    public void terminalTakesC1Controls() {
        TermBuffer t = new TermBuffer(100);
        feed(t, "\u009b31mred\u009b0m \u009d0;title\u0007ok");
        assertEquals("red ok", t.text(10));
        t.clear();
        feed(t, "A\u009d0;title\u009cB\u0090q");     // ST ends a string, also split across reads
        t.feed(new byte[] {(byte) 0xc2}, 0, 1);
        t.feed(new byte[] {(byte) 0x9c, 'C'}, 0, 2);
        feed(t, "\u001b]0;x\u001b\u009cD");             // ST right after an ESC in the string
        assertEquals("ABCD", t.text(10));
    }

    @Test
    public void terminalCanAndSubCancelSequences() {
        TermBuffer t = new TermBuffer(100);
        feed(t, "a\u001b[12\u0018b\u001b]0;x\u001ac\u001b\u0018d");
        assertEquals("abcd", t.text(10));
    }

    @Test
    public void terminalUtf8SplitAcrossReads() {
        TermBuffer t = new TermBuffer(100);
        byte[] b = "Привет".getBytes(StandardCharsets.UTF_8);
        t.feed(b, 0, 3);
        t.feed(b, 3, b.length - 3);
        assertEquals("Привет", t.text(10));
    }

    @Test
    public void terminalKeepsTheLastLines() {
        TermBuffer t = new TermBuffer(3);
        for (int i = 1; i <= 6; i++)
            feed(t, "l" + i + "\n");
        assertEquals("l4\nl5\nl6\n", t.text(10));
        assertEquals("l6\n", t.text(2));
        t.clear();
        assertEquals("", t.text(10));
    }

    @Test
    public void websocketFramesRoundTrip() {
        byte[] mask = {1, 2, 3, 4};
        for (int len : new int[] {0, 5, 125, 126, 65535, 65536, 70000}) {
            byte[] d = new byte[len];
            for (int i = 0; i < len; i++)
                d[i] = (byte) (i * 7);
            byte[] f = WsCodec.frame(WsCodec.BINARY, d, 0, len, mask);
            assertEquals((byte) 0x82, f[0]);
            assertTrue("masked", (f[1] & 0x80) != 0);
            WsCodec.Parser p = new WsCodec.Parser();
            // in two pieces, as reads come
            List<WsCodec.Frame> a = p.feed(f, 0, f.length / 2);
            List<WsCodec.Frame> b = p.feed(f, f.length / 2, f.length - f.length / 2);
            List<WsCodec.Frame> all = new java.util.ArrayList<>(a);
            all.addAll(b);
            assertEquals(1, all.size());
            assertEquals(WsCodec.BINARY, all.get(0).opcode);
            assertTrue(Arrays.equals(d, all.get(0).data));
        }
    }

    @Test
    public void websocketParserTakesServerFrames() {
        // what vectis sends: unmasked, a ping then data, in one read
        byte[] in = {(byte) 0x89, 0, (byte) 0x82, 3, 'a', 'b', 'c'};
        List<WsCodec.Frame> f = new WsCodec.Parser().feed(in, 0, in.length);
        assertEquals(2, f.size());
        assertEquals(WsCodec.PING, f.get(0).opcode);
        assertEquals("abc", new String(f.get(1).data, StandardCharsets.US_ASCII));
    }

    @Test
    public void websocketRequestHasPasswordAndNoOrigin() {
        String r = new String(WsCodec.request("localhost", "a2V5", "pw"), StandardCharsets.US_ASCII);
        assertTrue(r.startsWith("GET /ws/terminal HTTP/1.1\r\nHost: localhost\r\n"));
        assertTrue(r.contains("Sec-WebSocket-Key: a2V5\r\n") && r.contains("Authorization: Basic OnB3\r\n"));
        assertFalse(r.contains("Origin"));
        assertTrue(r.endsWith("\r\n\r\n"));
        assertFalse(new String(WsCodec.request("localhost", "a2V5", null), StandardCharsets.US_ASCII)
                .contains("Authorization"));
    }

    @Test
    public void tunnelKeysAreTheLibrarysButTheWebPort() {
        java.util.List<String> k = Settings.tunnelKeys();
        assertTrue(k.contains(org.openipc.camex.tunnel.CamexTunnel.SERVER));
        assertTrue(k.contains(org.openipc.camex.tunnel.CamexTunnel.PSK));
        assertTrue(k.contains(org.openipc.camex.tunnel.CamexTunnel.TRANSPORT));
        assertTrue(!k.contains(org.openipc.camex.tunnel.CamexTunnel.WEB_PORT));
    }

    @Test
    public void camexLinkUsedOnlyWhenNotFromRecents() {
        String l = "camex://connect?h=example.org";
        String view = "android.intent.action.VIEW";
        assertEquals(l, MainActivity.linkToOpen(view, l, 0));
        assertEquals(null, MainActivity.linkToOpen(view, l, 0x00100000));   // FLAG_ACTIVITY_LAUNCHED_FROM_HISTORY
        assertEquals(null, MainActivity.linkToOpen("android.intent.action.MAIN", l, 0));
        assertEquals(null, MainActivity.linkToOpen(view, null, 0));
    }

    @Test
    public void saveAsksForTheConsentOnlyWhenTheTunnelStarts() {   // #132
        assertEquals(true, MainActivity.consentOnSave(true, true));
        assertEquals(false, MainActivity.consentOnSave(false, true));   // stopped: Start asks
        assertEquals(false, MainActivity.consentOnSave(true, false));
        assertEquals(false, MainActivity.consentOnSave(false, false));
    }
}
