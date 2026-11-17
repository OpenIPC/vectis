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

    @Test
    public void webPortDefaultIs8085() {
        assertEquals(8085, Settings.DEFAULT_WEB);   // camex servers and Decoder expect it
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
        String r = new String(WsCodec.request(8085, "a2V5", "pw"), StandardCharsets.US_ASCII);
        assertTrue(r.startsWith("GET /ws/terminal HTTP/1.1\r\nHost: 127.0.0.1:8085\r\n"));
        assertTrue(r.contains("Sec-WebSocket-Key: a2V5\r\n") && r.contains("Authorization: Basic OnB3\r\n"));
        assertFalse(r.contains("Origin"));
        assertTrue(r.endsWith("\r\n\r\n"));
        assertFalse(new String(WsCodec.request(8085, "a2V5", ""), StandardCharsets.US_ASCII)
                .contains("Authorization"));
    }
}
