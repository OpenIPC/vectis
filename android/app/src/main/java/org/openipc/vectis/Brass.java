/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * Brass.java — the brass look of the app screen (brand brass 1.3.0, after
 * pusher and microsnander): heading with the version number, plate, panel,
 * display, the lamp board's rows, section bands, lamp and brass buttons,
 * built in code like the screen itself.  Colours come from
 * res/values/colors.xml.
 */
package org.openipc.vectis;

import android.content.Context;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.os.Build;
import android.util.TypedValue;
import android.view.Gravity;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

final class Brass {
    private Brass() {
    }

    @SuppressWarnings("deprecation")   // getColor(int) of Resources: the only one before 6.0
    static int color(Context c, int id) {
        return Build.VERSION.SDK_INT >= 23 ? c.getColor(id) : c.getResources().getColor(id);
    }

    static int dp(Context c, float v) {
        return (int) TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_DIP, v,
                c.getResources().getDisplayMetrics());
    }

    /** The heading: brass serif capitals with a soft glow, centred. */
    static TextView title(Context c, CharSequence text) {
        TextView t = new TextView(c);
        t.setText(text);
        t.setAllCaps(true);
        t.setTextSize(26);
        t.setTypeface(Typeface.SERIF);
        t.setLetterSpacing(0.2f);
        t.setTextColor(color(c, R.color.brass));
        t.setShadowLayer(dp(c, 8), 0, 0, 0x59c9a24d);
        t.setGravity(Gravity.CENTER_HORIZONTAL);
        return t;
    }

    /** A brass plate with mono capitals, e.g. the version. */
    static TextView plate(Context c, CharSequence text) {
        TextView t = new TextView(c, null, 0, R.style.Plate);
        t.setText(text);
        t.setAllCaps(true);
        return t;
    }

    /** The panel: a vertical column on the brass2-framed gradient. */
    static LinearLayout panel(Context c) {
        LinearLayout p = new LinearLayout(c);
        p.setOrientation(LinearLayout.VERTICAL);
        p.setBackgroundResource(R.drawable.panel);
        int pad = dp(c, 18);
        p.setPadding(pad, pad, pad, pad);
        return p;
    }

    /** The display: mono text on black, dim until a state colours it. */
    static TextView display(Context c) {
        TextView t = new TextView(c);
        t.setBackgroundResource(R.drawable.display);
        t.setTypeface(Typeface.MONOSPACE);
        t.setTextColor(color(c, R.color.dim));
        int pad = dp(c, 12);
        t.setPadding(pad, pad, pad, pad);
        return t;
    }

    /** The lamp of the web console: a circle in the state colour, a thin #6e5320 rim. */
    static GradientDrawable lamp(Context c, int colorId) {
        GradientDrawable d = new GradientDrawable();
        d.setShape(GradientDrawable.OVAL);
        d.setColor(color(c, colorId));
        d.setStroke(dp(c, 1), 0xff6e5320);
        return d;
    }

    /** The heading centred on the screen, the version number in small mono to its right. */
    static LinearLayout titleRow(Context c, CharSequence text, CharSequence version) {
        LinearLayout r = new LinearLayout(c);
        r.setOrientation(LinearLayout.HORIZONTAL);
        r.setBaselineAligned(true);
        // equal side columns: the title stays in the middle whatever the version's width
        r.addView(new View(c), new LinearLayout.LayoutParams(0, 1, 1));
        r.addView(title(c, text), new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT,
                LinearLayout.LayoutParams.WRAP_CONTENT));
        TextView v = new TextView(c);
        v.setText(version);
        v.setTypeface(Typeface.MONOSPACE);
        v.setTextSize(11);
        v.setTextColor(color(c, R.color.dim));
        v.setPadding(dp(c, 4), 0, 0, 0);
        r.addView(v, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1));
        return r;
    }

    /** One row of the lamp board: its lamp, the label (dim mono caps) and the value (ink mono). */
    static final class Row {
        final LinearLayout view;
        final View lamp;
        final TextView value;

        Row(Context c, String label) {
            view = new LinearLayout(c);
            view.setOrientation(LinearLayout.HORIZONTAL);
            view.setGravity(Gravity.TOP);
            view.setPadding(0, dp(c, 3), 0, dp(c, 3));
            lamp = new View(c);
            int s = dp(c, 12);
            LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(s, s);
            lp.setMargins(0, dp(c, 3), dp(c, 10), 0);
            view.addView(lamp, lp);
            TextView l = new TextView(c);
            l.setText(label);
            l.setTypeface(Typeface.MONOSPACE);
            l.setTextSize(13);
            l.setLetterSpacing(0.08f);
            l.setTextColor(color(c, R.color.dim));
            view.addView(l, new LinearLayout.LayoutParams(dp(c, 84), LinearLayout.LayoutParams.WRAP_CONTENT));
            value = new TextView(c);
            value.setTypeface(Typeface.MONOSPACE);
            value.setTextSize(13);
            value.setTextColor(color(c, R.color.ink));
            view.addView(value, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT,
                    LinearLayout.LayoutParams.WRAP_CONTENT));
        }

        /** colorId 0: no lamp (the empty slot keeps the column). */
        void set(int colorId, CharSequence v) {
            lamp.setBackground(colorId == 0 ? null : Brass.lamp(view.getContext(), colorId));
            if (!v.toString().contentEquals(value.getText()))
                value.setText(v);
        }
    }

    /** A section of a window: a full-width brass band with its name in dark serif capitals. */
    static void section(LinearLayout box, Context c, CharSequence text, boolean gap) {
        TextView t = new TextView(c);
        t.setText(text);
        t.setAllCaps(true);
        t.setTypeface(Typeface.SERIF, Typeface.BOLD);
        t.setLetterSpacing(0.25f);
        t.setTextSize(15);
        t.setTextColor(color(c, R.color.ink_dark));
        t.setBackgroundResource(R.drawable.plate);
        t.setGravity(Gravity.CENTER);
        int p = dp(c, 8);
        t.setPadding(p, p, p, p);
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT);
        lp.setMargins(0, dp(c, gap ? 28 : 8), 0, dp(c, 10));
        box.addView(t, lp);
    }

    static Button button(Context c, int text) {
        Button b = new Button(c, null, 0, R.style.Brass);
        b.setText(text);
        return b;
    }

    /** Layout parameters with the 6 dp margin of a brass button. */
    static LinearLayout.LayoutParams margins(Context c, int width, int height, float weight) {
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(width, height, weight);
        int m = dp(c, 6);
        lp.setMargins(m, m, m, m);
        return lp;
    }

    /**
     * The Show log window (brass 1.3.1, as pusher's): about `rows` lines high with its own
     * scrolling, so the page does not grow; oldest at the top; follows the last line unless
     * scrolled up; the remote's focus with the focus ring on a frame around it.
     */
    static final class LogWindow {
        final android.widget.FrameLayout view;    // the frame: display, focus ring
        private final android.widget.ScrollView scroll;
        private final TextView text;
        private String shown = "";

        LogWindow(Context c, int rows) {
            text = new TextView(c);
            text.setTypeface(Typeface.MONOSPACE);
            text.setTextSize(11);
            text.setTextColor(color(c, R.color.dim));
            int pad = dp(c, 12);
            text.setPadding(pad, pad, pad, pad);
            scroll = new android.widget.ScrollView(c) {
                @Override
                public boolean onInterceptTouchEvent(android.view.MotionEvent e) {
                    // the page scrolls too: keep the drag inside the window
                    if (e.getActionMasked() == android.view.MotionEvent.ACTION_DOWN)
                        getParent().requestDisallowInterceptTouchEvent(true);
                    return super.onInterceptTouchEvent(e);
                }
            };
            scroll.setFocusable(true);
            scroll.setScrollBarStyle(View.SCROLLBARS_INSIDE_OVERLAY);
            scroll.addView(text);
            // The ring on a frame around the scroll view: the foreground of a ScrollView
            // moves with its content; the frame takes the focused state of its child
            view = new android.widget.FrameLayout(c);
            view.setBackgroundResource(R.drawable.display);
            view.setAddStatesFromChildren(true);
            view.setForeground(c.getResources().getDrawable(R.drawable.focus_ring, null));
            view.addView(scroll, new android.widget.FrameLayout.LayoutParams(
                    LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.MATCH_PARENT));
            height = rows * text.getLineHeight() + 2 * pad;
        }

        /** The height for the layout parameters: `rows` lines and the padding. */
        final int height;

        void setText(String s) {
            if (s.equals(shown))
                return;
            boolean atEnd = scroll.getScrollY() + scroll.getHeight() >= text.getHeight() - dp(scroll.getContext(), 4);
            shown = s;
            text.setText(s);
            if (atEnd)
                scroll.post(() -> scroll.scrollTo(0, Math.max(0, text.getHeight() - scroll.getHeight())));
        }
    }
}
