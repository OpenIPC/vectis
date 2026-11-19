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

    /**
     * A check box or radio button in brass (1.4.0): the brass drawable, no tint, the focus ring
     * around the row, 6dp to the text and room for the ring after it.  For those made outside the theme (the settings window).
     */
    static void check(android.widget.CompoundButton b) {
        Context c = b.getContext();
        b.setButtonDrawable(b instanceof android.widget.RadioButton ? R.drawable.brass_radio : R.drawable.brass_check);
        b.setButtonTintList(null);
        b.setBackgroundResource(R.drawable.focus_ring);
        b.setPadding(dp(c, 6), b.getPaddingTop(), dp(c, 6), b.getPaddingBottom());   // the ring clears the last letter
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
     * scrolled up, keeping a reader's lines when old ones leave the top; the remote's focus with
     * the focus ring on a frame around it.  The same as camex's (#193 there).
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
            // the ring shows the focus: no system highlight over the text (Android 8+)
            if (android.os.Build.VERSION.SDK_INT >= 26) scroll.setDefaultFocusHighlightEnabled(false);
            scroll.setScrollBarStyle(View.SCROLLBARS_INSIDE_OVERLAY);
            scroll.addView(text);
            // The ring on a frame around the scroll view: the foreground of a
            // ScrollView moves with its content; the frame takes the focused
            // state of its child (FrameLayout foreground works from API 1)
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

        // Whether the window follows the end: true at the end, false once the user scrolls up;
        // kept by the scroll listener, so a scroll posted earlier sees a later drag (#193 review)
        private boolean follow = true;

        private boolean atEnd() {
            return scroll.getScrollY() + scroll.getHeight() >= text.getHeight() - dp(scroll.getContext(), 4);
        }

        // The offset the window scrolled itself to: a scroll event at it is not the user's
        private int autoTarget = -1;

        private void scrollToEnd() {
            autoTarget = Math.max(0, text.getHeight() - scroll.getHeight());
            scroll.scrollTo(0, autoTarget);
        }

        private void toEnd() {
            scroll.post(() -> {
                if (follow) scrollToEnd();
            });
        }

        /** Shown again (Show log on): at the end, as the rule wants. */
        void shown() {
            follow = true;
            toEnd();
        }

        /** The log and its count of lines gone from the top (LogBuffer.snapshot()). */
        void setText(String s, long removed) {
            if (s.equals(shown) && removed == shownRemoved)
                return;
            if (!listening) {
                listening = true;
                // the user's scroll decides whether to follow; the window's own does not
                scroll.getViewTreeObserver().addOnScrollChangedListener(() -> {
                    if (scroll.getHeight() == 0 || scroll.getScrollY() == autoTarget) return;
                    follow = atEnd();
                });
                // new lines laid out: to the end, once the text has its new height
                text.addOnLayoutChangeListener((v, l, t, r, b, ol, ot, or, ob) -> {
                    if (follow && b - t != ob - ot) scrollToEnd();
                });
            }
            // Lines gone from the top (the buffer is full): a reader scrolled up stays on the
            // same lines.  How many: the buffer's count, not a comparison of the text (equal
            // lines repeat); how far: the layout of their text, as long lines wrap (brass 1.3.1)
            long gone = removed - shownRemoved;
            int dy = 0;
            if (!follow && gone > 0 && text.getLayout() != null) {
                int chars = charsOfLines(shown, gone);
                android.text.Layout lay = text.getLayout();
                dy = lay.getLineTop(lay.getLineForOffset(Math.min(chars, shown.length())));
            }
            shown = s;
            shownRemoved = removed;
            text.setText(s);
            if (follow) {
                toEnd();
            } else if (dy > 0) {
                int d = dy;
                scroll.post(() -> scroll.scrollTo(0, Math.max(0, scroll.getScrollY() - d)));
            }
        }

        private long shownRemoved;

        private boolean listening;

        /** The length of the first n lines of {@code text} with their line ends (all of it if fewer). */
        static int charsOfLines(String text, long n) {
            int at = 0;
            for (long k = 0; k < n; k++) {
                int nl = text.indexOf('\n', at);
                if (nl < 0)
                    return text.length();
                at = nl + 1;
            }
            return at;
        }
    }
}
