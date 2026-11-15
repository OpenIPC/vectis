/*
 * Copyright (c) OpenIPC  https://openipc.org  MIT License
 *
 * Brass.java — the brass look of the app screen (brand brass 1.1.0, after
 * pusher and microsnander): heading, plate, panel, display, lamp and brass
 * buttons, built in code like the screen itself.  Colours come from
 * res/values/colors.xml.
 */
package org.openipc.vectis;

import android.content.Context;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.os.Build;
import android.util.TypedValue;
import android.view.Gravity;
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
}
