package com.example.robotjoystick;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RadialGradient;
import android.graphics.Shader;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

public class JoystickView extends View {
    interface Listener {
        void onJoystickMoved(float x, float y);
        void onJoystickReleased();
    }

    private final Paint basePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint ringPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint knobPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint crossPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    private Listener listener;
    private float knobX;
    private float knobY;
    private boolean tracking;

    public JoystickView(Context context) {
        super(context);
        init();
    }

    public JoystickView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    private void init() {
        setBackgroundColor(Color.TRANSPARENT);
        basePaint.setColor(Color.rgb(31, 41, 55));
        ringPaint.setStyle(Paint.Style.STROKE);
        ringPaint.setStrokeWidth(6f);
        ringPaint.setColor(Color.rgb(96, 165, 250));
        knobPaint.setColor(Color.rgb(59, 130, 246));
        crossPaint.setColor(Color.argb(120, 229, 231, 235));
        crossPaint.setStrokeWidth(3f);
    }

    void setListener(Listener listener) {
        this.listener = listener;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        float centerX = getWidth() / 2f;
        float centerY = getHeight() / 2f;
        float radius = Math.min(getWidth(), getHeight()) * 0.38f;
        float knobRadius = radius * 0.34f;

        float drawKnobX = tracking ? knobX : centerX;
        float drawKnobY = tracking ? knobY : centerY;

        basePaint.setShader(new RadialGradient(centerX, centerY, radius, Color.rgb(55, 65, 81),
                Color.rgb(17, 24, 39), Shader.TileMode.CLAMP));
        canvas.drawCircle(centerX, centerY, radius, basePaint);
        basePaint.setShader(null);
        canvas.drawCircle(centerX, centerY, radius, ringPaint);
        canvas.drawLine(centerX - radius, centerY, centerX + radius, centerY, crossPaint);
        canvas.drawLine(centerX, centerY - radius, centerX, centerY + radius, crossPaint);
        canvas.drawCircle(drawKnobX, drawKnobY, knobRadius, knobPaint);
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        float centerX = getWidth() / 2f;
        float centerY = getHeight() / 2f;
        float radius = Math.min(getWidth(), getHeight()) * 0.38f;

        if (event.getAction() == MotionEvent.ACTION_UP || event.getAction() == MotionEvent.ACTION_CANCEL) {
            tracking = false;
            knobX = centerX;
            knobY = centerY;
            invalidate();
            if (listener != null) {
                listener.onJoystickReleased();
            }
            return true;
        }

        tracking = true;
        float dx = event.getX() - centerX;
        float dy = event.getY() - centerY;
        float distance = (float) Math.sqrt(dx * dx + dy * dy);
        if (distance > radius) {
            dx = dx / distance * radius;
            dy = dy / distance * radius;
        }

        knobX = centerX + dx;
        knobY = centerY + dy;
        invalidate();

        if (listener != null) {
            listener.onJoystickMoved(dx / radius, -dy / radius);
        }
        return true;
    }
}
