package com.example.robotjoystick;

import android.app.Activity;
import android.graphics.Color;
import android.graphics.drawable.GradientDrawable;
import android.os.Bundle;
import android.view.Gravity;
import android.view.ViewGroup;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.SeekBar;
import android.widget.TextView;

public class MainActivity extends Activity {
    private static final int COLOR_BACKGROUND = Color.rgb(246, 248, 251);
    private static final int COLOR_SURFACE = Color.WHITE;
    private static final int COLOR_TEXT = Color.rgb(17, 24, 39);
    private static final int COLOR_MUTED = Color.rgb(75, 85, 99);
    private static final int COLOR_PRIMARY = Color.rgb(37, 99, 235);
    private static final int COLOR_DANGER = Color.rgb(220, 38, 38);

    private final RobotCommandClient commandClient = new RobotCommandClient();
    private LinearLayout controlContainer;
    private Button wheelTabButton;
    private Button armTabButton;
    private TextView statusText;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setBackgroundColor(COLOR_BACKGROUND);
        root.setPadding(22, 18, 22, 18);

        TextView title = new TextView(this);
        title.setText("ROS2 Robot Controller");
        title.setTextColor(COLOR_TEXT);
        title.setTextSize(21f);
        title.setGravity(Gravity.CENTER);
        title.setPadding(0, 2, 0, 10);
        root.addView(title, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT));

        WebView videoView = new WebView(this);
        videoView.setBackgroundColor(COLOR_SURFACE);
        WebSettings settings = videoView.getSettings();
        settings.setJavaScriptEnabled(false);
        settings.setLoadWithOverviewMode(true);
        settings.setUseWideViewPort(true);
        videoView.loadUrl(RobotConfig.VIDEO_URL);
        videoView.setBackground(makeRoundedBackground(COLOR_SURFACE, Color.rgb(226, 232, 240), 14f));
        root.addView(videoView, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                0,
                0.72f));

        LinearLayout tabRow = new LinearLayout(this);
        tabRow.setOrientation(LinearLayout.HORIZONTAL);
        tabRow.setGravity(Gravity.CENTER);
        tabRow.setPadding(0, 12, 0, 8);

        wheelTabButton = new Button(this);
        wheelTabButton.setText("Wheels");
        wheelTabButton.setAllCaps(false);
        wheelTabButton.setOnClickListener(view -> showWheelControls());
        tabRow.addView(wheelTabButton, new LinearLayout.LayoutParams(
                0,
                ViewGroup.LayoutParams.WRAP_CONTENT,
                1f));

        armTabButton = new Button(this);
        armTabButton.setText("Arm");
        armTabButton.setAllCaps(false);
        armTabButton.setOnClickListener(view -> showArmControls());
        tabRow.addView(armTabButton, new LinearLayout.LayoutParams(
                0,
                ViewGroup.LayoutParams.WRAP_CONTENT,
                1f));

        root.addView(tabRow, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT));

        controlContainer = new LinearLayout(this);
        controlContainer.setOrientation(LinearLayout.VERTICAL);
        controlContainer.setPadding(0, 4, 0, 0);
        root.addView(controlContainer, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                0,
                1.18f));

        statusText = new TextView(this);
        statusText.setText("Ready. Camera stays on while controls switch below.");
        statusText.setTextColor(COLOR_MUTED);
        statusText.setGravity(Gravity.CENTER);
        statusText.setTextSize(13f);
        statusText.setPadding(0, 8, 0, 4);
        root.addView(statusText, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT));

        showWheelControls();
        setContentView(root);
    }

    private void showWheelControls() {
        updateTabs(true);
        controlContainer.removeAllViews();

        JoystickView joystickView = new JoystickView(this);
        joystickView.setListener(new JoystickView.Listener() {
            @Override
            public void onJoystickMoved(float x, float y) {
                statusText.setText(String.format("x %.2f / y %.2f", x, y));
                commandClient.sendMove(x, y, MainActivity.this::setStatus);
            }

            @Override
            public void onJoystickReleased() {
                commandClient.sendStop(MainActivity.this::setStatus);
            }
        });
        controlContainer.addView(joystickView, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                0,
                1f));

        Button stopButton = new Button(this);
        stopButton.setText("EMERGENCY STOP");
        stopButton.setAllCaps(false);
        styleButton(stopButton, COLOR_DANGER, Color.WHITE);
        stopButton.setOnClickListener(view -> commandClient.sendStop(this::setStatus));
        controlContainer.addView(stopButton, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT));

        setStatus("Wheel control ready.");
    }

    private void showArmControls() {
        updateTabs(false);
        commandClient.sendStop(this::setStatus);
        controlContainer.removeAllViews();

        TextView armTitle = new TextView(this);
        armTitle.setText("Robot Arm Control");
        armTitle.setTextColor(COLOR_TEXT);
        armTitle.setTextSize(18f);
        armTitle.setGravity(Gravity.CENTER);
        armTitle.setPadding(0, 8, 0, 12);
        controlContainer.addView(armTitle, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT));

        addArmSlider("Base", 1, 500);
        addArmSlider("Shoulder", 2, 500);
        addArmSlider("Elbow", 3, 500);
        addArmSlider("Wrist Pitch", 4, 500);
        addArmSlider("Wrist Roll", 5, 500);
        addArmSlider("Gripper", 10, 500);

        LinearLayout buttonRow = new LinearLayout(this);
        buttonRow.setOrientation(LinearLayout.HORIZONTAL);

        Button homeButton = new Button(this);
        homeButton.setText("Arm Home");
        homeButton.setAllCaps(false);
        styleButton(homeButton, COLOR_PRIMARY, Color.WHITE);
        homeButton.setOnClickListener(view -> commandClient.sendArmAction("home", this::setStatus));
        buttonRow.addView(homeButton, new LinearLayout.LayoutParams(
                0,
                ViewGroup.LayoutParams.WRAP_CONTENT,
                1f));

        Button gripButton = new Button(this);
        gripButton.setText("Grip");
        gripButton.setAllCaps(false);
        styleButton(gripButton, Color.rgb(14, 165, 233), Color.WHITE);
        gripButton.setOnClickListener(view -> commandClient.sendArmAction("grip", this::setStatus));
        buttonRow.addView(gripButton, new LinearLayout.LayoutParams(
                0,
                ViewGroup.LayoutParams.WRAP_CONTENT,
                1f));

        controlContainer.addView(buttonRow, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT));

        setStatus("Arm control ready.");
    }

    private void addArmSlider(String label, int servoId, int progress) {
        TextView text = new TextView(this);
        text.setText(label + "  ID " + servoId + "  " + progress);
        text.setTextColor(COLOR_TEXT);
        text.setTextSize(14f);
        text.setPadding(0, 4, 0, 0);
        controlContainer.addView(text, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT));

        SeekBar slider = new SeekBar(this);
        slider.setMax(1000);
        slider.setProgress(progress);
        slider.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int value, boolean fromUser) {
                text.setText(label + "  ID " + servoId + "  " + value);
                if (fromUser) {
                    setStatus(label + " servo preview: " + value);
                }
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                commandClient.sendArmServo(
                        servoId,
                        seekBar.getProgress(),
                        getArmDurationMs(servoId),
                        MainActivity.this::setStatus);
            }
        });
        controlContainer.addView(slider, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT));
    }

    private int getArmDurationMs(int servoId) {
        switch (servoId) {
            case 2:
            case 3:
                return 2200;
            case 4:
            case 5:
                return 1200;
            case 10:
                return 700;
            default:
                return 1500;
        }
    }

    private void updateTabs(boolean wheelsSelected) {
        wheelTabButton.setText(wheelsSelected ? "Wheels *" : "Wheels");
        armTabButton.setText(wheelsSelected ? "Arm" : "Arm *");
        styleButton(wheelTabButton, wheelsSelected ? COLOR_PRIMARY : COLOR_SURFACE,
                wheelsSelected ? Color.WHITE : COLOR_PRIMARY);
        styleButton(armTabButton, wheelsSelected ? COLOR_SURFACE : COLOR_PRIMARY,
                wheelsSelected ? COLOR_PRIMARY : Color.WHITE);
        wheelTabButton.setEnabled(true);
        armTabButton.setEnabled(true);
    }

    @Override
    protected void onDestroy() {
        commandClient.sendStop(null);
        commandClient.shutdown();
        super.onDestroy();
    }

    private void setStatus(String status) {
        statusText.setText(status);
    }

    private GradientDrawable makeRoundedBackground(int fillColor, int strokeColor, float radius) {
        GradientDrawable drawable = new GradientDrawable();
        drawable.setColor(fillColor);
        drawable.setCornerRadius(radius);
        drawable.setStroke(2, strokeColor);
        return drawable;
    }

    private void styleButton(Button button, int backgroundColor, int textColor) {
        button.setTextColor(textColor);
        button.setBackground(makeRoundedBackground(backgroundColor, Color.rgb(191, 219, 254), 12f));
        button.setPadding(10, 8, 10, 8);
    }
}
