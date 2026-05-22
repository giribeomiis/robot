package com.example.robotjoystick;

import android.app.Activity;
import android.graphics.Color;
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
        root.setBackgroundColor(Color.rgb(17, 24, 39));
        root.setPadding(24, 24, 24, 24);

        TextView title = new TextView(this);
        title.setText("Robot Camera + Joystick");
        title.setTextColor(Color.WHITE);
        title.setTextSize(22f);
        title.setGravity(Gravity.CENTER);
        title.setPadding(0, 8, 0, 18);
        root.addView(title, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT));

        WebView videoView = new WebView(this);
        videoView.setBackgroundColor(Color.BLACK);
        WebSettings settings = videoView.getSettings();
        settings.setJavaScriptEnabled(false);
        settings.setLoadWithOverviewMode(true);
        settings.setUseWideViewPort(true);
        videoView.loadUrl(RobotConfig.VIDEO_URL);
        root.addView(videoView, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                0,
                1.15f));

        LinearLayout tabRow = new LinearLayout(this);
        tabRow.setOrientation(LinearLayout.HORIZONTAL);
        tabRow.setGravity(Gravity.CENTER);
        tabRow.setPadding(0, 18, 0, 10);

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
        root.addView(controlContainer, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                0,
                1f));

        statusText = new TextView(this);
        statusText.setText("Ready. Camera stays on while controls switch below.");
        statusText.setTextColor(Color.rgb(209, 213, 219));
        statusText.setGravity(Gravity.CENTER);
        statusText.setPadding(0, 12, 0, 12);
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
        armTitle.setTextColor(Color.WHITE);
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
        homeButton.setOnClickListener(view -> commandClient.sendArmAction("home", this::setStatus));
        buttonRow.addView(homeButton, new LinearLayout.LayoutParams(
                0,
                ViewGroup.LayoutParams.WRAP_CONTENT,
                1f));

        Button gripButton = new Button(this);
        gripButton.setText("Grip");
        gripButton.setAllCaps(false);
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
        text.setTextColor(Color.rgb(229, 231, 235));
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
        wheelTabButton.setEnabled(!wheelsSelected);
        armTabButton.setEnabled(wheelsSelected);
        wheelTabButton.setText(wheelsSelected ? "Wheels *" : "Wheels");
        armTabButton.setText(wheelsSelected ? "Arm" : "Arm *");
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
}
