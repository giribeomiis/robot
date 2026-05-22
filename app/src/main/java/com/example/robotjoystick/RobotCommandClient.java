package com.example.robotjoystick;

import android.os.Handler;
import android.os.Looper;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLEncoder;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

final class RobotCommandClient {
    interface StatusListener {
        void onStatus(String status);
    }

    private final ExecutorService executor = Executors.newSingleThreadExecutor();
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private long lastSentAt;

    void sendMove(float x, float y, StatusListener listener) {
        long now = System.currentTimeMillis();
        if (now - lastSentAt < RobotConfig.COMMAND_INTERVAL_MS) {
            return;
        }
        lastSentAt = now;
        send("move", x, y, listener);
    }

    void sendStop(StatusListener listener) {
        lastSentAt = 0;
        send("stop", 0f, 0f, listener);
    }

    void sendArmServo(int servoId, int position, int durationMs, StatusListener listener) {
        String url = RobotConfig.ARM_URL
                + "?action=servo"
                + "&id=" + encode(Integer.toString(servoId))
                + "&position=" + encode(Integer.toString(position))
                + "&duration=" + encode(Integer.toString(durationMs));
        sendUrl(url, "servo " + servoId + " -> " + position, listener);
    }

    void sendArmAction(String action, StatusListener listener) {
        String url = RobotConfig.ARM_URL + "?action=" + encode(action);
        sendUrl(url, "arm " + action, listener);
    }

    void shutdown() {
        executor.shutdownNow();
    }

    private void send(String action, float x, float y, StatusListener listener) {
        sendUrl(buildCommandUrl(action, x, y), action, listener);
    }

    private void sendUrl(String urlText, String label, StatusListener listener) {
        executor.execute(() -> {
            String status;
            try {
                URL url = new URL(urlText);
                HttpURLConnection connection = (HttpURLConnection) url.openConnection();
                connection.setConnectTimeout(RobotConfig.NETWORK_TIMEOUT_MS);
                connection.setReadTimeout(RobotConfig.NETWORK_TIMEOUT_MS);
                connection.setRequestMethod("GET");
                int code = connection.getResponseCode();
                connection.disconnect();
                status = label + " sent (" + code + ")";
            } catch (IOException e) {
                status = "robot offline: " + e.getMessage();
            }

            if (listener != null) {
                String finalStatus = status;
                mainHandler.post(() -> listener.onStatus(finalStatus));
            }
        });
    }

    private String buildCommandUrl(String action, float x, float y) {
        String separator = RobotConfig.CONTROL_URL.contains("?") ? "&" : "?";
        return RobotConfig.CONTROL_URL
                + separator
                + "action=" + encode(action)
                + "&x=" + encode(String.format(Locale.US, "%.2f", x))
                + "&y=" + encode(String.format(Locale.US, "%.2f", y));
    }

    private static String encode(String value) {
        try {
            return URLEncoder.encode(value, "UTF-8");
        } catch (IOException e) {
            return value;
        }
    }
}
