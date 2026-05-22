package com.example.robotjoystick;

final class RobotConfig {
    private RobotConfig() {
    }

    // Replace 192.168.0.50 with the Ubuntu server IP address.
    static final String VIDEO_URL = "http://192.168.1.6:5000/stream";
    static final String CONTROL_URL = "http://192.168.1.6:5000/control";
    static final String ARM_URL = "http://192.168.1.6:5000/arm";
    static final int COMMAND_INTERVAL_MS = 120;
    static final int NETWORK_TIMEOUT_MS = 1000;
}
