import os
import time
from dataclasses import dataclass
from typing import Generator, Optional

import cv2
from flask import Flask, Response, jsonify, request


app = Flask(__name__)


@dataclass
class RobotState:
    action: str = "stop"
    x: float = 0.0
    y: float = 0.0
    updated_at: float = time.time()


state = RobotState()


class MotorController:
    def __init__(self) -> None:
        self.use_gpio = os.getenv("ROBOT_USE_GPIO", "0") == "1"
        self.left_pwm = None
        self.right_pwm = None
        if self.use_gpio:
            self._setup_gpio()

    def _setup_gpio(self) -> None:
        try:
            from gpiozero import Motor, PWMOutputDevice

            left_forward = int(os.getenv("LEFT_FORWARD_PIN", "17"))
            left_backward = int(os.getenv("LEFT_BACKWARD_PIN", "27"))
            right_forward = int(os.getenv("RIGHT_FORWARD_PIN", "22"))
            right_backward = int(os.getenv("RIGHT_BACKWARD_PIN", "23"))
            left_enable = int(os.getenv("LEFT_ENABLE_PIN", "18"))
            right_enable = int(os.getenv("RIGHT_ENABLE_PIN", "13"))

            self.left_motor = Motor(forward=left_forward, backward=left_backward)
            self.right_motor = Motor(forward=right_forward, backward=right_backward)
            self.left_pwm = PWMOutputDevice(left_enable)
            self.right_pwm = PWMOutputDevice(right_enable)
        except Exception as exc:
            self.use_gpio = False
            print(f"GPIO disabled: {exc}")

    def move(self, x: float, y: float) -> None:
        x = clamp(x)
        y = clamp(y)
        left_speed = clamp(y + x)
        right_speed = clamp(y - x)

        if not self.use_gpio:
            print(f"move x={x:.2f} y={y:.2f} left={left_speed:.2f} right={right_speed:.2f}")
            return

        self._drive_side(self.left_motor, self.left_pwm, left_speed)
        self._drive_side(self.right_motor, self.right_pwm, right_speed)

    def stop(self) -> None:
        if not self.use_gpio:
            print("stop")
            return

        self.left_motor.stop()
        self.right_motor.stop()
        self.left_pwm.value = 0
        self.right_pwm.value = 0

    @staticmethod
    def _drive_side(motor, pwm, speed: float) -> None:
        pwm.value = abs(speed)
        if speed > 0:
            motor.forward()
        elif speed < 0:
            motor.backward()
        else:
            motor.stop()


motor_controller = MotorController()


def clamp(value: float) -> float:
    return max(-1.0, min(1.0, value))


def parse_float(name: str, default: float = 0.0) -> float:
    try:
        return float(request.args.get(name, default))
    except ValueError:
        return default


@app.get("/control")
def control():
    action = request.args.get("action", "move")
    x = clamp(parse_float("x"))
    y = clamp(parse_float("y"))

    if action == "stop":
        motor_controller.stop()
        x = 0.0
        y = 0.0
    else:
        action = "move"
        motor_controller.move(x, y)

    state.action = action
    state.x = x
    state.y = y
    state.updated_at = time.time()
    return jsonify({"ok": True, "action": action, "x": x, "y": y})


@app.get("/status")
def status():
    return jsonify(
        {
            "ok": True,
            "action": state.action,
            "x": state.x,
            "y": state.y,
            "updated_at": state.updated_at,
            "gpio_enabled": motor_controller.use_gpio,
        }
    )


def open_camera() -> Optional[cv2.VideoCapture]:
    source = os.getenv("ROBOT_CAMERA_SOURCE", "0")
    if source.isdigit():
        source = int(source)
    camera = cv2.VideoCapture(source)
    if camera.isOpened():
        return camera
    camera.release()
    return None


def mjpeg_frames() -> Generator[bytes, None, None]:
    camera = open_camera()
    if camera is None:
        yield b"--frame\r\nContent-Type: text/plain\r\n\r\ncamera unavailable\r\n"
        return

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                time.sleep(0.05)
                continue

            ok, encoded = cv2.imencode(".jpg", frame)
            if not ok:
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + encoded.tobytes()
                + b"\r\n"
            )
    finally:
        camera.release()


@app.get("/stream")
def stream():
    return Response(mjpeg_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    host = os.getenv("ROBOT_HOST", "0.0.0.0")
    port = int(os.getenv("ROBOT_PORT", "5000"))
    app.run(host=host, port=port, threaded=True)
