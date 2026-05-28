import os
import threading
import time
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from flask import Flask, Response, jsonify, request
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import ServoPosition, ServosPosition
from sensor_msgs.msg import Image

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


app = Flask(__name__)


DASHBOARD_HTML = """
<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ROS2 Robot Control Room</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f5f7fb;
      --panel: #ffffff;
      --line: #dbe3ef;
      --text: #111827;
      --muted: #667085;
      --primary: #2563eb;
      --danger: #dc2626;
      --ok: #16a34a;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: Arial, "Noto Sans KR", sans-serif;
      background: var(--bg);
      color: var(--text);
    }
    header {
      padding: 18px 22px 10px;
      border-bottom: 1px solid var(--line);
      background: var(--panel);
    }
    h1 { margin: 0; font-size: 24px; }
    .subtitle { margin-top: 6px; color: var(--muted); font-size: 14px; }
    main {
      display: grid;
      grid-template-columns: minmax(420px, 1.55fr) minmax(320px, 0.9fr);
      gap: 16px;
      padding: 16px;
    }
    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 10px;
      padding: 14px;
    }
    .video {
      width: 100%;
      aspect-ratio: 16 / 9;
      object-fit: contain;
      background: #e5e7eb;
      border-radius: 8px;
      border: 1px solid var(--line);
    }
    .status-grid {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 10px;
      margin-top: 12px;
    }
    .status-card {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 10px;
      background: #f8fafc;
    }
    .label { color: var(--muted); font-size: 12px; }
    .value { margin-top: 4px; font-weight: 700; }
    h2 { margin: 0 0 12px; font-size: 18px; }
    .controls {
      display: grid;
      gap: 14px;
    }
    .drive-grid {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 8px;
      max-width: 320px;
      margin: 0 auto;
    }
    button {
      border: 1px solid #bfdbfe;
      border-radius: 8px;
      background: #ffffff;
      color: var(--primary);
      padding: 12px;
      font-size: 15px;
      font-weight: 700;
      cursor: pointer;
    }
    button.primary { background: var(--primary); color: #fff; }
    button.danger { background: var(--danger); color: #fff; border-color: #fecaca; }
    button:hover { filter: brightness(0.97); }
    .span-3 { grid-column: span 3; }
    .arm-row {
      display: grid;
      grid-template-columns: 112px 1fr 52px;
      gap: 8px;
      align-items: center;
      margin: 8px 0;
    }
    input[type="range"] { width: 100%; }
    .actions {
      display: grid;
      grid-template-columns: 1fr 1fr 1fr;
      gap: 8px;
      margin-top: 10px;
    }
    @media (max-width: 860px) {
      main { grid-template-columns: 1fr; }
      .status-grid { grid-template-columns: repeat(2, 1fr); }
    }
  </style>
</head>
<body>
  <header>
    <h1>ROS2 Robot Control Room</h1>
    <div class="subtitle">Camera + YOLO detection + wheel control + robot arm control</div>
  </header>
  <main>
    <section>
      <img class="video" src="/stream" alt="Robot camera stream">
      <div class="status-grid">
        <div class="status-card"><div class="label">Camera</div><div class="value" id="camera">-</div></div>
        <div class="status-card"><div class="label">Detection</div><div class="value" id="detect">-</div></div>
        <div class="status-card"><div class="label">People</div><div class="value" id="people">-</div></div>
        <div class="status-card"><div class="label">Drive</div><div class="value" id="drive">-</div></div>
      </div>
    </section>
    <section class="controls">
      <div>
        <h2>Wheels</h2>
        <div class="drive-grid">
          <div></div><button onmousedown="drive(0, 0.35)" onmouseup="stop()" ontouchstart="drive(0, 0.35)" ontouchend="stop()">Forward</button><div></div>
          <button onmousedown="drive(-0.45, 0)" onmouseup="stop()" ontouchstart="drive(-0.45, 0)" ontouchend="stop()">Left</button>
          <button class="danger" onclick="stop()">STOP</button>
          <button onmousedown="drive(0.45, 0)" onmouseup="stop()" ontouchstart="drive(0.45, 0)" ontouchend="stop()">Right</button>
          <div></div><button onmousedown="drive(0, -0.35)" onmouseup="stop()" ontouchstart="drive(0, -0.35)" ontouchend="stop()">Back</button><div></div>
        </div>
      </div>
      <div>
        <h2>Robot Arm</h2>
        <div id="arm-sliders"></div>
        <div class="actions">
          <button class="primary" onclick="armAction('home')">Home</button>
          <button onclick="armAction('grip')">Grip</button>
          <button onclick="armAction('release')">Release</button>
        </div>
      </div>
      <div>
        <h2>Detection</h2>
        <div class="actions">
          <button onclick="detect(true)">YOLO On</button>
          <button onclick="detect(false)">YOLO Off</button>
          <button onclick="refreshStatus()">Refresh</button>
        </div>
      </div>
    </section>
  </main>
  <script>
    const servos = [
      ["Base", 1, 500, 1500],
      ["Shoulder", 2, 500, 2200],
      ["Elbow", 3, 500, 2200],
      ["Wrist Pitch", 4, 500, 1200],
      ["Wrist Roll", 5, 500, 1200],
      ["Gripper", 10, 500, 700],
    ];

    function request(path) {
      return fetch(path).then(r => r.json()).catch(err => ({ ok: false, error: String(err) }));
    }

    function drive(x, y) {
      request(`/control?action=move&x=${x}&y=${y}`).then(refreshStatus);
    }

    function stop() {
      request("/control?action=stop").then(refreshStatus);
    }

    function armServo(id, position, duration) {
      request(`/arm?action=servo&id=${id}&position=${position}&duration=${duration}`).then(refreshStatus);
    }

    function armAction(action) {
      request(`/arm?action=${action}`).then(refreshStatus);
    }

    function detect(enabled) {
      request(`/detect?enabled=${enabled}`).then(refreshStatus);
    }

    function buildArmSliders() {
      const root = document.getElementById("arm-sliders");
      root.innerHTML = "";
      for (const [label, id, initial, duration] of servos) {
        const row = document.createElement("div");
        row.className = "arm-row";
        row.innerHTML = `
          <div>${label}</div>
          <input type="range" min="0" max="1000" value="${initial}">
          <div>${initial}</div>
        `;
        const slider = row.querySelector("input");
        const value = row.querySelector("div:last-child");
        slider.addEventListener("input", () => value.textContent = slider.value);
        slider.addEventListener("change", () => armServo(id, slider.value, duration));
        root.appendChild(row);
      }
    }

    function refreshStatus() {
      request("/status").then(data => {
        document.getElementById("camera").textContent = data.camera_frame_ready ? "Ready" : "Waiting";
        document.getElementById("detect").textContent = data.detection_enabled ? "On" : "Off";
        document.getElementById("people").textContent = data.person_count ?? 0;
        document.getElementById("drive").textContent = `${data.action || "-"} x=${data.x ?? 0} y=${data.y ?? 0}`;
      });
    }

    buildArmSliders();
    refreshStatus();
    setInterval(refreshStatus, 1000);
  </script>
</body>
</html>
"""


class RobotState:
    action = "stop"
    x = 0.0
    y = 0.0
    updated_at = time.time()
    person_count = 0
    detection_enabled = True


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("android_robot_http_bridge")
        self.bridge = CvBridge()
        self.latest_jpeg: Optional[bytes] = None
        self.latest_people = []
        self.frame_count = 0
        self.yolo = None
        self.yolo_confidence = float(os.getenv("YOLO_CONFIDENCE", "0.35"))
        self.yolo_image_size = int(os.getenv("YOLO_IMAGE_SIZE", "416"))
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        if YOLO is not None:
            model_path = os.getenv("YOLO_MODEL", "yolov8n.pt")
            self.yolo = YOLO(model_path)
            self.get_logger().info(f"Loaded YOLO person detector: {model_path}")
        else:
            self.get_logger().warn("ultralytics is not installed; falling back to OpenCV HOG.")
        image_topic = self.declare_parameter("image_topic", "/camera/color/image_raw").value
        cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/controller/cmd_vel").value
        arm_topic = self.declare_parameter(
            "arm_topic",
            "/ros_robot_controller/bus_servo/set_position"
        ).value
        self.create_subscription(Image, image_topic, self.on_image, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.arm_publisher = self.create_publisher(ServosPosition, arm_topic, 10)
        self.get_logger().info(f"Streaming ROS image topic: {image_topic}")
        self.get_logger().info(f"Publishing robot commands to: {cmd_vel_topic}")
        self.get_logger().info(f"Publishing arm commands to: {arm_topic}")

    def on_image(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if state.detection_enabled:
            self.frame_count += 1
            if self.frame_count % 5 == 0:
                self.latest_people = self.detect_people(frame)
                state.person_count = len(self.latest_people)
            self.draw_people(frame, self.latest_people)

        ok, encoded = cv2.imencode(".jpg", frame)
        if ok:
            self.latest_jpeg = encoded.tobytes()

    def detect_people(self, frame):
        if self.yolo is not None:
            return self.detect_people_yolo(frame)
        return self.detect_people_hog(frame)

    def detect_people_yolo(self, frame):
        results = self.yolo.predict(
            source=frame,
            classes=[0],
            conf=self.yolo_confidence,
            imgsz=self.yolo_image_size,
            verbose=False,
        )

        people = []
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes.xyxy.cpu().numpy():
                x1, y1, x2, y2 = box
                people.append((int(x1), int(y1), int(x2 - x1), int(y2 - y1)))
        return people

    def detect_people_hog(self, frame):
        height, width = frame.shape[:2]
        target_width = 480
        scale = width / target_width if width > target_width else 1.0

        if scale > 1.0:
            target_height = int(height / scale)
            detect_frame = cv2.resize(frame, (target_width, target_height))
        else:
            detect_frame = frame

        boxes, _ = self.hog.detectMultiScale(
            detect_frame,
            winStride=(8, 8),
            padding=(8, 8),
            scale=1.05,
        )

        people = []
        for x, y, w, h in boxes:
            people.append((
                int(x * scale),
                int(y * scale),
                int(w * scale),
                int(h * scale),
            ))
        return people

    def draw_people(self, frame, people) -> None:
        for index, (x, y, w, h) in enumerate(people, start=1):
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 80), 2)
            cv2.putText(
                frame,
                f"person {index}",
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 80),
                2,
            )

    def publish_move(self, x: float, y: float) -> None:
        msg = Twist()
        msg.linear.x = y * 0.2
        msg.angular.z = -x * 0.5
        self.cmd_vel_publisher.publish(msg)

    def publish_stop(self) -> None:
        self.cmd_vel_publisher.publish(Twist())

    def publish_servo(self, servo_id: int, position: int, duration: float) -> None:
        command = ServosPosition()
        command.duration = duration

        servo = ServoPosition()
        servo.id = servo_id
        servo.position = position
        command.position = [servo]

        self.arm_publisher.publish(command)

    def publish_arm_home(self) -> None:
        command = ServosPosition()
        command.duration = 1.0
        for servo_id, position in [(1, 500), (2, 500), (3, 500), (4, 500), (5, 500), (10, 500)]:
            servo = ServoPosition()
            servo.id = servo_id
            servo.position = position
            command.position.append(servo)
        self.arm_publisher.publish(command)


state = RobotState()
camera_node: Optional[CameraNode] = None


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
        x = 0.0
        y = 0.0
        if camera_node:
            camera_node.publish_stop()
    else:
        action = "move"
        if camera_node:
            camera_node.publish_move(x, y)

    state.action = action
    state.x = x
    state.y = y
    state.updated_at = time.time()
    return jsonify({"ok": True, "action": action, "x": x, "y": y})


@app.get("/arm")
def arm():
    action = request.args.get("action", "servo")

    if camera_node is None:
        return jsonify({"ok": False, "error": "ROS node is not ready"}), 503

    if action == "home":
        camera_node.publish_arm_home()
        return jsonify({"ok": True, "action": "home"})

    if action == "grip":
        camera_node.publish_servo(10, 300, 0.5)
        return jsonify({"ok": True, "action": "grip", "id": 10, "position": 300})

    if action == "release":
        camera_node.publish_servo(10, 700, 0.5)
        return jsonify({"ok": True, "action": "release", "id": 10, "position": 700})

    servo_id = int(parse_float("id", 1))
    position = int(parse_float("position", 500))
    duration_ms = parse_float("duration", 500)
    position = max(0, min(1000, position))
    duration = max(0.05, duration_ms / 1000.0)

    camera_node.publish_servo(servo_id, position, duration)
    return jsonify(
        {
            "ok": True,
            "action": "servo",
            "id": servo_id,
            "position": position,
            "duration": duration,
        }
    )


@app.get("/status")
def status():
    has_frame = camera_node is not None and camera_node.latest_jpeg is not None
    return jsonify(
        {
            "ok": True,
            "action": state.action,
            "x": state.x,
            "y": state.y,
            "updated_at": state.updated_at,
            "camera_frame_ready": has_frame,
            "person_count": state.person_count,
            "detection_enabled": state.detection_enabled,
        }
    )


@app.get("/detect")
def detect():
    enabled = request.args.get("enabled")
    if enabled is not None:
        state.detection_enabled = enabled.lower() in ("1", "true", "yes", "on")
    return jsonify(
        {
            "ok": True,
            "detection_enabled": state.detection_enabled,
            "person_count": state.person_count,
        }
    )


def mjpeg_frames():
    while True:
        frame = camera_node.latest_jpeg if camera_node else None
        if frame is None:
            time.sleep(0.1)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + frame
            + b"\r\n"
        )
        time.sleep(0.03)


@app.get("/stream")
def stream():
    return Response(mjpeg_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.get("/")
def dashboard():
    return Response(DASHBOARD_HTML, mimetype="text/html")


def spin_ros():
    rclpy.spin(camera_node)


if __name__ == "__main__":
    rclpy.init()
    camera_node = CameraNode()
    threading.Thread(target=spin_ros, daemon=True).start()
    app.run(host="0.0.0.0", port=5000, threaded=True)
