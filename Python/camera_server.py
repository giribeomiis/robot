import os
import time
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from flask import Flask, Response, jsonify, request
from geometry_msgs.msg import Twist
from rclpy.node import Node
from ros_robot_controller_msgs.msg import ServoPosition, ServosPosition
from sensor_msgs.msg import Image

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


app = Flask(__name__)
latest_jpeg = None
node = None


DASHBOARD_HTML = """
<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Control Room</title>
  <style>
    body { margin: 0; font-family: Arial, sans-serif; background: #f5f7fb; color: #111827; }
    header { padding: 16px 22px; background: #fff; border-bottom: 1px solid #dbe3ef; }
    main { display: grid; grid-template-columns: 1.5fr 1fr; gap: 16px; padding: 16px; }
    section { background: #fff; border: 1px solid #dbe3ef; border-radius: 10px; padding: 14px; }
    img { width: 100%; aspect-ratio: 16 / 9; object-fit: contain; background: #e5e7eb; border-radius: 8px; }
    button { margin: 4px; padding: 12px; border-radius: 8px; border: 1px solid #bfdbfe; background: #fff; color: #2563eb; font-weight: 700; cursor: pointer; }
    button.danger { background: #dc2626; color: #fff; }
    .grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; max-width: 320px; }
    .arm-row { display: grid; grid-template-columns: 100px 1fr 50px; gap: 8px; align-items: center; margin: 8px 0; }
    .status { margin-top: 8px; color: #475569; line-height: 1.5; }
    @media (max-width: 900px) { main { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <header>
    <h1>Robot Control Room</h1>
    <div>Camera / YOLO / Wheels / Robot Arm</div>
  </header>

  <main>
    <section>
      <img src="/stream" alt="camera">
      <p class="status" id="status">status loading...</p>
    </section>

    <section>
      <h2>Wheels</h2>
      <div class="grid">
        <div></div>
        <button onmousedown="drive(0,0.35)" onmouseup="stop()" ontouchstart="drive(0,0.35)" ontouchend="stop()">Forward</button>
        <div></div>

        <button onmousedown="drive(-0.45,0)" onmouseup="stop()" ontouchstart="drive(-0.45,0)" ontouchend="stop()">Left</button>
        <button class="danger" onclick="stop()">STOP</button>
        <button onmousedown="drive(0.45,0)" onmouseup="stop()" ontouchstart="drive(0.45,0)" ontouchend="stop()">Right</button>

        <div></div>
        <button onmousedown="drive(0,-0.35)" onmouseup="stop()" ontouchstart="drive(0,-0.35)" ontouchend="stop()">Back</button>
        <div></div>
      </div>

      <h2>Robot Arm</h2>
      <div id="arm"></div>
      <button onclick="armAction('home')">Home</button>
      <button onclick="armAction('grip')">Grip</button>
      <button onclick="armAction('release')">Release</button>

      <h2>YOLO</h2>
      <button onclick="detect(true)">Detection On</button>
      <button onclick="detect(false)">Detection Off</button>
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

    function req(path) {
      return fetch(path).then(r => r.json()).catch(e => ({ok:false, error:String(e)}));
    }

    function drive(x, y) {
      req(`/control?action=move&x=${x}&y=${y}`).then(updateStatus);
    }

    function stop() {
      req("/control?action=stop").then(updateStatus);
    }

    function armServo(id, position, duration) {
      req(`/arm?action=servo&id=${id}&position=${position}&duration=${duration}`).then(updateStatus);
    }

    function armAction(action) {
      req(`/arm?action=${action}`).then(updateStatus);
    }

    function detect(enabled) {
      req(`/detect?enabled=${enabled}`).then(updateStatus);
    }

    function buildArm() {
      const root = document.getElementById("arm");
      for (const [label, id, value, duration] of servos) {
        const row = document.createElement("div");
        row.className = "arm-row";
        row.innerHTML = `<span>${label}</span><input type="range" min="0" max="1000" value="${value}"><b>${value}</b>`;
        const slider = row.querySelector("input");
        const text = row.querySelector("b");
        slider.addEventListener("input", () => text.textContent = slider.value);
        slider.addEventListener("change", () => armServo(id, slider.value, duration));
        root.appendChild(row);
      }
    }

    function updateStatus() {
      req("/status").then(data => {
        document.getElementById("status").innerHTML =
          `camera=${data.camera_frame_ready ? "ready" : "waiting"}<br>` +
          `yolo=${data.yolo_loaded ? "loaded" : "not loaded"} / detection=${data.detection_enabled ? "on" : "off"}<br>` +
          `people=${data.person_count}`;
      });
    }

    buildArm();
    updateStatus();
    setInterval(updateStatus, 1000);
  </script>
</body>
</html>
"""


class RobotBridge(Node):
    def __init__(self):
        super().__init__("android_robot_bridge")

        self.bridge = CvBridge()
        self.latest_people = []
        self.frame_count = 0
        self.detection_enabled = True

        self.yolo = None
        self.yolo_confidence = float(os.getenv("YOLO_CONFIDENCE", "0.35"))
        self.yolo_image_size = int(os.getenv("YOLO_IMAGE_SIZE", "416"))

        if YOLO is not None:
            model_path = os.getenv("YOLO_MODEL", "yolov8n.pt")
            self.yolo = YOLO(model_path)
            self.get_logger().info(f"Loaded YOLO model: {model_path}")
        else:
            self.get_logger().warn("ultralytics is not installed. Detection disabled.")

        self.create_subscription(
            Image,
            "/depth_cam/rgb/image_raw",
            self.on_image,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            "/controller/cmd_vel",
            10
        )

        self.arm_pub = self.create_publisher(
            ServosPosition,
            "/ros_robot_controller/bus_servo/set_position",
            10
        )

        self.get_logger().info("Streaming /depth_cam/rgb/image_raw")
        self.get_logger().info("Publishing /controller/cmd_vel")
        self.get_logger().info("Publishing /ros_robot_controller/bus_servo/set_position")

    def on_image(self, msg):
        global latest_jpeg

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if self.detection_enabled and self.yolo is not None:
            self.frame_count += 1
            if self.frame_count % 3 == 0:
                self.latest_people = self.detect_people(frame)
            self.draw_people(frame, self.latest_people)

        ok, encoded = cv2.imencode(".jpg", frame)

        if ok:
            latest_jpeg = encoded.tobytes()

    def detect_people(self, frame):
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
                people.append((
                    int(x1),
                    int(y1),
                    int(x2 - x1),
                    int(y2 - y1),
                ))

        return people

    def draw_people(self, frame, people):
        for index, (x, y, w, h) in enumerate(people, start=1):
            cv2.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                (0, 255, 80),
                2
            )

            cv2.putText(
                frame,
                f"person {index}",
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 80),
                2,
            )

    def move(self, x, y):
        msg = Twist()
        msg.linear.x = y * 0.2
        msg.angular.z = -x * 0.5
        self.cmd_pub.publish(msg)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def servo(self, servo_id, position, duration):
        msg = ServosPosition()
        msg.duration = duration

        servo = ServoPosition()
        servo.id = servo_id
        servo.position = position

        msg.position = [servo]
        self.arm_pub.publish(msg)

    def arm_home(self):
        msg = ServosPosition()
        msg.duration = 1.0

        for servo_id, position in [
            (1, 500),
            (2, 500),
            (3, 500),
            (4, 500),
            (5, 500),
            (10, 500),
        ]:
            servo = ServoPosition()
            servo.id = servo_id
            servo.position = position
            msg.position.append(servo)

        self.arm_pub.publish(msg)


@app.get("/")
def dashboard():
    return Response(DASHBOARD_HTML, mimetype="text/html")


@app.get("/control")
def control():
    action = request.args.get("action", "move")

    x = float(request.args.get("x", 0))
    y = float(request.args.get("y", 0))

    x = max(-1.0, min(1.0, x))
    y = max(-1.0, min(1.0, y))

    if action == "stop":
        node.stop()
        return jsonify({"ok": True, "action": "stop", "x": 0, "y": 0})

    node.move(x, y)
    return jsonify({"ok": True, "action": "move", "x": x, "y": y})


@app.get("/arm")
def arm():
    action = request.args.get("action", "servo")

    if action == "home":
        node.arm_home()
        return jsonify({"ok": True, "action": "home"})

    if action == "grip":
        node.servo(10, 300, 0.5)
        return jsonify({"ok": True, "action": "grip", "id": 10, "position": 300})

    if action == "release":
        node.servo(10, 700, 0.5)
        return jsonify({"ok": True, "action": "release", "id": 10, "position": 700})

    servo_id = int(request.args.get("id", 1))
    position = int(request.args.get("position", 500))
    duration_ms = float(request.args.get("duration", 500))

    servo_id = max(1, min(253, servo_id))
    position = max(0, min(1000, position))
    duration = max(0.05, duration_ms / 1000.0)

    node.servo(servo_id, position, duration)

    return jsonify({
        "ok": True,
        "action": "servo",
        "id": servo_id,
        "position": position,
        "duration": duration
    })


@app.get("/detect")
def detect():
    enabled = request.args.get("enabled")

    if enabled is not None:
        node.detection_enabled = enabled.lower() in ("1", "true", "yes", "on")

    return jsonify({
        "ok": True,
        "detection_enabled": node.detection_enabled,
        "person_count": len(node.latest_people),
        "yolo_loaded": node.yolo is not None,
    })


def frames():
    while True:
        if latest_jpeg is None:
            time.sleep(0.1)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            latest_jpeg +
            b"\r\n"
        )

        time.sleep(0.03)


@app.get("/stream")
def stream():
    return Response(
        frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.get("/status")
def status():
    return jsonify({
        "ok": True,
        "camera_frame_ready": latest_jpeg is not None,
        "detection_enabled": node.detection_enabled if node else False,
        "person_count": len(node.latest_people) if node else 0,
        "yolo_loaded": node.yolo is not None if node else False,
    })


def spin_ros():
    rclpy.spin(node)


if __name__ == "__main__":
    rclpy.init()
    node = RobotBridge()

    threading.Thread(target=spin_ros, daemon=True).start()

    app.run(host="0.0.0.0", port=5000, threaded=True)
