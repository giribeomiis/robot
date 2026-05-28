import os
import time
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from flask import Flask, Response, jsonify, request
from geometry_msgs.msg import Twist
from rclpy.node import Node
from ros_robot_controller_msgs.msg import ServoPosition, ServosPosition
from sensor_msgs.msg import CameraInfo, Image

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


app = Flask(__name__)

latest_jpeg = None
latest_jpeg_lock = threading.Lock()

latest_depth_jpeg = None
latest_depth_jpeg_lock = threading.Lock()

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
    button.active { background: #2563eb; color: #fff; }
    .grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; max-width: 320px; }
    .arm-row { display: grid; grid-template-columns: 100px 1fr 50px; gap: 8px; align-items: center; margin: 8px 0; }
    .status { margin-top: 8px; color: #475569; line-height: 1.5; font-family: monospace; font-size: 13px; }
    .people-list { margin-top: 8px; font-family: monospace; font-size: 12px; color: #334155; }
    .people-list div { padding: 2px 0; }
    .follow-row { display: flex; align-items: center; gap: 8px; margin: 8px 0; }
    .follow-row input { width: 80px; padding: 6px; border: 1px solid #cbd5e1; border-radius: 6px; }
    .follow-state { margin-top: 8px; padding: 8px; background: #f1f5f9; border-radius: 6px; font-family: monospace; font-size: 12px; }
    .camera-title { margin: 14px 0 8px; }
    @media (max-width: 900px) { main { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <header>
    <h1>Robot Control Room</h1>
    <div>RGB / Depth / YOLO+Depth / Wheels / Robot Arm / Auto-Follow</div>
  </header>

  <main>
    <section>
      <h2 class="camera-title">RGB Camera</h2>
      <img src="/stream" alt="rgb camera">

      <h2 class="camera-title">Depth Camera</h2>
      <img src="/depth_stream" alt="depth camera">

      <p class="status" id="status">status loading...</p>
      <div class="people-list" id="people"></div>
    </section>

    <section>
      <h2>Auto-Follow</h2>
      <div>
        <button id="follow-on" onclick="follow(true)">Follow ON</button>
        <button id="follow-off" class="danger" onclick="follow(false)">Follow OFF</button>
      </div>
      <div class="follow-row">
        <label>Target distance (m):</label>
        <input id="target-dist" type="number" step="0.1" min="0.3" max="5.0" value="1.2">
        <button onclick="setTargetDist()">Set</button>
      </div>
      <div class="follow-state" id="follow-state">follow: off</div>

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

    function drive(x, y) { req(`/control?action=move&x=${x}&y=${y}`).then(updateStatus); }
    function stop() { req("/control?action=stop").then(updateStatus); }
    function armServo(id, position, duration) {
      req(`/arm?action=servo&id=${id}&position=${position}&duration=${duration}`).then(updateStatus);
    }
    function armAction(action) { req(`/arm?action=${action}`).then(updateStatus); }
    function detect(enabled) { req(`/detect?enabled=${enabled}`).then(updateStatus); }
    function follow(enabled) { req(`/follow?enabled=${enabled}`).then(updateStatus); }
    function setTargetDist() {
      const v = document.getElementById("target-dist").value;
      req(`/follow?target_distance=${v}`).then(updateStatus);
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
          `rgb=${data.camera_frame_ready ? "ready" : "waiting"} / ` +
          `depth_view=${data.depth_view_ready ? "ready" : "waiting"} / ` +
          `depth=${data.depth_frame_ready ? "ready" : "waiting"} / ` +
          `intrinsics=${data.camera_info_received ? "ok" : "missing"}<br>` +
          `yolo=${data.yolo_loaded ? "loaded" : "not loaded"} / ` +
          `detection=${data.detection_enabled ? "on" : "off"} / ` +
          `yolo_fps=${data.yolo_fps}<br>` +
          `people=${data.person_count} / dropped=${data.frames_dropped}`;

        const f = data.follow;
        const onBtn = document.getElementById("follow-on");
        const offBtn = document.getElementById("follow-off");
        onBtn.className = f.enabled ? "active" : "";
        offBtn.className = f.enabled ? "danger" : "active";

        let txt = `follow: ${f.enabled ? "ON" : "off"} / target=${f.target_distance.toFixed(2)}m<br>`;
        txt += `state: ${f.state}<br>`;
        if (f.target_locked) {
          txt += `locked: dist=${f.locked_distance === null ? "?" : f.locked_distance.toFixed(2)+"m"} / `;
          txt += `err_x=${f.error_x.toFixed(2)} / err_dist=${f.error_distance === null ? "?" : f.error_distance.toFixed(2)+"m"}<br>`;
        }
        txt += `cmd: lin=${f.last_linear.toFixed(2)} ang=${f.last_angular.toFixed(2)}`;
        document.getElementById("follow-state").innerHTML = txt;
      });

      req("/people").then(data => {
        const root = document.getElementById("people");
        if (!data.ok || !data.people || data.people.length === 0) {
          root.innerHTML = "";
          return;
        }
        root.innerHTML = data.people.map((p, i) => {
          const d = p.distance_m === null ? "no depth" : `${p.distance_m.toFixed(2)}m`;
          let pos = "";
          if (p.position_3d_m) {
            const [X, Y, Z] = p.position_3d_m;
            pos = ` 3D=(${X.toFixed(2)}, ${Y.toFixed(2)}, ${Z.toFixed(2)})`;
          }
          return `<div>person ${i+1}: dist=${d}${pos}</div>`;
        }).join("");
      });
    }

    buildArm();
    updateStatus();
    setInterval(updateStatus, 300);
  </script>
</body>
</html>
"""


class RobotBridge(Node):
    def __init__(self):
        super().__init__("android_robot_bridge")

        self.bridge = CvBridge()

        self.latest_people = []
        self._people_lock = threading.Lock()
        self._last_detection_time = 0.0

        self._pending_frame = None
        self._yolo_lock = threading.Lock()
        self._yolo_event = threading.Event()
        self._stop_event = threading.Event()

        self._latest_depth = None
        self._depth_lock = threading.Lock()
        self._depth_frame_ready = False

        self._camera_info = None
        self._camera_info_lock = threading.Lock()

        self._image_width = None
        self._image_width_lock = threading.Lock()

        self._frames_dropped = 0
        self._yolo_fps = 0.0
        self._yolo_fps_window_start = time.monotonic()
        self._yolo_fps_window_count = 0

        self.detection_enabled = True
        self.depth_min_m = float(os.getenv("DEPTH_MIN_M", "0.1"))
        self.depth_max_m = float(os.getenv("DEPTH_MAX_M", "10.0"))

        self.yolo = None
        self.yolo_confidence = float(os.getenv("YOLO_CONFIDENCE", "0.35"))
        self.yolo_image_size = int(os.getenv("YOLO_IMAGE_SIZE", "416"))

        if YOLO is not None:
            model_path = os.getenv("YOLO_MODEL", "yolov8n.pt")
            self.yolo = YOLO(model_path)
            self.get_logger().info(f"Loaded YOLO model: {model_path}")
        else:
            self.get_logger().warn("ultralytics is not installed. Detection disabled.")

        self.follow = FollowController(self)

        self._yolo_thread = None
        if self.yolo is not None:
            self._yolo_thread = threading.Thread(
                target=self._yolo_worker,
                name="yolo_worker",
                daemon=True,
            )
            self._yolo_thread.start()
            self.get_logger().info("YOLO worker thread started")

        rgb_topic = os.getenv("RGB_TOPIC", "/depth_cam/rgb/image_raw")
        depth_topic = os.getenv("DEPTH_TOPIC", "/depth_cam/depth/image_raw")
        info_topic = os.getenv("CAMERA_INFO_TOPIC", "/depth_cam/rgb/camera_info")

        self.create_subscription(Image, rgb_topic, self.on_image, 10)
        self.create_subscription(Image, depth_topic, self.on_depth, 10)
        self.create_subscription(CameraInfo, info_topic, self.on_camera_info, 10)

        self.cmd_pub = self.create_publisher(Twist, "/controller/cmd_vel", 10)
        self.arm_pub = self.create_publisher(
            ServosPosition,
            "/ros_robot_controller/bus_servo/set_position",
            10,
        )

        self.follow.start()

        self.get_logger().info(f"Subscribed RGB: {rgb_topic}")
        self.get_logger().info(f"Subscribed depth: {depth_topic}")
        self.get_logger().info(f"Subscribed camera_info: {info_topic}")

    def on_image(self, msg):
        global latest_jpeg

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"RGB conversion error: {e}")
            return

        h, w = frame.shape[:2]
        with self._image_width_lock:
            if self._image_width != w:
                self._image_width = w

        if self.detection_enabled and self.yolo is not None:
            with self._yolo_lock:
                if self._pending_frame is not None:
                    self._frames_dropped += 1
                self._pending_frame = frame.copy()
            self._yolo_event.set()

        with self._people_lock:
            people_snapshot = list(self.latest_people)

        if people_snapshot:
            self._draw_people(frame, people_snapshot)

        target = self.follow.get_locked_target()
        if target is not None:
            self._draw_target_marker(frame, target)

        ok, encoded = cv2.imencode(".jpg", frame)
        if ok:
            data = encoded.tobytes()
            with latest_jpeg_lock:
                latest_jpeg = data

    def on_depth(self, msg):
        global latest_depth_jpeg

        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")
            return

        with self._depth_lock:
            self._latest_depth = depth
            self._depth_frame_ready = True

        try:
            if depth.dtype == np.uint16:
                depth_m = depth.astype(np.float32) / 1000.0
            else:
                depth_m = depth.astype(np.float32)

            valid = np.isfinite(depth_m) & (depth_m > 0)
            display = np.zeros(depth_m.shape, dtype=np.uint8)

            if valid.any():
                clipped = np.clip(depth_m, 0.2, 4.0)
                normalized = ((clipped - 0.2) / (4.0 - 0.2) * 255.0).astype(np.uint8)
                display[valid] = normalized[valid]

            display = cv2.applyColorMap(display, cv2.COLORMAP_TURBO)

            ok, encoded = cv2.imencode(".jpg", display)
            if ok:
                with latest_depth_jpeg_lock:
                    latest_depth_jpeg = encoded.tobytes()
        except Exception as e:
            self.get_logger().error(f"Depth encode error: {e}")

    def on_camera_info(self, msg):
        try:
            k = msg.k
            fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
        except Exception:
            return

        if fx <= 0 or fy <= 0:
            return

        with self._camera_info_lock:
            self._camera_info = (fx, fy, cx, cy)

    def _yolo_worker(self):
        while not self._stop_event.is_set():
            if not self._yolo_event.wait(timeout=0.5):
                continue

            self._yolo_event.clear()

            with self._yolo_lock:
                frame = self._pending_frame
                self._pending_frame = None

            if frame is None or not self.detection_enabled:
                continue

            try:
                boxes = self._detect_people(frame)
            except Exception as e:
                self.get_logger().error(f"YOLO inference error: {e}")
                continue

            with self._depth_lock:
                depth = self._latest_depth
            with self._camera_info_lock:
                cam_info = self._camera_info

            rgb_h, rgb_w = frame.shape[:2]

            people = []
            for (x, y, w, h) in boxes:
                distance = self._depth_at_box(depth, x, y, w, h, rgb_w, rgb_h)

                position_3d = None
                if distance is not None and cam_info is not None:
                    fx, fy, cx_intr, cy_intr = cam_info
                    px = x + w / 2.0
                    py = y + h / 2.0
                    X = (px - cx_intr) * distance / fx
                    Y = (py - cy_intr) * distance / fy
                    Z = distance
                    position_3d = (X, Y, Z)

                people.append({
                    "x": int(x), "y": int(y), "w": int(w), "h": int(h),
                    "distance": distance,
                    "position_3d": position_3d,
                })

            with self._people_lock:
                self.latest_people = people
                self._last_detection_time = time.monotonic()

            self._yolo_fps_window_count += 1
            now = time.monotonic()
            elapsed = now - self._yolo_fps_window_start
            if elapsed >= 1.0:
                self._yolo_fps = self._yolo_fps_window_count / elapsed
                self._yolo_fps_window_start = now
                self._yolo_fps_window_count = 0

    def _detect_people(self, frame):
        results = self.yolo.predict(
            source=frame,
            classes=[0],
            conf=self.yolo_confidence,
            imgsz=self.yolo_image_size,
            verbose=False,
        )

        boxes = []
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes.xyxy.cpu().numpy():
                x1, y1, x2, y2 = box
                boxes.append((int(x1), int(y1), int(x2 - x1), int(y2 - y1)))
        return boxes

    def _depth_at_box(self, depth_frame, x, y, w, h, rgb_w, rgb_h):
        if depth_frame is None:
            return None

        dh, dw = depth_frame.shape[:2]

        if (dh, dw) != (rgb_h, rgb_w):
            sx = dw / rgb_w
            sy = dh / rgb_h
            x = x * sx
            y = y * sy
            w = w * sx
            h = h * sy

        cx_start = max(0, int(x + w * 0.25))
        cx_end = min(dw, int(x + w * 0.75))
        cy_start = max(0, int(y + h * 0.25))
        cy_end = min(dh, int(y + h * 0.75))

        if cx_end <= cx_start or cy_end <= cy_start:
            return None

        roi = depth_frame[cy_start:cy_end, cx_start:cx_end]

        if roi.dtype == np.uint16:
            mask = roi > 0
            if not mask.any():
                return None
            valid_m = roi[mask].astype(np.float32) / 1000.0
        elif roi.dtype == np.float32 or roi.dtype == np.float64:
            mask = np.isfinite(roi) & (roi > 0)
            if not mask.any():
                return None
            valid_m = roi[mask]
        else:
            return None

        valid_m = valid_m[(valid_m >= self.depth_min_m) & (valid_m <= self.depth_max_m)]

        if valid_m.size == 0:
            return None

        return float(np.median(valid_m))

    def _draw_people(self, frame, people):
        for index, p in enumerate(people, start=1):
            x, y, w, h = p["x"], p["y"], p["w"], p["h"]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 80), 2)

            label = f"person {index}"
            if p["distance"] is not None:
                label += f" {p['distance']:.2f}m"

            cv2.putText(
                frame,
                label,
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 80),
                2,
            )

    def _draw_target_marker(self, frame, target):
        x, y, w, h = target["x"], target["y"], target["w"], target["h"]
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 80, 255), 3)
        cv2.putText(
            frame,
            "TARGET",
            (x, y + h + 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 80, 255),
            2,
        )

    def move(self, x, y):
        msg = Twist()
        msg.linear.x = y * 0.2
        msg.angular.z = -x * 0.5
        self.cmd_pub.publish(msg)

    def publish_cmd(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
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

    def get_image_width(self):
        with self._image_width_lock:
            return self._image_width

    def get_people_and_age(self):
        with self._people_lock:
            people = list(self.latest_people)
            last_t = self._last_detection_time

        if last_t == 0.0:
            return people, float("inf")

        return people, time.monotonic() - last_t

    def shutdown(self):
        self._stop_event.set()
        self._yolo_event.set()
        self.follow.stop()

        if self._yolo_thread is not None:
            self._yolo_thread.join(timeout=2.0)


class FollowController:
    def __init__(self, bridge):
        self.bridge = bridge

        self.rate_hz = float(os.getenv("FOLLOW_RATE_HZ", "20"))
        self.target_distance = float(os.getenv("FOLLOW_TARGET_DIST_M", "1.2"))

        self.kp_yaw = float(os.getenv("FOLLOW_KP_YAW", "1.2"))
        self.kp_dist = float(os.getenv("FOLLOW_KP_DIST", "0.5"))

        self.max_linear = float(os.getenv("FOLLOW_MAX_LIN", "0.2"))
        self.max_angular = float(os.getenv("FOLLOW_MAX_ANG", "0.5"))

        self.deadzone_x = float(os.getenv("FOLLOW_DEADZONE_X", "0.08"))
        self.deadzone_dist = float(os.getenv("FOLLOW_DEADZONE_DIST", "0.1"))

        self.detection_timeout = float(os.getenv("FOLLOW_TIMEOUT_S", "1.0"))
        self.min_safe_distance = float(os.getenv("FOLLOW_MIN_DIST_M", "0.5"))

        self._enabled = False
        self._lock = threading.Lock()
        self._state = "idle"
        self._locked_target = None
        self._last_linear = 0.0
        self._last_angular = 0.0
        self._last_error_x = 0.0
        self._last_error_distance = None

        self._thread = None
        self._stop_event = threading.Event()

    def start(self):
        self._thread = threading.Thread(
            target=self._loop,
            name="follow_controller",
            daemon=True,
        )
        self._thread.start()

    def stop(self):
        self._stop_event.set()

        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def set_enabled(self, enabled):
        with self._lock:
            was_enabled = self._enabled
            self._enabled = bool(enabled)

            if not self._enabled:
                self._state = "idle"
                self._locked_target = None

        if was_enabled and not enabled:
            self.bridge.publish_cmd(0.0, 0.0)

    def set_target_distance(self, distance):
        with self._lock:
            self.target_distance = max(0.3, min(5.0, float(distance)))

    def get_locked_target(self):
        with self._lock:
            return dict(self._locked_target) if self._locked_target else None

    def snapshot(self):
        with self._lock:
            return {
                "enabled": self._enabled,
                "state": self._state,
                "target_distance": self.target_distance,
                "target_locked": self._locked_target is not None,
                "locked_distance": (
                    self._locked_target["distance"]
                    if self._locked_target else None
                ),
                "error_x": self._last_error_x,
                "error_distance": self._last_error_distance,
                "last_linear": self._last_linear,
                "last_angular": self._last_angular,
            }

    def _loop(self):
        period = 1.0 / self.rate_hz

        while not self._stop_event.is_set():
            loop_start = time.monotonic()

            try:
                self._step()
            except Exception as e:
                self.bridge.get_logger().error(f"FollowController step error: {e}")
                self.bridge.publish_cmd(0.0, 0.0)

            elapsed = time.monotonic() - loop_start
            sleep_for = period - elapsed

            if sleep_for > 0:
                time.sleep(sleep_for)

    def _step(self):
        with self._lock:
            enabled = self._enabled

        if not enabled:
            with self._lock:
                self._state = "idle"
                self._locked_target = None
                self._last_linear = 0.0
                self._last_angular = 0.0
            return

        people, age = self.bridge.get_people_and_age()
        image_width = self.bridge.get_image_width()

        if age > self.detection_timeout or image_width is None:
            self._set_state("searching", None, 0.0, None)
            self.bridge.publish_cmd(0.0, 0.0)
            return

        target = self._select_target(people)
        if target is None:
            self._set_state("searching", None, 0.0, None)
            self.bridge.publish_cmd(0.0, 0.0)
            return

        linear, angular, err_x, err_dist = self._compute_command(target, image_width)

        with self._lock:
            self._state = "tracking"
            self._locked_target = target
            self._last_linear = linear
            self._last_angular = angular
            self._last_error_x = err_x
            self._last_error_distance = err_dist

        self.bridge.publish_cmd(linear, angular)

    def _set_state(self, state, target, linear, err_dist):
        with self._lock:
            self._state = state
            self._locked_target = target
            self._last_linear = linear
            self._last_angular = 0.0
            self._last_error_x = 0.0
            self._last_error_distance = err_dist

    def _select_target(self, people):
        if not people:
            return None

        with_distance = [p for p in people if p["distance"] is not None]

        if with_distance:
            return min(with_distance, key=lambda p: p["distance"])

        return max(people, key=lambda p: p["w"] * p["h"])

    def _compute_command(self, target, image_width):
        cx_target = target["x"] + target["w"] / 2.0
        half_width = image_width / 2.0
        err_x = (cx_target - half_width) / half_width

        if abs(err_x) < self.deadzone_x:
            angular = 0.0
        else:
            angular = -self.kp_yaw * err_x

        distance = target["distance"]

        if distance is None:
            linear = 0.0
            err_dist = None
        else:
            err_dist = distance - self.target_distance

            if distance < self.min_safe_distance:
                linear = 0.0
            elif abs(err_dist) < self.deadzone_dist:
                linear = 0.0
            else:
                linear = self.kp_dist * err_dist

        linear = max(-self.max_linear, min(self.max_linear, linear))
        angular = max(-self.max_angular, min(self.max_angular, angular))

        return linear, angular, err_x, err_dist


@app.get("/")
def dashboard():
    return Response(DASHBOARD_HTML, mimetype="text/html")


@app.get("/control")
def control():
    if node is None:
        return jsonify({"ok": False, "error": "node not ready"}), 503

    action = request.args.get("action", "move")
    x = float(request.args.get("x", 0))
    y = float(request.args.get("y", 0))
    x = max(-1.0, min(1.0, x))
    y = max(-1.0, min(1.0, y))

    if node.follow.snapshot()["enabled"]:
        node.follow.set_enabled(False)

    if action == "stop":
        node.stop()
        return jsonify({
            "ok": True,
            "action": "stop",
            "x": 0,
            "y": 0,
            "follow_auto_disabled": True,
        })

    node.move(x, y)
    return jsonify({
        "ok": True,
        "action": "move",
        "x": x,
        "y": y,
        "follow_auto_disabled": True,
    })


@app.get("/arm")
def arm():
    if node is None:
        return jsonify({"ok": False, "error": "node not ready"}), 503

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
        "duration": duration,
    })


@app.get("/detect")
def detect():
    if node is None:
        return jsonify({"ok": False, "error": "node not ready"}), 503

    enabled = request.args.get("enabled")
    if enabled is not None:
        new_val = enabled.lower() in ("1", "true", "yes", "on")
        node.detection_enabled = new_val

        if not new_val and node.follow.snapshot()["enabled"]:
            node.follow.set_enabled(False)

    with node._people_lock:
        person_count = len(node.latest_people)

    return jsonify({
        "ok": True,
        "detection_enabled": node.detection_enabled,
        "person_count": person_count,
        "yolo_loaded": node.yolo is not None,
        "yolo_fps": round(node._yolo_fps, 1),
        "frames_dropped": node._frames_dropped,
    })


@app.get("/follow")
def follow():
    if node is None:
        return jsonify({"ok": False, "error": "node not ready"}), 503

    enabled = request.args.get("enabled")
    target_distance = request.args.get("target_distance")

    if enabled is not None:
        want = enabled.lower() in ("1", "true", "yes", "on")
        if want and not node.detection_enabled:
            return jsonify({
                "ok": False,
                "error": "detection is off - enable /detect first",
            }), 400
        node.follow.set_enabled(want)

    if target_distance is not None:
        try:
            node.follow.set_target_distance(float(target_distance))
        except ValueError:
            return jsonify({"ok": False, "error": "invalid target_distance"}), 400

    snap = node.follow.snapshot()
    return jsonify({"ok": True, **snap})


@app.get("/people")
def people():
    if node is None:
        return jsonify({"ok": False, "error": "node not ready"}), 503

    with node._people_lock:
        snapshot = list(node.latest_people)

    return jsonify({
        "ok": True,
        "count": len(snapshot),
        "people": [
            {
                "x": p["x"],
                "y": p["y"],
                "w": p["w"],
                "h": p["h"],
                "distance_m": p["distance"],
                "position_3d_m": (
                    list(p["position_3d"])
                    if p["position_3d"] is not None
                    else None
                ),
            }
            for p in snapshot
        ],
    })


def frames():
    while True:
        with latest_jpeg_lock:
            data = latest_jpeg

        if data is None:
            time.sleep(0.1)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + data
            + b"\r\n"
        )

        time.sleep(0.03)


def depth_frames():
    while True:
        with latest_depth_jpeg_lock:
            data = latest_depth_jpeg

        if data is None:
            time.sleep(0.1)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + data
            + b"\r\n"
        )

        time.sleep(0.03)


@app.get("/stream")
def stream():
    return Response(frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.get("/depth_stream")
def depth_stream():
    return Response(depth_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.get("/status")
def status():
    if node is None:
        return jsonify({
            "ok": False,
            "camera_frame_ready": False,
            "depth_view_ready": False,
            "depth_frame_ready": False,
            "camera_info_received": False,
            "detection_enabled": False,
            "person_count": 0,
            "yolo_loaded": False,
            "yolo_fps": 0.0,
            "frames_dropped": 0,
            "follow": {
                "enabled": False,
                "state": "idle",
                "target_distance": 0.0,
                "target_locked": False,
                "locked_distance": None,
                "error_x": 0.0,
                "error_distance": None,
                "last_linear": 0.0,
                "last_angular": 0.0,
            },
        })

    with latest_jpeg_lock:
        camera_ready = latest_jpeg is not None
    with latest_depth_jpeg_lock:
        depth_view_ready = latest_depth_jpeg is not None
    with node._depth_lock:
        depth_ready = node._depth_frame_ready
    with node._camera_info_lock:
        info_received = node._camera_info is not None
    with node._people_lock:
        person_count = len(node.latest_people)

    return jsonify({
        "ok": True,
        "camera_frame_ready": camera_ready,
        "depth_view_ready": depth_view_ready,
        "depth_frame_ready": depth_ready,
        "camera_info_received": info_received,
        "detection_enabled": node.detection_enabled,
        "person_count": person_count,
        "yolo_loaded": node.yolo is not None,
        "yolo_fps": round(node._yolo_fps, 1),
        "frames_dropped": node._frames_dropped,
        "follow": node.follow.snapshot(),
    })


def spin_ros():
    rclpy.spin(node)


if __name__ == "__main__":
    rclpy.init()
    node = RobotBridge()

    threading.Thread(target=spin_ros, daemon=True).start()

    try:
        app.run(host="0.0.0.0", port=5000, threaded=True)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
