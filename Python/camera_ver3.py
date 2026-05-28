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
    .status { margin-top: 8px; color: #475569; line-height: 1.5; font-family: monospace; font-size: 13px; }
    .people-list { margin-top: 8px; font-family: monospace; font-size: 12px; color: #334155; }
    .people-list div { padding: 2px 0; }
    @media (max-width: 900px) { main { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <header>
    <h1>Robot Control Room</h1>
    <div>Camera / YOLO+Depth / Wheels / Robot Arm</div>
  </header>

  <main>
    <section>
      <img src="/stream" alt="camera">
      <p class="status" id="status">status loading...</p>
      <div class="people-list" id="people"></div>
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
    function drive(x, y) { req(`/control?action=move&x=${x}&y=${y}`).then(updateStatus); }
    function stop() { req("/control?action=stop").then(updateStatus); }
    function armServo(id, position, duration) {
      req(`/arm?action=servo&id=${id}&position=${position}&duration=${duration}`).then(updateStatus);
    }
    function armAction(action) { req(`/arm?action=${action}`).then(updateStatus); }
    function detect(enabled) { req(`/detect?enabled=${enabled}`).then(updateStatus); }

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
          `camera=${data.camera_frame_ready ? "ready" : "waiting"} / ` +
          `depth=${data.depth_frame_ready ? "ready" : "waiting"} / ` +
          `intrinsics=${data.camera_info_received ? "ok" : "missing"}<br>` +
          `yolo=${data.yolo_loaded ? "loaded" : "not loaded"} / ` +
          `detection=${data.detection_enabled ? "on" : "off"} / ` +
          `yolo_fps=${data.yolo_fps}<br>` +
          `people=${data.person_count} / dropped=${data.frames_dropped}`;
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
    setInterval(updateStatus, 500);
  </script>
</body>
</html>
"""


class RobotBridge(Node):
    def __init__(self):
        super().__init__("android_robot_bridge")

        self.bridge = CvBridge()

        # ------- 탐지 결과 -------
        # 각 entry: {"x", "y", "w", "h", "distance", "position_3d"}
        # distance: meters or None ; position_3d: (X, Y, Z) in meters or None
        self.latest_people = []
        self._people_lock = threading.Lock()

        # ------- YOLO 워커로 넘기는 최신 RGB 프레임 -------
        self._pending_frame = None
        self._yolo_lock = threading.Lock()
        self._yolo_event = threading.Event()
        self._stop_event = threading.Event()

        # ------- Depth 프레임 + 카메라 intrinsics -------
        self._latest_depth = None
        self._depth_lock = threading.Lock()
        self._depth_frame_ready = False

        self._camera_info = None  # (fx, fy, cx, cy)
        self._camera_info_lock = threading.Lock()

        # ------- 통계 -------
        self._frames_dropped = 0
        self._yolo_fps = 0.0
        self._yolo_fps_window_start = time.monotonic()
        self._yolo_fps_window_count = 0

        # ------- 설정 -------
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

        # ------- YOLO 워커 스레드 -------
        self._yolo_thread = None
        if self.yolo is not None:
            self._yolo_thread = threading.Thread(
                target=self._yolo_worker,
                name="yolo_worker",
                daemon=True,
            )
            self._yolo_thread.start()
            self.get_logger().info("YOLO worker thread started")

        # ------- ROS 구독/발행 -------
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

        self.get_logger().info(f"Subscribed RGB: {rgb_topic}")
        self.get_logger().info(f"Subscribed depth: {depth_topic}")
        self.get_logger().info(f"Subscribed camera_info: {info_topic}")

    # ------------------------------------------------------------------
    # ROS 콜백
    # ------------------------------------------------------------------
    def on_image(self, msg):
        global latest_jpeg

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"RGB conversion error: {e}")
            return

        # 1) YOLO 워커에 핸드오프
        if self.detection_enabled and self.yolo is not None:
            with self._yolo_lock:
                if self._pending_frame is not None:
                    self._frames_dropped += 1
                self._pending_frame = frame.copy()
            self._yolo_event.set()

        # 2) 최신 탐지 결과로 박스 그리기
        with self._people_lock:
            people_snapshot = list(self.latest_people)

        if people_snapshot:
            self._draw_people(frame, people_snapshot)

        # 3) JPEG 인코딩
        ok, encoded = cv2.imencode(".jpg", frame)
        if ok:
            data = encoded.tobytes()
            with latest_jpeg_lock:
                latest_jpeg = data

    def on_depth(self, msg):
        try:
            # 인코딩은 카메라에 따라 16UC1 또는 32FC1
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")
            return

        # 참조 재바인딩은 원자적이지만, 일관성 위해 락 사용.
        # 복사는 불필요 — cv_bridge가 매번 새 array를 반환함.
        with self._depth_lock:
            self._latest_depth = depth
            self._depth_frame_ready = True

    def on_camera_info(self, msg):
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        try:
            k = msg.k
            fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
        except Exception:
            return

        if fx <= 0 or fy <= 0:
            return  # invalid intrinsics

        with self._camera_info_lock:
            self._camera_info = (fx, fy, cx, cy)

    # ------------------------------------------------------------------
    # YOLO 워커
    # ------------------------------------------------------------------
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

            # depth와 intrinsics 스냅샷 — 복사 불필요 (포인터 재바인딩이라)
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

            # FPS 통계
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

    # ------------------------------------------------------------------
    # Depth 처리 — 박스 중앙 50% 영역의 중앙값
    # ------------------------------------------------------------------
    def _depth_at_box(self, depth_frame, x, y, w, h, rgb_w, rgb_h):
        """Return median depth in central region of box, in meters.
        Returns None if no valid depth available."""
        if depth_frame is None:
            return None

        dh, dw = depth_frame.shape[:2]

        # 해상도가 다르면 박스를 depth 좌표계로 스케일
        if (dh, dw) != (rgb_h, rgb_w):
            sx = dw / rgb_w
            sy = dh / rgb_h
            x = x * sx
            y = y * sy
            w = w * sx
            h = h * sy

        # 박스 중앙 50% 영역만 사용 (배경 픽셀 회피)
        cx_start = max(0, int(x + w * 0.25))
        cx_end = min(dw, int(x + w * 0.75))
        cy_start = max(0, int(y + h * 0.25))
        cy_end = min(dh, int(y + h * 0.75))

        if cx_end <= cx_start or cy_end <= cy_start:
            return None

        roi = depth_frame[cy_start:cy_end, cx_start:cx_end]

        # dtype에 따라 단위 변환
        if roi.dtype == np.uint16:
            # mm 단위 → 0 = hole
            mask = roi > 0
            if not mask.any():
                return None
            valid_m = roi[mask].astype(np.float32) / 1000.0
        elif roi.dtype == np.float32 or roi.dtype == np.float64:
            # 이미 m 단위 — NaN과 0 제거
            mask = np.isfinite(roi) & (roi > 0)
            if not mask.any():
                return None
            valid_m = roi[mask]
        else:
            return None

        # 거리 범위 필터
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

    # ------------------------------------------------------------------
    # 로봇 제어
    # ------------------------------------------------------------------
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
        for servo_id, position in [(1, 500), (2, 500), (3, 500), (4, 500), (5, 500), (10, 500)]:
            servo = ServoPosition()
            servo.id = servo_id
            servo.position = position
            msg.position.append(servo)
        self.arm_pub.publish(msg)

    def shutdown(self):
        self._stop_event.set()
        self._yolo_event.set()
        if self._yolo_thread is not None:
            self._yolo_thread.join(timeout=2.0)


# ----------------------------------------------------------------------
# Flask 엔드포인트
# ----------------------------------------------------------------------
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

    if action == "stop":
        node.stop()
        return jsonify({"ok": True, "action": "stop", "x": 0, "y": 0})

    node.move(x, y)
    return jsonify({"ok": True, "action": "move", "x": x, "y": y})


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
        "ok": True, "action": "servo",
        "id": servo_id, "position": position, "duration": duration,
    })


@app.get("/detect")
def detect():
    if node is None:
        return jsonify({"ok": False, "error": "node not ready"}), 503

    enabled = request.args.get("enabled")
    if enabled is not None:
        node.detection_enabled = enabled.lower() in ("1", "true", "yes", "on")

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


@app.get("/people")
def people():
    """탐지된 사람 리스트 (거리/3D 좌표 포함). 자동 추종 등 외부 사용용."""
    if node is None:
        return jsonify({"ok": False, "error": "node not ready"}), 503

    with node._people_lock:
        snapshot = list(node.latest_people)

    return jsonify({
        "ok": True,
        "count": len(snapshot),
        "people": [
            {
                "x": p["x"], "y": p["y"], "w": p["w"], "h": p["h"],
                "distance_m": p["distance"],
                "position_3d_m": list(p["position_3d"]) if p["position_3d"] is not None else None,
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
            b"Content-Type: image/jpeg\r\n\r\n" +
            data +
            b"\r\n"
        )
        time.sleep(0.03)


@app.get("/stream")
def stream():
    return Response(frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.get("/status")
def status():
    if node is None:
        return jsonify({
            "ok": False,
            "camera_frame_ready": False,
            "depth_frame_ready": False,
            "camera_info_received": False,
            "detection_enabled": False,
            "person_count": 0,
            "yolo_loaded": False,
            "yolo_fps": 0.0,
            "frames_dropped": 0,
        })

    with latest_jpeg_lock:
        camera_ready = latest_jpeg is not None
    with node._depth_lock:
        depth_ready = node._depth_frame_ready
    with node._camera_info_lock:
        info_received = node._camera_info is not None
    with node._people_lock:
        person_count = len(node.latest_people)

    return jsonify({
        "ok": True,
        "camera_frame_ready": camera_ready,
        "depth_frame_ready": depth_ready,
        "camera_info_received": info_received,
        "detection_enabled": node.detection_enabled,
        "person_count": person_count,
        "yolo_loaded": node.yolo is not None,
        "yolo_fps": round(node._yolo_fps, 1),
        "frames_dropped": node._frames_dropped,
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
