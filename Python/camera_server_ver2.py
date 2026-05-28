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

# JPEG 스트리밍 버퍼는 Flask 스레드도 접근하므로 락이 필요하다.
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
          `people=${data.person_count} / yolo_fps=${data.yolo_fps}<br>` +
          `frames_dropped=${data.frames_dropped}`;
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

        # 탐지 결과 — YOLO 워커가 쓰고 ROS 콜백이 읽는다.
        self.latest_people = []
        self._people_lock = threading.Lock()

        # YOLO 워커로 넘기는 최신 프레임 (가장 최신 1장만 유지).
        self._pending_frame = None
        self._yolo_lock = threading.Lock()
        self._yolo_event = threading.Event()
        self._stop_event = threading.Event()

        # 통계 — 디버깅/관측용.
        self._frames_dropped = 0
        self._yolo_inference_count = 0
        self._yolo_fps = 0.0
        self._yolo_fps_window_start = time.monotonic()
        self._yolo_fps_window_count = 0

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

        # YOLO 워커 스레드 시작 — 모델이 로드된 경우에만.
        self._yolo_thread = None
        if self.yolo is not None:
            self._yolo_thread = threading.Thread(
                target=self._yolo_worker,
                name="yolo_worker",
                daemon=True,
            )
            self._yolo_thread.start()
            self.get_logger().info("YOLO worker thread started")

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

    # ------------------------------------------------------------------
    # ROS 콜백 — 빠르게 끝내고 리턴해야 한다.
    # ------------------------------------------------------------------
    def on_image(self, msg):
        global latest_jpeg

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # 1) YOLO 워커에 프레임 핸드오프 (논블로킹).
        #    워커가 처리 중이면 이전 pending은 그냥 덮어쓴다 = 자동 frame drop.
        if self.detection_enabled and self.yolo is not None:
            with self._yolo_lock:
                if self._pending_frame is not None:
                    # 워커가 미처 못 가져간 프레임이 있었다 = 드랍 카운트.
                    self._frames_dropped += 1
                self._pending_frame = frame.copy()  # 메모리 공유 방지
            self._yolo_event.set()

        # 2) 가장 최신 탐지 결과로 박스 그리기 (탐지 결과는 살짝 stale할 수 있음).
        with self._people_lock:
            people_snapshot = list(self.latest_people)

        if people_snapshot:
            self.draw_people(frame, people_snapshot)

        # 3) JPEG 인코딩 후 스트리밍 버퍼로 게시.
        ok, encoded = cv2.imencode(".jpg", frame)
        if ok:
            data = encoded.tobytes()
            with latest_jpeg_lock:
                latest_jpeg = data

    # ------------------------------------------------------------------
    # YOLO 워커 — 별도 스레드에서 추론만 담당.
    # ------------------------------------------------------------------
    def _yolo_worker(self):
        while not self._stop_event.is_set():
            # 새 프레임이 올 때까지 대기. timeout을 두는 이유는 종료 신호를 받기 위해서.
            if not self._yolo_event.wait(timeout=0.5):
                continue
            self._yolo_event.clear()

            # 대기 중인 프레임을 꺼낸다 (consume).
            with self._yolo_lock:
                frame = self._pending_frame
                self._pending_frame = None

            if frame is None:
                continue

            # detection_enabled가 꺼지면 큐에 있는 잔여 프레임은 그냥 버린다.
            if not self.detection_enabled:
                continue

            try:
                people = self._detect_people(frame)
            except Exception as e:
                self.get_logger().error(f"YOLO inference error: {e}")
                continue

            with self._people_lock:
                self.latest_people = people

            # FPS 통계 — 1초 간격으로 갱신.
            self._yolo_inference_count += 1
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

    # ------------------------------------------------------------------
    # 종료 처리
    # ------------------------------------------------------------------
    def shutdown(self):
        self._stop_event.set()
        self._yolo_event.set()  # 워커를 깨워서 종료 체크하게 함
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
        "ok": True,
        "action": "servo",
        "id": servo_id,
        "position": position,
        "duration": duration
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
            "detection_enabled": False,
            "person_count": 0,
            "yolo_loaded": False,
            "yolo_fps": 0.0,
            "frames_dropped": 0,
        })

    with latest_jpeg_lock:
        camera_ready = latest_jpeg is not None
    with node._people_lock:
        person_count = len(node.latest_people)

    return jsonify({
        "ok": True,
        "camera_frame_ready": camera_ready,
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
