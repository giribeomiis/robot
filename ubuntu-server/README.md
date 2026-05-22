# Ubuntu Robot HTTP API

Android app -> Ubuntu server HTTP API -> motor/camera control 구조에서 Ubuntu 서버에 올리는 예제입니다.

## 실행

```bash
cd ubuntu-server
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python server.py
```

서버가 `5000` 포트로 뜨면 Android 앱 설정을 Ubuntu 서버 IP로 바꿉니다.

```java
static final String VIDEO_URL = "http://UBUNTU_SERVER_IP:5000/stream";
static final String CONTROL_URL = "http://UBUNTU_SERVER_IP:5000/control";
```

예를 들어 서버 IP가 `192.168.0.50`이면:

```java
static final String VIDEO_URL = "http://192.168.0.50:5000/stream";
static final String CONTROL_URL = "http://192.168.0.50:5000/control";
```

## API

```text
GET /control?action=move&x=0.4&y=0.8
GET /control?action=stop&x=0&y=0
GET /status
GET /stream
```

`x`는 좌우, `y`는 전후입니다. 값 범위는 `-1.0`부터 `1.0`입니다.

## 모터 연결

기본 상태에서는 GPIO를 건드리지 않고 명령을 터미널에 출력합니다.

라즈베리파이처럼 `gpiozero`를 쓸 수 있는 장치라면:

```bash
pip install gpiozero
ROBOT_USE_GPIO=1 python server.py
```

핀 번호는 환경변수로 바꿀 수 있습니다.

```bash
LEFT_FORWARD_PIN=17 \
LEFT_BACKWARD_PIN=27 \
RIGHT_FORWARD_PIN=22 \
RIGHT_BACKWARD_PIN=23 \
LEFT_ENABLE_PIN=18 \
RIGHT_ENABLE_PIN=13 \
ROBOT_USE_GPIO=1 \
python server.py
```

일반 Ubuntu PC에서 Arduino나 모터 드라이버로 시리얼 명령을 보내는 구조라면 `MotorController.move()` 안에서 `print(...)` 부분을 시리얼 전송 코드로 바꾸면 됩니다.

## 카메라

기본 카메라는 `/dev/video0`입니다. 다른 카메라나 RTSP 주소를 쓰려면:

```bash
ROBOT_CAMERA_SOURCE=/dev/video2 python server.py
ROBOT_CAMERA_SOURCE=rtsp://192.168.0.10:8554/live python server.py
```

## ROS2 Orbbec 카메라

Orbbec 깊이 카메라가 ROS2 토픽으로 이미지를 내보내는 경우 Android 앱은 토픽을 직접 열 수 없습니다. 이때는 `ros_camera_server.py`를 사용해 ROS2 이미지 토픽을 HTTP MJPEG 스트림으로 바꿉니다.

먼저 카메라를 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
ros2 launch orbbec_camera dabai.launch.py
```

다른 터미널에서 토픽을 확인합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
ros2 topic list | grep image
```

색상 영상 토픽이 `/camera/color/image_raw`이면 브릿지를 실행합니다.

```bash
cd ~/ubuntu-server
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
sudo apt install -y python3-flask python3-opencv ros-$ROS_DISTRO-cv-bridge
python3 ros_camera_server.py
```

다른 토픽을 써야 하면:

```bash
python3 ros_camera_server.py --ros-args -p image_topic:=/실제/이미지/토픽 -p cmd_vel_topic:=/controller/cmd_vel
```

예를 들어 깊이 영상을 보려면 실제 토픽 이름을 확인한 뒤 `/camera/depth/image_raw` 같은 값으로 바꾸면 됩니다.

JetRover 환경에서 확인된 토픽을 쓰는 예:

```bash
python3 ros_camera_server.py --ros-args -p image_topic:=/depth_cam/rgb/image_raw -p cmd_vel_topic:=/controller/cmd_vel
```
