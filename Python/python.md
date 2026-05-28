0. 빨간 터미널에서 종료하고
1. sudo systemctl stop start_app_node.service

2. 터미널 1: 카메라 켜기
source/opt/ros/humble/setup.zsh source~/ros2_ws/install/setup.zsh ros2 launch peripherals depth_camera.launch.py

3. 터미널 2: 바퀴/베이스 켜기
source/opt/ros/humble/setup.zsh source~/ros2_ws/install/setup.zsh ros2 launch slam slam.launch.py

4. 터미널 3: 서버 + YOLO + 관제페이지 켜기
cd~/android-camera-server 
source/opt/ros/humble/setup.zsh 
source~/ros2_ws/install/setup.zsh 
YOLO_MODEL=yolov8n.pt YOLO_IMAGE_SIZE=416 YOLO_CONFIDENCE=0.35 python3 camera_server.py

5. 추가할 예정 아두이노
Android 앱 / PC 관제 페이지
        ↓ HTTP
Jetson camera_server.py
        ↓ Serial USB
Arduino
        ↓
LED / 센서 / 릴레이 / 추가 서보 / 기타 장치
