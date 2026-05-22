# Robot Joystick Controller

Android Studio에서 열 수 있는 로봇 조종 예제 앱입니다.

## 기능

- 로봇 카메라 스트림을 앱 상단 `WebView`에 표시합니다.
- 하단 조이스틱을 움직이면 `x`, `y` 값을 `-1.00`부터 `1.00` 사이로 정규화해 로봇 제어 URL로 전송합니다.
- 조이스틱에서 손을 떼거나 `EMERGENCY STOP` 버튼을 누르면 `stop` 명령을 보냅니다.

## 전체 구조

```text
Android app -> Ubuntu server HTTP API -> motor/camera control
```

Ubuntu 서버 예제는 `ubuntu-server/` 폴더에 있습니다. 서버를 실행한 뒤 Android 앱의 `RobotConfig.java`를 Ubuntu 서버 IP로 바꾸면 됩니다.

## 로봇 주소 바꾸기

`app/src/main/java/com/example/robotjoystick/RobotConfig.java`에서 아래 값을 Ubuntu 서버 IP에 맞게 수정하세요.

```java
static final String VIDEO_URL = "http://192.168.0.50:5000/stream";
static final String CONTROL_URL = "http://192.168.0.50:5000/control";
```

현재 조종 명령은 HTTP GET으로 전송됩니다.

```text
http://192.168.0.50:5000/control?action=move&x=0.40&y=0.80
http://192.168.0.50:5000/control?action=stop&x=0.00&y=0.00
```

ESP32, Raspberry Pi, Arduino Wi-Fi 서버 등에서 이 쿼리를 받아 모터 속도로 변환하면 됩니다.

## Android Studio에서 실행

1. Android Studio에서 이 폴더를 엽니다.
2. Gradle Sync가 끝날 때까지 기다립니다.
3. 휴대폰이나 에뮬레이터를 선택하고 Run을 누릅니다.
4. 실제 로봇을 테스트할 때는 휴대폰을 로봇 Wi-Fi와 같은 네트워크에 연결합니다.

현재 이 PC에서는 Android SDK 경로가 잡혀 있지 않아 터미널 빌드는 `SDK location not found`에서 멈춥니다. Android Studio에서 프로젝트를 열면 SDK 설치 또는 `local.properties`의 `sdk.dir` 설정 안내가 뜰 수 있습니다.

## 참고

- HTTP 스트림을 바로 보기 위해 `android:usesCleartextTraffic="true"`가 켜져 있습니다.
- 카메라가 RTSP만 제공한다면 `WebView` 대신 `VideoView`나 ExoPlayer 기반으로 바꾸는 편이 좋습니다.
