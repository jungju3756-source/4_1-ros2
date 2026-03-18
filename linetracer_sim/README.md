# linetracer_sim

라인 트레이서 시뮬레이션 패키지. 영상 파일을 입력으로 라인 검출 및 모터 속도 명령을 시뮬레이션한다.

---

## 노드 구성

| 노드 | 실행 파일 | 역할 |
|---|---|---|
| `video_publisher` | `video_publisher` | MP4 영상을 `/video` 토픽으로 퍼블리시 |
| `line_controller` | `line_controller` | `/video` 구독 → 라인 검출 → `/topic_dxlpub` 퍼블리시 |
| `dxl_subscriber` | `dxl_subscriber` | `/topic_dxlpub` 구독 → 모터 RPM 출력 (시뮬레이션) |

## 토픽 구성

| 토픽 | 메시지 타입 | 방향 |
|---|---|---|
| `/video` | `sensor_msgs/msg/Image` | video_publisher → line_controller |
| `/topic_dxlpub` | `geometry_msgs/msg/Vector3` | line_controller → dxl_subscriber |

- `Vector3.x` = left_rpm
- `Vector3.y` = right_rpm

---

## 빌드

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select linetracer_sim
source install/setup.bash
```

---

## 실행

터미널 3개를 열어 순서대로 실행한다.

### 터미널 1 — 영상 퍼블리셔

```bash
source ~/ros2_ws/install/setup.bash

# 5번 영상 (CW outer)
ros2 run linetracer_sim video_publisher \
  --ros-args -p video_path:=/home/aim88/simulation/5_lt_cw_100rpm_out.mp4

# 또는 7번 영상 (CCW inner)
ros2 run linetracer_sim video_publisher \
  --ros-args -p video_path:=/home/aim88/simulation/7_lt_ccw_100rpm_in.mp4
```

### 터미널 2 — 라인 검출 + 속도 제어

```bash
source ~/ros2_ws/install/setup.bash
ros2 run linetracer_sim line_controller
```

실행 후 키보드 입력으로 제어:

| 키 | 동작 |
|---|---|
| `s` | 속도 명령 전송 시작 (`/topic_dxlpub` 퍼블리시) |
| `q` | 정지 (0 rpm 즉시 전송 후 퍼블리시 중단) |

터미널 출력 예시:
```
[line_controller] err:12 | left:88.0 | right:112.0 | time:3.45ms | sending:YES
```

### 터미널 3 — 모터 시뮬레이터

```bash
source ~/ros2_ws/install/setup.bash
ros2 run linetracer_sim dxl_subscriber
```

터미널 출력 예시:
```
[dxl_subscriber] [모터 시뮬] left_rpm: 88.0 | right_rpm: 112.0
```

---

## 속도 계산 공식

```
error      = 320 - line_x      # 화면 중앙(320)과 라인 위치의 차이
left_rpm   = 100.0 - 1.0 * error
right_rpm  = 100.0 + 1.0 * error
```

- 기준 속도 (BASE_RPM): 100 rpm
- 제어 이득 (K): 1.0

---

## 디렉토리 구조

```
Motion_Control/
├── linedetection/          # linedetection 패키지 복사본 (참조용)
│   ├── linedetect_nano/
│   └── linedetect_wsl/
└── linetracer_sim/         # 이 패키지
    ├── CMakeLists.txt
    ├── package.xml
    ├── README.md
    └── src/
        ├── video_publisher.cpp
        ├── line_controller.cpp
        └── dxl_subscriber.cpp
```
