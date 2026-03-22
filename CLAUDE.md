# CLAUDE.md

이 파일은 이 저장소에서 작업할 때 Claude Code(claude.ai/code)에게 제공되는 안내 문서입니다.

## 빌드 명령어

```bash
# 전체 패키지 빌드
cd ~/ros2_ws && colcon build

# 특정 패키지만 빌드
colcon build --packages-select dxl_nano
colcon build --packages-select camera_ros2
colcon build --packages-select linedetect_rapi5
colcon build --packages-select linedetect_wsl
colcon build --packages-select linetracer_sim

# 빌드 후 워크스페이스 소싱
source ~/ros2_ws/install/setup.bash
```

## 실행 명령어

```bash
# 영상 퍼블리셔 (Raspberry Pi 5에서 실행)
ros2 run linedetect_rapi5 pub_node

# 라인 검출 구독자 / 디버그 표시 (WSL2-Ubuntu에서 실행)
ros2 run linedetect_wsl sub_node

# 라인트레이서 제어 노드 - 검출 + P제어 + 모터 명령 발행 (WSL2-Ubuntu에서 실행)
ros2 run linetracer_sim linetracer_node

# Dynamixel 모터 구독자 노드 (Raspberry Pi 5에서 실행)
ros2 run dxl_nano sub_node

# 카메라 퍼블리셔 노드 (Raspberry Pi 5에서 실행)
ros2 run camera_ros2 pub_node

# 카메라 구독자 / 화면 표시 (WSL2-Ubuntu 또는 원격 머신에서 실행)
ros2 run camera_ros2 sub_node
```

## 하드웨어 설정

```bash
# Dynamixel 모터용 USB 시리얼 포트 권한 부여
sudo chmod a+rw /dev/ttyUSB0
```

네트워크로 연결된 모든 머신에서 `ROS_DOMAIN_ID`를 동일한 값으로 설정해야 통신이 가능합니다.

## 전체 아키텍처

이 워크스페이스는 두 가지 시나리오를 지원합니다: **라인트레이서 시뮬레이션**과 **실제 카메라 스트리밍**.

### 라인트레이서 시뮬레이션 파이프라인

```
[MP4 영상 파일]
      ↓
[linedetect_rapi5/pub]  ─[Image_Topic]──────────────────────────┐
                                                                  ↓
                                              [linetracer_sim/linetracer] ← 's' 키로 시작
                                              (라인 검출 + P제어)
                                                                  ↓
                                                         [vel_cmd_topic]
                                                                  ↓
                                                       [dxl_nano/sub] → 모터 구동
```

### 실제 카메라 스트리밍 파이프라인

```
Raspberry Pi 5                          원격 머신 (WSL2/Ubuntu)
├── camera_ros2/pub  ─[image/compressed]──────────────> camera_ros2/sub
└── dxl_nano/sub    <─[vel_cmd_topic]─────────────────── (외부 퍼블리셔)
```

### ROS 토픽 정리

| 토픽 | 메시지 타입 | 발행자 | 구독자 |
|------|------------|--------|--------|
| `Image_Topic` | `sensor_msgs/CompressedImage` | linedetect_rapi5 | linedetect_wsl, linetracer_sim |
| `vel_cmd_topic` | `geometry_msgs/Vector3` | linetracer_sim | dxl_nano |
| `image/compressed` | `sensor_msgs/CompressedImage` | camera_ros2/pub | camera_ros2/sub |

모든 노드의 QoS: `BestEffort` + `KeepLast(10)`

## 패키지 설명

### `linedetect_rapi5`

MP4 영상 파일을 열어 프레임을 `Image_Topic`에 약 30Hz로 발행합니다. 영상이 끝나면 무한 반복합니다.

- 영상 경로는 `src/pub.cpp` 상단에 하드코딩되어 있습니다 (`/home/rapi5/ros2_ws/video/simulation/...`).

### `linedetect_wsl`

`Image_Topic`을 구독하여 라인 검출 결과를 OpenCV 창에 표시하는 **디버그 전용** 노드입니다. 모터 명령은 발행하지 않습니다.

- ROI: 프레임 하단 90px (270~360행)
- 밝기 보정 후 임계값 128로 이진화 → 연결 성분 분석(CCL)
- 유효 블롭 면적: 100~15000px, 이전 위치에서 150px 이내

### `linetracer_sim`

라인 검출과 P제어를 통합한 핵심 제어 노드입니다.

- ROI: 프레임 하단 1/4
- **2단계 블롭 탐색**: 1단계(150px 반경) → 위치 업데이트 → 2단계(30px 반경) 재확인
- **P제어**: `error = 검출된_x - (frame_width / 2.0)`
  ```
  left_vel  = base_vel - error * k
  right_vel = -(base_vel + error * k)
  ```
- ROS 동적 파라미터: `k`(비례이득, 기본 0.13), `base_vel`(기본 50)
- 키보드: `s` 시작, `q` 종료 (논블로킹 입력)
- `linetracer.cpp` 하나로 구성 — `CMakeLists.txt`/`package.xml` 없음 (추가 필요)

### `dxl_nano`

`vel_cmd_topic`(`geometry_msgs/Vector3`)을 구독하여 USB 시리얼(`/dev/ttyUSB0`)로 Dynamixel 모터 2개를 구동합니다.

- 모터 모델은 `dxl.hpp`의 `#define`으로 컴파일 시 선택: `MX12W`(기본, Protocol v1.0), `XC430W150`, `XL430W250`(Protocol v2.0)
- `Vector3.x` / `Vector3.y` → 좌/우 모터 속도
- `GroupSyncWrite`로 동기 명령 전송

### `camera_ros2`

GStreamer(`libcamerasrc`)로 Raspberry Pi 카메라 캡처 → JPEG 압축 → `image/compressed` 발행(40Hz).

## 주요 소스 파일

| 파일 | 역할 |
|------|------|
| `src/linedetect_rapi5/src/pub.cpp` | MP4 파일 읽기 + 압축 프레임 발행 |
| `src/linedetect_wsl/src/sub.cpp` | 라인 검출 + 디버그 시각화 |
| `src/linetracer_sim/linetracer.cpp` | 라인 검출 + P제어 + 속도 명령 발행 |
| `src/dxl_nano/src/sub.cpp` | 속도 명령 수신 → `setVelocity()` 호출 |
| `src/dxl_nano/src/dxl.cpp` | Dynamixel SDK 래퍼 |
| `src/dxl_nano/include/dxl_nano/dxl.hpp` | 모터 모델 `#define` 선택 |
| `src/camera_ros2/src/pub.cpp` | GStreamer 캡처 + 발행 |
| `src/camera_ros2/src/sub.cpp` | 압축 해제 + OpenCV 표시 |

## ROS 배포판

Ubuntu 24.04 위의 ROS Jazzy. 누락된 의존성 설치:
```bash
sudo apt install ros-jazzy-cv-bridge libopencv-dev
```
