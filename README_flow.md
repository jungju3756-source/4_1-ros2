# 라인트레이서 ROS2 시스템 - 전체 흐름 및 실행 방법

## 시스템 구성

```
[ Raspberry Pi 5 ]                        [ WSL2 - Ubuntu ]
  linedetect_rapi5                          linetracer_sim
  (MP4 영상 발행)                           (라인 검출 + P제어)
       │                                          │
       │ Image_Topic                              │ topic_dxlpub
       │ sensor_msgs/CompressedImage              │ geometry_msgs/Vector3
       └──────────────────────────────────────────┤
                                                  │
  dxl_nano  ◄───────────────────────────────────┘
  (모터 구동)
```

## 패키지별 역할

### 1. `linedetect_rapi5` (Pi5에서 실행)
- MP4 영상 파일을 프레임 단위로 읽어 약 30Hz로 발행
- 발행 토픽: `Image_Topic` (`sensor_msgs/CompressedImage`)
- 영상 끝나면 자동으로 처음부터 반복
- 영상 경로: `src/pub.cpp` 상단에 하드코딩

### 2. `linetracer_sim` (WSL2에서 실행)
- `Image_Topic` 구독 → 라인 검출 → P제어 → 속도 명령 발행
- **라인 검출 과정:**
  1. 프레임 하단 1/4 ROI 추출
  2. 그레이스케일 변환 + 밝기 보정 (평균 100 유지)
  3. 임계값 150으로 이진화
  4. 연결 성분 분석(CCL)으로 블롭 탐색
  5. 2단계 탐색: 1단계(150px 반경) → 2단계(30px 반경) 재확인
- **P제어:**
  ```
  error     = (frame_width / 2) - detected_line_x
  left_vel  = base_vel - error * k
  right_vel = -(base_vel + error * k)
  ```
- 발행 토픽: `topic_dxlpub` (`geometry_msgs/Vector3`)
  - `x`: 좌측 모터 속도
  - `y`: 우측 모터 속도
- 키보드: `s` → 주행 시작 / `q` → 정지

### 3. `dxl_nano` (Pi5에서 실행)
- `topic_dxlpub` 구독 → Dynamixel 모터 2개 구동
- 통신: USB 시리얼 `/dev/ttyUSB0`
- 모터 모델: `dxl.hpp`의 `#define DXL_MODEL`로 컴파일 시 선택
  - `MX12W` (기본, Protocol v1.0, 2Mbps)
  - `XC430W150` (Protocol v2.0, 4Mbps)
  - `XL430W250` (Protocol v2.0, 4Mbps)
- `Vector3.x` → 모터 ID 1 (좌), `Vector3.y` → 모터 ID 2 (우)

## QoS 설정

| 항목 | 값 |
|------|----|
| 신뢰성 | BestEffort |
| 히스토리 | KeepLast(10) |

> 퍼블리셔와 서브스크라이버 모두 동일하게 설정해야 통신됩니다.

---

## 사전 준비

### 공통
- 모든 머신에서 동일한 `ROS_DOMAIN_ID` 설정
  ```bash
  export ROS_DOMAIN_ID=5
  ```
- 네트워크가 같은 서브넷에 있어야 합니다

### Pi5
```bash
# Dynamixel 포트 권한
sudo chmod a+rw /dev/ttyUSB0

# 워크스페이스 빌드
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash
```

### WSL2
```bash
# 의존성 설치 (최초 1회)
sudo apt install ros-jazzy-cv-bridge libopencv-dev

# 워크스페이스 빌드
cd ~/ros2_ws
colcon build --packages-select linetracer_sim
source ~/ros2_ws/install/setup.bash
```

---

## 실행 방법

### Pi5 — 터미널 1: 영상 발행
```bash
source ~/ros2_ws/install/setup.bash
ros2 run linedetect_rapi5 pub_node
```

### Pi5 — 터미널 2: 모터 제어
```bash
sudo chmod a+rw /dev/ttyUSB0
source ~/ros2_ws/install/setup.bash
ros2 run dxl_nano sub_node
```

### WSL2 — 터미널: 라인 검출 + 제어 명령 발행
```bash
source ~/ros2_ws/install/setup.bash
ros2 run linetracer_sim linetracer_node
# 실행 후 's' 키: 주행 시작 / 'q' 키: 정지
```

---

## 파라미터 조정 (런타임)

`linetracer_sim` 실행 중 별도 터미널에서 P제어 파라미터를 동적으로 변경할 수 있습니다.

```bash
# 비례 이득 변경 (기본값: 0.13)
ros2 param set /line_tracker_node k 0.15

# 기본 속도 변경 (기본값: 50)
ros2 param set /line_tracker_node base_vel 60
```

---

## 토픽 확인

```bash
# 토픽 목록
ros2 topic list

# 영상 수신 확인
ros2 topic echo Image_Topic --no-arr

# 모터 명령 확인
ros2 topic echo topic_dxlpub
```

---

## 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| 모터가 안 움직임 | `ROS_DOMAIN_ID` 불일치 | 양쪽 머신에서 동일하게 설정 |
| 모터가 안 움직임 | QoS 불일치 | pub/sub 모두 `BestEffort` 확인 |
| `dynamixel open error` | 포트 권한 없음 | `sudo chmod a+rw /dev/ttyUSB0` |
| 영상이 안 보임 | 네트워크 문제 | `ping`으로 Pi5 ↔ WSL 통신 확인 |
| 라인 검출 실패 | 조명/영상 문제 | `k`, `base_vel` 파라미터 조정 |
