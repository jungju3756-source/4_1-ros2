# lidarsave

라이다 스캔 영상을 저장하고 키보드로 로봇을 원격 제어하는 패키지입니다.

---

## 패키지 구성

| 항목 | 내용 |
|------|------|
| 노드 이름 | `lidarsave_node` |
| 구독 토픽 | `/scan` (sensor_msgs/LaserScan) |
| 발행 토픽 | `/vel_cmd` (geometry_msgs/Vector3) |
| 저장 파일 | `lidar_save_output.mp4` (실행 경로에 저장) |

---

## 시스템 구성

```
[Raspberry Pi5]                    [WSL2]
  Rplidar C1
      ↓
  sllidar_node  →  /scan Topic  →  lidarsave_node
                                       ↓ 스캔영상 화면출력 + mp4 저장
  dxl_nano      ←  /vel_cmd    ←  키보드 입력 (WASD)
      ↓
  Dynamixel
```

---

## 스캔 영상 생성 원리

라이다의 `/scan` 토픽에서 각도(rad)와 거리(m)를 받아 2D 이미지로 변환합니다.

### 좌표 변환 공식

```
i번째 측정 각도 θ = angle_min + angle_increment × i

픽셀 좌표:
  px = 250 + (range × 250) × sin(π - θ)
  py = 250 - (range × 250) × cos(π - θ)
```

- 이미지 크기: 500 × 500 px = 2m × 2m 영역
- 스케일: 1m = 250px
- 라이다 위치: 이미지 중심 (250, 250) — 십자가로 표시
- 반경 1m 초과 장애물은 제외 (MAX_RANGE = 1.0m)
- 장애물: 빨간 점 (BGR: 0, 0, 255)으로 표시

### 동영상 저장

- 코덱: mp4v
- 프레임레이트: 10fps (라이다 토픽 주기와 동일)
- 저장 경로: 실행 위치의 `lidar_save_output.mp4`

---

## 키보드 제어

100ms 주기 타이머가 stdin을 폴링하여 키 입력을 감지하고 `/vel_cmd`를 발행합니다.

| 키 | 동작 | 좌모터 (x) | 우모터 (y) |
|----|------|-----------|-----------|
| `w` | 전진 | +50 | -50 |
| `s` | 후진 | -50 | +50 |
| `a` | 좌회전 | -50 | -50 |
| `d` | 우회전 | +50 | +50 |
| `space` | 정지 | 0 | 0 |
| `q` | 종료 | 0 | 0 |

> `/vel_cmd` Vector3: x = 좌모터 속도, y = 우모터 속도 (RPM)

---

## 동작 흐름

```
1. 실행 → 터미널 raw 모드 설정 + VideoWriter 초기화
2. /scan 수신 (10Hz)
   ├── 500×500 흰색 이미지 생성
   ├── 반경 1m 이내 장애물만 빨간 점으로 변환·그리기
   ├── imshow("LidarSave (2m x 2m)") 화면 출력
   └── VideoWriter로 mp4 프레임 저장
3. 키보드 타이머 (100ms)
   └── WASD 감지 → /vel_cmd 발행 → Pi5 dxl_nano → Dynamixel 구동
4. q 입력 → 정지 명령 발행 후 종료
```

---

## 빌드

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lidarsave
source install/setup.bash
```

---

## 실행 방법

### 1단계: Raspberry Pi5에서 실행

```bash
# 터미널 1 — 라이다 퍼블리셔
source ~/ros2_ws/install/setup.bash
ros2 run sllidar_ros2 sllidar_node

# 터미널 2 — 다이내믹셀 구독자
source ~/ros2_ws/install/setup.bash
ros2 run dxl_rapi5 dxl_nano
```

### 2단계: WSL2에서 실행

```bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=<Pi5와 동일한 값>
export ROS_LOCALHOST_ONLY=0

ros2 run lidarsave lidarsave_node
```

### 3단계: 조작

- 스캔 영상 창이 뜨면 키보드로 로봇 제어
- 로봇 주행 후 `q`로 종료하면 `lidar_save_output.mp4` 저장 완료
