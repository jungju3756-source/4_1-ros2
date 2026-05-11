# lidardrive

실제 라이다(`/scan` 토픽)를 실시간으로 수신하여 장애물 회피 알고리즘으로 자율주행하는 패키지입니다.

---

## 패키지 구성

| 항목 | 내용 |
|------|------|
| 노드 이름 | `lidardrive_node` |
| 구독 토픽 | `/scan` (sensor_msgs/LaserScan) |
| 발행 토픽 | `/vel_cmd` (geometry_msgs/Vector3) |
| 저장 파일 | `lidardrive_output.mp4` (시각화 결과 영상) |

---

## 시스템 구성

```
[Raspberry Pi5]                        [WSL2]
  Rplidar C1
      ↓
  sllidar_node  →  /scan Topic  →  lidardrive_node
                                     ├── 스캔 영상 생성 (2m×2m)
                                     ├── 장애물 회피 알고리즘
                                     ├── 화면 출력 + mp4 저장
                                     └── /vel_cmd 발행
  dxl_nano      ←  /vel_cmd   ←─────┘
      ↓
  Dynamixel
```

---

## 스캔 영상 생성 원리

`/scan` 토픽의 각 측정값을 2D 이미지 픽셀로 변환합니다.

### 좌표 변환 공식

```
i번째 측정 각도 θ = angle_min + angle_increment × i

픽셀 좌표:
  px = 250 + (range × 250) × sin(π - θ)
  py = 250 - (range × 250) × cos(π - θ)
```

- 이미지 크기: 500 × 500 px = 2m × 2m 영역
- 스케일: 1m = 250px
- 라이다 위치: 이미지 중심 (250, 250) — 검정 십자가
- 반경 1m 초과 장애물 제외 (MAX_RANGE = 1.0m)
- 장애물: 빨간 점 (BGR: 0, 0, 255)

---

## 장애물 회피 알고리즘

### Step 1. 전방 180도 영역 설정

이미지 상반부(y < 250)가 라이다 기준 전방 180도 영역입니다.

```
이미지 좌표계:
  (0,0)────────────(499,0)
    │   전방 180도   │
    │  좌측½ │ 우측½ │  ← y < 250 탐색
    │        │(250,250)
    │────────────────│  ← y = 250 (로봇 위치)
    │   후방 영역    │
  (0,499)────────(499,499)
```

### Step 2. 좌/우 최단거리 장애물 검출

전방 영역 픽셀을 순회하여 이미지 중심(250, 250)에서 가장 가까운 빨간 점을 좌/우 각각 1개씩 선택합니다.

```cpp
// 빨간 점 조건 (BGR): px[2]>150 && px[1]<80 && px[0]<80
// 중심에서 거리:  d = sqrt((x-250)² + (y-250)²)

// x < 250: 좌측 → 최소 d 점 → (lx, ly)
// x ≥ 250: 우측 → 최소 d 점 → (rx, ry)
```

### Step 3. 한쪽에 장애물이 없는 경우 처리

| 상황 | 처리 방식 |
|------|-----------|
| 좌측 장애물 없음 (error > 0) | lx=0, ly=249로 대체 → 우측 방향으로 회피 |
| 우측 장애물 없음 (error < 0) | rx=499, ry=249로 대체 → 좌측 방향으로 회피 |
| 전방 장애물 전혀 없음 | error=0 → 직진 |

### Step 4. 에러 계산

```
mid_x = (lx + rx) / 2    ← 두 최단 장애물의 중간 x좌표
error = mid_x - 250       ← 로봇 정면(x=250)과의 차이
```

| error | 의미 | 결과 |
|-------|------|------|
| = 0 | 정면 방향 균형 | 직진 |
| > 0 | 중앙이 오른쪽 → 우측 공간 넓음 | 우회전 |
| < 0 | 중앙이 왼쪽 → 좌측 공간 넓음 | 좌회전 |

### Step 5. 속도 명령 계산 (P제어)

```
BASE_VEL = 50,  K_GAIN = 0.15

left_vel  =  BASE_VEL + error × K_GAIN  =  50 + error × 0.15
right_vel = -(BASE_VEL - error × K_GAIN) = -(50 - error × 0.15)

예시 1) error=+100 (우측 공간 넓음 → 우회전)
  left_vel  = 50 + 100×0.15 = 65
  right_vel = -(50 - 100×0.15) = -35  → 우회전

예시 2) error=-100 (좌측 공간 넓음 → 좌회전)
  left_vel  = 50 - 100×0.15 = 35
  right_vel = -(50 + 100×0.15) = -65  → 좌회전

예시 3) error=0 (직진)
  left_vel  = 50
  right_vel = -50  → 직진
```

> `/vel_cmd` Vector3: x = 좌모터 속도, y = 우모터 속도

---

## 키보드 제어

별도 스레드(keyboardLoop)에서 `select()` + 100ms timeout으로 키 입력을 감지합니다.

| 키 | 동작 |
|----|------|
| `s` | 자율주행 시작 (mode=true → vel_cmd 발행) |
| `q` | 정지 (mode=false → vel_cmd = 0, 0) |

> `std::atomic<bool> mode_`로 키보드 스레드와 스캔 콜백 간 안전하게 상태 공유

---

## 시각화 영상 구성

| 색상 | 의미 |
|------|------|
| 빨간 점 | 반경 1m 이내 장애물 위치 |
| 녹색 화살표 | 좌측 최단거리 장애물 방향 |
| 빨간 화살표 | 우측 최단거리 장애물 방향 |
| 파란 화살표 | 중앙 방향 (로봇 진행 방향) |
| 검정 십자가 | 라이다(로봇) 위치 (250, 250) |
| 상단 텍스트 | [RUN/STOP] err=값 L=좌속도 R=우속도 |

---

## 동작 흐름

```
1. 실행
   ├── /scan 구독 등록
   ├── /vel_cmd 퍼블리셔 생성
   ├── VideoWriter 초기화 (10fps, lidardrive_output.mp4)
   └── 키보드 스레드 시작 (s/q 대기)

2. /scan 수신 시마다 scanCb() 호출 (~10Hz)
   ├── [영상 생성] 500×500 흰 배경 + 반경 1m 이내 빨간 점
   ├── [알고리즘] 전방 상반부에서 좌/우 최단 장애물 탐색
   ├── [에러 계산] error = mid_x - 250
   ├── [속도 계산] left/right vel = BASE_VEL ± error × K_GAIN
   ├── [발행] mode_=true면 계산값, false면 (0,0) 발행
   ├── [시각화] 화살표 그리기 + imshow 출력
   └── [저장] mp4 프레임 기록

3. q 입력 → vel_cmd(0,0) 발행 → 로봇 정지
   Ctrl+C  → 소멸자에서 정지 명령 발행 후 종료
```

---

## 빌드

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lidardrive
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

ros2 run lidardrive lidardrive_node
```

### 3단계: 조작

```
s 키 → 장애물 회피 자율주행 시작
          → /scan 수신 → 알고리즘 → /vel_cmd 발행 → Dynamixel 구동
q 키 → 정지 명령 발행 → 로봇 정지
```

- 실행 중 시각화 창에서 로봇이 감지하는 장애물과 진행 방향을 실시간으로 확인 가능
- 종료 시 `lidardrive_output.mp4`에 전체 주행 영상 저장
