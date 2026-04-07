# lidarsim

`lidarsave`로 저장한 라이다 스캔 동영상(mp4)을 재생하면서 장애물 회피 알고리즘을 시뮬레이션하고, `/vel_cmd`를 발행해 실제 로봇을 구동하는 패키지입니다.

---

## 패키지 구성

| 항목 | 내용 |
|------|------|
| 노드 이름 | `lidarsim_node` |
| 입력 파일 | `/home/aim88/simulation/lidarplot2.mp4` |
| 발행 토픽 | `/vel_cmd` (geometry_msgs/Vector3) |
| 저장 파일 | `lidarsim_output.mp4` (처리 결과 영상) |

---

## 시스템 구성

```
[WSL2]
  lidarplot2.mp4 (lidarsave로 저장한 스캔 영상)
        ↓ (10fps 타이머로 프레임 읽기)
  lidarsim_node
    ├── 장애물 회피 알고리즘
    ├── 시각화 영상 화면 출력
    └── lidarsim_output.mp4 저장
        ↓
  /vel_cmd Topic

[Raspberry Pi5]
  dxl_nano  ←  /vel_cmd  ←  Dynamixel 구동
```

---

## 장애물 회피 알고리즘

입력 mp4는 `lidarsave`가 생성한 스캔 영상으로, 장애물이 빨간 점(R>150, G<80, B<80)으로 표시되어 있습니다.

### Step 1. 전방 영역 탐색

이미지 상반부 (y < 250) 가 라이다 기준 전방 180도 영역에 해당합니다.

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

전방 영역을 픽셀 단위로 순회하며 중심(250, 250)에서 가장 가까운 빨간 점을 좌/우 각각 1개씩 선택합니다.

```cpp
// 중심에서 픽셀 거리
float d = sqrt((x-250)² + (y-250)²)

// 좌측 ½ (x < 250): 최소 d → (lx, ly)
// 우측 ½ (x ≥ 250): 최소 d → (rx, ry)
```

### Step 3. 한쪽에 장애물이 없는 경우

| 상황 | 처리 |
|------|------|
| 좌측 장애물 없음 | lx=0, ly=249 (좌측 경계) 로 대체 |
| 우측 장애물 없음 | rx=499, ry=249 (우측 경계) 로 대체 |
| 양쪽 모두 없음 | error=0 → 직진 |

### Step 4. 에러 계산

두 최단거리 장애물의 중앙 방향을 로봇 정면(x=250)과 비교합니다.

```
mid_x = (lx + rx) / 2
error = mid_x - 250
```

| error 값 | 의미 | 동작 |
|----------|------|------|
| = 0 | 균형 또는 장애물 없음 | 직진 |
| > 0 | 중앙이 오른쪽 (좌측에 장애물 많음) | 우회전 |
| < 0 | 중앙이 왼쪽 (우측에 장애물 많음) | 좌회전 |

### Step 5. 속도 명령 계산

```
left_vel  =  BASE_VEL + error × K_GAIN  =  50 + error × 0.3
right_vel = -(BASE_VEL - error × K_GAIN) = -(50 - error × 0.3)

예시 1) error=+50 (우측 공간 넓음 → 우회전)
  left_vel  = 50 + 50×0.3 = 65
  right_vel = -(50 - 50×0.3) = -35  → 우회전

예시 2) error=-50 (좌측 공간 넓음 → 좌회전)
  left_vel  = 50 - 50×0.3 = 35
  right_vel = -(50 + 50×0.3) = -65  → 좌회전

예시 3) error=0 (직진)
  left_vel  = 50
  right_vel = -50  → 직진
```

---

## 키보드 제어

별도 스레드(keyboardLoop)에서 `select()` + 100ms timeout으로 키 입력을 감지합니다.

| 키 | 동작 |
|----|------|
| `s` | 장애물 회피 주행 시작 (mode=true) |
| `q` | 정지, vel_cmd=(0,0) 발행 (mode=false) |

> 키보드 스레드는 `std::atomic<bool> mode_`로 메인 스레드와 안전하게 상태를 공유합니다.

---

## 시각화 영상 구성

| 색상 | 의미 |
|------|------|
| 빨간 점 | 원본 스캔 영상의 장애물 |
| 녹색 화살표 | 좌측 최단거리 장애물 방향 |
| 빨간 화살표 | 우측 최단거리 장애물 방향 |
| 파란 화살표 | 중앙 방향 (로봇 진행 방향) |
| 검정 십자가 | 라이다(로봇) 위치 |
| 상단 텍스트 | [RUN/STOP] error=값 L=좌속도 R=우속도 |

---

## 동작 흐름

```
1. 실행 → lidarplot2.mp4 열기 + VideoWriter 초기화 + 키보드 스레드 시작
2. 10Hz 타이머 콜백 (timerCb)
   ├── mp4에서 프레임 1장 읽기 (500×500)
   ├── 전방 상반부(y<250)에서 좌/우 최단 빨간 점 탐색
   ├── mid_x 계산 → error = mid_x - 250
   ├── left_vel / right_vel 계산
   ├── mode_=true면 /vel_cmd 발행, false면 (0,0) 발행
   ├── 화살표 시각화 후 imshow 출력
   └── lidarsim_output.mp4 프레임 저장
3. mp4 끝 도달 → 정지 명령 발행 후 자동 종료
```

---

## 빌드

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lidarsim
source install/setup.bash
```

---

## 실행 방법

### 1단계: Raspberry Pi5에서 실행

```bash
# 다이내믹셀 구독자
source ~/ros2_ws/install/setup.bash
ros2 run dxl_rapi5 dxl_nano
```

### 2단계: WSL2에서 실행

```bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=<Pi5와 동일한 값>
export ROS_LOCALHOST_ONLY=0

ros2 run lidarsim lidarsim_node
```

### 3단계: 조작

```
s 키 → 장애물 회피 시뮬레이션 시작 + /vel_cmd 발행 → 로봇 구동
q 키 → 정지
영상 끝  → 자동 정지 및 종료
```

- 처리 결과는 실행 경로의 `lidarsim_output.mp4`에 저장됩니다.
