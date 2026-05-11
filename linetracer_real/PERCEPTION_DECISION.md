# linetracer_real — 인지 · 판단 · 제어 상세 정리

---

## 인지 (Perception)

카메라에서 받은 이미지를 **"라인이 어디 있는지 분석할 수 있는 이진 영상"** 으로 변환하는 단계입니다.

---

### ⓪ 콜백 진입

```cpp
void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
auto startTime = std::chrono::steady_clock::now();
```

Pi5 카메라가 JPEG 이미지를 보낼 때마다 자동 호출됩니다.  
처리 시간 측정을 위해 시작 시각을 기록합니다.

---

### ① 파라미터 갱신

```cpp
k_        = this->get_parameter("k").as_double();
base_vel_ = this->get_parameter("base_vel").as_int();
```

`ros2 param set`으로 외부에서 값을 바꾸면 다음 프레임부터 즉시 반영됩니다.

| 파라미터 | 기본값 | 의미 |
|----------|--------|------|
| `k` | 0.14 | P 제어 비례 게인 |
| `base_vel` | 120 | 직진 기본 속도 |

> 생성자 초기화 리스트의 `k_=0.13`, `base_vel_=150` 은 곧바로 `get_parameter()`로 덮어써지므로 실제 사용값은 파라미터 기본값(0.14, 120)입니다.

---

### ② JPEG 디코딩

```cpp
cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
if (frame.empty()) return;
```

ROS 토픽으로 받은 데이터는 **JPEG 바이트 배열**입니다.  
`imdecode`로 OpenCV가 다룰 수 있는 **BGR 3채널 이미지**로 변환합니다.

```
msg->data              →    frame
[FF D8 FF E0 ...]           BGR 픽셀 행렬
(JPEG 바이트 배열)           (OpenCV Mat)
```

디코딩 실패 시 `frame.empty() = true` → `return`으로 조기 종료합니다.

---

### ③ ROI 추출

```cpp
cv::Rect roi_rect(0, frame.rows * 3 / 4, frame.cols, frame.rows / 4);
cv::Mat roi = frame(roi_rect).clone();
```

전체 프레임에서 **하단 1/4 영역만** 잘라냅니다.

```
┌──────────────────────┐  ↑
│                      │  │
│     (무시 영역)      │  │  rows × 3/4
│                      │  │
├──────────────────────┤  ↓  ← rows * 3/4
│   ROI (사용 영역)    │  ← rows / 4
└──────────────────────┘
```

**왜 하단 1/4만?**
- 라인은 항상 카메라 아래쪽(로봇 바로 앞)에 찍힘
- 위쪽은 벽, 천장, 원거리 배경 등 노이즈만 존재
- 처리 영역을 줄여 속도 향상

`.clone()` 으로 깊은 복사를 해서 이후 처리가 원본 `frame`에 영향을 주지 않습니다.

---

### ④ 그레이스케일 변환

```cpp
cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
```

```
BGR 3채널                →     GRAY 1채널
(B, G, R) per pixel            밝기값 0~255
```

**왜?**
- 라인 감지에는 색상 정보 불필요, 밝기만 필요
- 채널 3→1로 줄어 이후 처리 속도 향상
- 이진화를 위해 단일 채널 필요

---

### ⑤ 밝기 정규화

```cpp
roi += cv::Scalar(100) - cv::mean(roi);
```

ROI 전체 픽셀의 **평균 밝기를 항상 100으로 강제** 조정합니다.

```
보정값 = 100 - 현재평균
roi의 모든 픽셀 += 보정값
```

| 환경 | 현재 평균 | 보정값 | 결과 |
|------|----------|--------|------|
| 밝은 조명 | 200 | -100 | 평균 → 100 |
| 어두운 조명 | 40 | +60 | 평균 → 100 |
| 보통 조명 | 100 | 0 | 평균 → 100 |

**정규화가 없다면?**

```
밝은 환경 → 거의 모든 픽셀이 150 이상 → 이진화 후 전체 흰색 → 노이즈 폭발
어두운 환경 → 거의 모든 픽셀이 150 이하 → 이진화 후 전체 검정 → 라인 못 찾음
```

**정규화 후에는?**

```
어떤 환경이든 평균 = 100
→ 임계값 150 = "평균보다 +50 밝은 픽셀"
→ 항상 일관된 기준으로 라인 추출
```

---

### ⑥ 이진화

```cpp
cv::threshold(roi, roi, 150, 255, cv::THRESH_BINARY);
```

그레이스케일 영상을 **흑백 2값**으로 변환합니다.

```
픽셀값 > 150  →  255 (흰색) ← 라인 후보
픽셀값 ≤ 150  →    0 (검정) ← 배경
```

정규화로 평균=100이 된 상태에서 임계값 150의 의미:

```
평균(100)         임계값(150)              최대(255)
─────────────────────┼──────────────────────────
       배경(검정)     │        라인(흰색)
                  평균보다 +50 이상 밝은 픽셀만 흰색
```

이 로봇은 **어두운 바닥 위 흰색 라인** 환경을 가정합니다.  
밝은 바닥 위 검은 라인이라면 `THRESH_BINARY_INV` 를 써야 합니다.

---

### ⑦ Connected Component 분석

```cpp
int n_labels = cv::connectedComponentsWithStats(bin_roi, labels_, stats, centroids);
```

흰색 픽셀들을 **서로 연결된 덩어리(blob) 단위로 묶고** 각각의 정보를 계산합니다.

```
이진 영상                     분석 결과
┌────────────────┐            blob 0: 배경 (항상 제외)
│  ■■            │  →         blob 1: area=120,  center=(150, 30)
│  ■■■    ■■    │            blob 2: area=3200, center=(310, 45)  ← 라인
│       ■■■■    │            blob 3: area=55,   center=(480, 20)
└────────────────┘
```

| 출력 | 내용 |
|------|------|
| `labels_` | 각 픽셀이 몇 번 blob인지 표시한 행렬 |
| `stats` | 각 blob의 `[x, y, width, height, area]` |
| `centroids` | 각 blob의 중심좌표 `(cx, cy)` |

---

### 인지 단계 흐름 요약

```
CompressedImage (JPEG 바이트)
        ↓  imdecode
    BGR frame
        ↓  Rect(0, rows*3/4, cols, rows/4)
    ROI (하단 1/4)
        ↓  cvtColor
    GRAY 1채널
        ↓  += Scalar(100) - mean
    밝기 정규화 (평균=100)
        ↓  threshold(150)
    이진 영상 (흰=라인후보, 검=배경)
        ↓  connectedComponentsWithStats
    blob 목록 (stats, centroids)
        ↓
판단 단계로 전달
```

> 인지 단계의 목표는 **"어느 픽셀 덩어리가 라인인지 후보를 뽑아내는 것"** 이며,  
> 실제로 어떤 blob이 진짜 라인인지 고르는 것은 판단 단계에서 합니다.

---
---

## 판단 (Decision)

인지 단계에서 넘겨받은 blob 목록에서 **"진짜 라인"을 골라내고, 오차를 계산해서 주행 여부를 결정**하는 단계입니다.

---

### ① 후보 blob 필터링

```cpp
for (int i = 1; i < n_labels; i++) {
    int area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area > 100) { ... }
}
```

0번은 항상 배경이므로 1번부터 순회하고, **면적 100 이하는 노이즈로 무시**합니다.

```
blob 1: area=12   → ❌ 무시 (먼지, 작은 반사광)
blob 2: area=850  → ✅ 후보
blob 3: area=3200 → ✅ 후보
blob 4: area=55   → ❌ 무시
```

---

### ② [1단계] 넓은 그물 탐색 (반경 150px)

```cpp
double dist = cv::norm(cv::Point2d(cx, cy)
                     - cv::Point2d(last_line_x_, last_line_y_));

if (dist < min_dist && dist <= 150.0) {
    min_index = i;
}
```

**직전 프레임의 추적점**에서 150px 이내에 있는 blob 중 **가장 가까운 것**을 후보로 선택합니다.

```
직전 추적점 ★ (320, 45)

         ·········
       ·     A     ·   A: dist=40px  ✅ 후보 (최근접)
      · ★          ·   B: dist=210px ❌ 범위 밖
       ·           ·   C: dist=90px  ✅ 후보 (but A가 더 가까움)
         ·········
              C              B
```

**최초 `last_line_` 은 어떻게 정해지는가?**

```cpp
// 생성자에서 하드코딩
last_line_x_ = 320.0;   // 640px 기준 화면 중앙
last_line_y_ = 45.0;    // ROI 높이(약 60px) 중앙 부근
```

카메라 해상도가 다르다면 `카메라_가로 / 2.0` 으로 수정이 필요합니다.

**왜 150px?**  
한 프레임(약 25ms) 사이에 라인이 150px 이상 이동하지 않는다는 가정입니다.

---

### ③ 추적점 1차 갱신

```cpp
if (min_index != -1 && min_dist <= 150.0) {
    last_line_x_ = centroids.at<double>(min_index, 0);
    last_line_y_ = centroids.at<double>(min_index, 1);
}
```

1단계에서 찾은 후보의 중심점으로 추적점을 업데이트합니다.

```
직전 추적점:    (320, 45)
선택된 blob A:  center = (295, 42)
갱신 후:        last_line_x_ = 295,  last_line_y_ = 42
```

이 갱신된 점을 기준으로 바로 다음 ④단계에서 다시 탐색합니다.

---

### ④ [2단계] 좁은 그물 재탐색

```cpp
for (int i = 1; i < stats.rows; i++) {
    double d = cv::norm(cv::Point2d(cx, cy)
                      - cv::Point2d(last_line_x_, last_line_y_));
    if (d < best) {
        best = d;
        idx = i;
    }
}
```

1단계와의 차이점:

| | 1단계 | 2단계 |
|---|---|---|
| 탐색 기준점 | 직전 프레임 추적점 | **갱신된** 추적점 |
| 면적 필터 | `area > 100` 적용 | **없음** |
| 거리 상한 | 150px | **없음** (무조건 최근접) |
| 목적 | 후보 범위 좁히기 | 최종 blob 확정 |

---

### ⑤ 유효성 판정 (거리 ≤ 30px)

```cpp
if (best > 30.0) {
    idx = -1;
}
```

2단계에서 찾은 최근접 blob이 추적점에서 **30px 초과이면 무효** 처리합니다.

```
갱신된 추적점 ★ (295, 42)

  ★ ── 25px ── blob A  →  best=25 ≤ 30  →  idx=A  ✅ 유효
  ★ ── 50px ── blob B  →  best=50 > 30  →  idx=-1 ❌ 무효
```

**왜 2단계로 나누는가?**

```
1단계만 있으면:
  150px 안에 노이즈 blob이 있으면 끌려감 → 불안정

2단계만 있으면:
  30px 이내만 허용 → 라인이 조금만 빠르게 움직여도 놓침

2단계 조합:
  1단계: 150px 넓게 → 움직임 추적 (추적점 이동)
  2단계:  30px 좁게 → 정밀 확인 (헛발 방지)
```

**`idx = -1` 일 때 (라인 손실 처리)**

```
last_line_x_, last_line_y_ 는 변하지 않고 유지
→ 다음 프레임에 라인이 다시 나타나면 같은 위치에서 재탐색
→ 일시적 손실 후 자동 복구 가능
```

---

### ⑥ 오차 계산

```cpp
double error = (bin_roi.cols / 2.0) - last_line_x_;
```

| 라인 위치 | error 값 | 의미 |
|----------|---------|------|
| 왼쪽 치우침 | 양수(+) | 로봇이 왼쪽으로 회전해야 함 |
| 중앙 | 0 | 직진 |
| 오른쪽 치우침 | 음수(-) | 로봇이 오른쪽으로 회전해야 함 |

```
x=0 ──── 라인(150) ──── 중앙(320) ──────────── x=640
error = 320 - 150 = +170  (라인이 왼쪽, 양수)

x=0 ──────────── 중앙(320) ──────── 라인(500) ── x=640
error = 320 - 500 = -180  (라인이 오른쪽, 음수)
```

---

### ⑦ 주행 모드 판정

```cpp
if (mode_) {
    vel_msg.x = base_vel_ - error * k_;        // 좌측 모터
    vel_msg.y = -(base_vel_ + error * k_);     // 우측 모터
} else {
    vel_msg.x = 0;  vel_msg.y = 0;
}
```

`mode_` 는 키보드 스레드가 별도로 관리하는 `atomic<bool>` 입니다.

```
's' 입력 → mode_ = true  → P 제어 계산 → 모터 구동
'q' 입력 → mode_ = false → 속도 0      → 정지
```

`atomic<bool>` 을 사용하는 이유: 키보드 스레드와 콜백 스레드가 **동시에 접근**하기 때문에 race condition 방지를 위함입니다.

**error에 따른 모터 출력 예시 (base_vel=120, k=0.14):**

| 상황 | error | 좌측 모터 | 우측 모터 | 결과 |
|------|-------|----------|----------|------|
| 라인 중앙 | 0 | 120 | -120 | 직진 |
| 라인 왼쪽 | +170 | 96.2 | -143.8 | 좌회전 |
| 라인 오른쪽 | -180 | 145.2 | -94.8 | 우회전 |

---

### 판단 단계 흐름 요약

```
[인지에서 받은 것]
bin_roi, stats, centroids (blob 목록)
           ↓
① 면적 > 100 필터링 (노이즈 제거)
           ↓
② 직전 추적점 기준 150px 이내 최근접 탐색
           ↓
③ 추적점 1차 갱신 (last_line_x_, last_line_y_)
           ↓
④ 갱신된 추적점 기준 재탐색 (면적 필터 없음)
           ↓
⑤ 거리 ≤ 30px?  →  Yes: 유효  /  No: idx=-1 (손실, 추적점 유지)
           ↓
⑥ error = (cols/2) - last_line_x_
           ↓
⑦ mode_?  →  ON: P제어 계산  /  OFF: 속도=0
           ↓
[제어로 넘기는 것]
vel_msg.x (좌측 모터), vel_msg.y (우측 모터)
```

> 판단 단계의 핵심은 **"여러 흰색 덩어리 중 시간적 연속성을 이용해 진짜 라인만 추적"** 하는 것입니다.  
> 단순히 가장 큰 blob을 고르는 것이 아니라, **직전 위치에서 가장 가까운 blob을 2단계로 확인**하는 방식으로 노이즈에 강건합니다.

---
---

## 제어 (Control)

판단 단계에서 계산된 오차(error)를 바탕으로 **P 제어로 좌우 모터 속도를 결정하고 Pi5로 퍼블리시**하는 단계입니다.

---

### ① 주행 모드 분기

```cpp
if (mode_) {
    // 주행: P 제어 계산
} else {
    vel_msg.x = 0;
    vel_msg.y = 0;
}
```

`mode_` 는 키보드 스레드가 설정하는 `atomic<bool>` 입니다.

```
's' 키 → mode_ = true  → P 제어 계산 실행
'q' 키 → mode_ = false → 속도 0 으로 정지
```

**`atomic<bool>` 을 쓰는 이유:**

키보드 스레드와 image_callback 스레드가 **동시에** `mode_` 를 읽고 쓰기 때문에 일반 `bool` 이면 race condition이 발생할 수 있습니다.  
`atomic<bool>` 은 하드웨어 수준에서 동시 접근을 안전하게 처리합니다.

```
image_callback 스레드:  mode_ 읽기  (매 프레임)
keyboardLoop 스레드:    mode_ 쓰기  ('s'/'q' 입력 시)
→ atomic 이 없으면 동시 접근 시 값이 깨질 수 있음
```

---

### ② P 제어 계산

```cpp
vel_msg.x =   base_vel_ - error * k_;   // 좌측 모터
vel_msg.y = -(base_vel_ + error * k_);  // 우측 모터
```

#### P 제어란?

**Proportional Control (비례 제어)** — 오차에 비례해서 보정값을 결정합니다.

```
보정값 = error × k_
         ↑          ↑
       오차       비례 게인
```

오차가 클수록 보정이 강해지고, 오차가 0이 되면 보정도 0이 됩니다.

---

#### 좌측/우측 모터 수식 해석

```
좌측 = base_vel_ - error × k_
우측 = -(base_vel_ + error × k_)
```

우측에 `-` 가 붙는 이유: Dynamixel 모터의 **차동 구동(Differential Drive)** 구조 때문입니다.

```
로봇을 위에서 본 모습:

    ← 좌측 모터 (정방향 +)
    → 우측 모터 (역방향 -)

직진 시:
  좌측 = +120  (앞으로)
  우측 = -120  (앞으로, 반대 방향으로 달려있으므로 부호 반전)
```

---

#### 라인 위치별 모터 출력 (base_vel=120, k=0.14)

**라인이 중앙 (error = 0)**

```
좌측 = 120 - 0 = 120
우측 = -(120 + 0) = -120

→ 양쪽 동일 속도 → 직진
```

**라인이 왼쪽 (error = +170)**

```
좌측 = 120 - (170 × 0.14) = 120 - 23.8 =  96.2  (감속)
우측 = -(120 + (170 × 0.14)) = -(143.8) = -143.8  (가속)

좌측 느림 + 우측 빠름 → 왼쪽으로 회전 → 라인 쪽으로 복귀 ✅
```

**라인이 오른쪽 (error = -180)**

```
좌측 = 120 - (-180 × 0.14) = 120 + 25.2 = 145.2  (가속)
우측 = -(120 + (-180 × 0.14)) = -(94.8) = -94.8   (감속)

좌측 빠름 + 우측 느림 → 오른쪽으로 회전 → 라인 쪽으로 복귀 ✅
```

**요약 표 (base_vel=120, k=0.14)**

| 상황 | error | 좌측 모터 | 우측 모터 | 결과 |
|------|-------|----------|----------|------|
| 라인 중앙 | 0 | +120 | -120 | 직진 |
| 라인 왼쪽 (약) | +50 | +113 | -127 | 약한 좌회전 |
| 라인 왼쪽 (강) | +170 | +96.2 | -143.8 | 강한 좌회전 |
| 라인 오른쪽 (약) | -50 | +127 | -113 | 약한 우회전 |
| 라인 오른쪽 (강) | -180 | +145.2 | -94.8 | 강한 우회전 |

---

#### 게인 k 의 역할

```
k 가 너무 작으면 (예: 0.01):
  보정값이 너무 약함 → 라인에서 벗어나도 거의 안 꺾임 → 라인 이탈

k 가 너무 크면 (예: 1.0):
  보정값이 너무 강함 → 조금만 벗어나도 과격하게 꺾임 → 좌우 진동(oscillation)

k = 0.14 (현재):
  적당한 반응 속도로 부드럽게 라인 추적
```

```
error=100 일 때 k에 따른 보정값 비교:

k=0.01 → 보정값=1    (거의 직진)
k=0.14 → 보정값=14   (적당한 꺾임)
k=0.50 → 보정값=50   (큰 꺾임)
k=1.00 → 보정값=100  (속도가 0이 될 수도 있음)
```

런타임 중 튜닝:
```bash
ros2 param set /line_tracker_node k 0.15      # 게인 증가 (더 민감)
ros2 param set /line_tracker_node k 0.10      # 게인 감소 (더 완만)
```

---

### ③ Vector3 퍼블리시

```cpp
vel_pub_->publish(vel_msg);
// topic: topic_dxlpub
// type:  geometry_msgs/Vector3
// QoS:  Reliable KeepLast(10)
```

계산된 속도를 Pi5의 `dxl_rapi5` 노드로 전송합니다.

**Vector3 필드 사용 규칙:**

```
vel_msg.x = 좌측 모터 속도  (양수 = 정방향)
vel_msg.y = 우측 모터 속도  (음수 = 정방향, 차동 구동)
vel_msg.z = 사용 안 함 (0)
```

**왜 Reliable QoS?**

```
BestEffort (카메라 구독):
  패킷 손실 허용, 최신 프레임만 중요
  → 영상은 1~2장 빠져도 바로 다음 프레임이 옴

Reliable (모터 퍼블리시):
  패킷 손실 불허, 재전송 보장
  → 모터 명령이 손실되면 로봇이 예상치 못한 동작을 할 수 있음
```

---

### ④ 성능 측정 및 로그

```cpp
auto endTime = std::chrono::steady_clock::now();
float totalTime = std::chrono::duration<float, std::milli>(endTime - startTime).count();
RCLCPP_INFO(this->get_logger(), "err:%.2lf lvel:%.2f rvel:%.2f time:%.2f",
            error, vel_msg.x, vel_msg.y, totalTime);
```

매 프레임마다 터미널에 출력됩니다:

```
[INFO] err:12.50 lvel:118.25 rvel:-121.75 time:3.42
        ↑           ↑             ↑            ↑
      오차       좌측 모터    우측 모터    처리시간(ms)
```

**처리 시간(time) 의미:**
- 낮을수록 좋음 (빠른 처리 = 높은 제어 주기)
- 카메라 fps 대비 처리 시간이 길면 큐에 이미지가 쌓임
- 3~5ms 수준이면 30fps 카메라에 충분

---

### 키보드 스레드 — keyboardLoop()

제어 단계와 직접 연결되는 별도 스레드입니다.

```cpp
void LineTrackerProcessor::keyboardLoop() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);         // 현재 터미널 설정 백업
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);       // raw 모드: 즉시 입력, 화면 출력 없음
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while (running_) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        struct timeval tv = {0, 100000};    // 100ms timeout
        if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0) {
            char ch = getchar();
            if (ch == 'q') { mode_ = false; RCLCPP_WARN(..., "STOP"); }
            else if (ch == 's') { mode_ = true;  RCLCPP_INFO(..., "START"); }
        }
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // 터미널 복구
}
```

**termios raw 모드가 필요한 이유:**

기본 터미널은 **canonical 모드**로 Enter 키를 눌러야 입력이 전달됩니다.

```
canonical 모드:  s → Enter → 전달  (실시간 제어 불가)
raw 모드:        s → 즉시 전달     (실시간 제어 가능) ✅
```

**`select` 100ms timeout 이유:**

```
blocking 방식 (getchar만 사용):
  입력이 없으면 영원히 대기 → running_=false 가 되도 종료 못 함

select 100ms timeout:
  입력 없으면 100ms 후 루프 재진입 → running_ 확인 → 정상 종료 ✅
```

**소멸자에서 터미널 복구:**

```cpp
~LineTrackerProcessor() {
    running_ = false;        // keyboardLoop while 탈출
    key_thread_.join();      // 스레드 종료 대기
}
// keyboardLoop 종료 시 tcsetattr(oldt) 실행 → 터미널 정상 복구
```

복구하지 않으면 노드 종료 후 터미널에서 입력한 문자가 보이지 않는 등 셸이 깨집니다.

---

### 제어 단계 흐름 요약

```
[판단에서 받은 것]
error, mode_
           ↓
① mode_ 확인 (atomic<bool>)
   ├── true  → P 제어 계산
   │           vel_msg.x =   base_vel_ - error × k_
   │           vel_msg.y = -(base_vel_ + error × k_)
   └── false → vel_msg.x = 0 / vel_msg.y = 0
           ↓
② vel_pub_->publish(vel_msg)
   topic: topic_dxlpub  /  Reliable QoS
           ↓
③ 처리시간 계산 및 로그 출력
   err / lvel / rvel / time(ms)
           ↓
[Pi5로 전달]
topic_dxlpub → dxl_rapi5 → Dynamixel 모터
```

> 제어 단계의 핵심은 **"오차에 비례한 속도 차이로 라인을 향해 부드럽게 방향을 틀어가는 것"** 입니다.  
> 게인 `k` 가 이 반응 강도를 결정하며, `ros2 param set` 으로 주행 중에도 실시간 튜닝이 가능합니다.
