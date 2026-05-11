# linetracer_real 블럭도

자율주행 3대 기능(**인지 → 판단 → 제어**)에 따른 `linetracer_real` 노드 상세 블럭도입니다.
인지·판단·제어는 모두 **WSL2 쪽 `linetracer_real` 노드** 안에서 수행되며, 센서(카메라)와 액추에이터(Dynamixel)만 Pi5에 위치합니다.

## 전체 시스템 블럭도

```mermaid
flowchart LR
    subgraph PI5_SENSOR["🔵 센서 (Raspberry Pi5)"]
        CAM["카메라<br/>camera_ros2<br/>libcamerasrc → JPEG"]
    end

    subgraph WSL["🟩🟩🟩  WSL2 — linetracer_real 노드 (자율주행 SW)  🟩🟩🟩"]
        direction LR
        subgraph PERC_BOX["🟢 인지 (Perception)"]
            PERC["영상 전처리<br/>+ 라인 특징 추출"]
        end
        subgraph DECI_BOX["🟡 판단 (Decision)"]
            DECI["라인 확정<br/>+ 오차 계산<br/>+ 모드 결정"]
        end
        subgraph CTRL_BOX["🔴 제어 (Control)"]
            CTRL["P 제어<br/>+ Vector3 퍼블리시"]
        end
        PERC -->|"환경인식<br/>(bin_roi, stats, centroids)"| DECI
        DECI -->|"판단결과<br/>(error, mode_)"| CTRL
    end

    subgraph PI5_ACT["🔴 액추에이터 (Raspberry Pi5)"]
        DXL["Dynamixel 모터<br/>dxl_rapi5"]
    end

    CAM -->|"센싱정보<br/>Image_Topic<br/>CompressedImage<br/>(BestEffort QoS)"| PERC
    CTRL -->|"제어명령<br/>topic_dxlpub<br/>Vector3<br/>(Reliable QoS)"| DXL

    classDef sensor fill:#cce5ff,stroke:#0066cc,stroke-width:2px,color:#000
    classDef wsl_outer fill:#e8f5e9,stroke:#1b5e20,stroke-width:4px,color:#000
    classDef perc fill:#c8e6c9,stroke:#2e7d32,stroke-width:2px,color:#000
    classDef deci fill:#fff9c4,stroke:#f57f17,stroke-width:2px,color:#000
    classDef ctrl fill:#ffcdd2,stroke:#c62828,stroke-width:2px,color:#000
    classDef actuator fill:#f8d7da,stroke:#dc3545,stroke-width:2px,color:#000
    class CAM sensor
    class WSL wsl_outer
    class PERC_BOX,PERC perc
    class DECI_BOX,DECI deci
    class CTRL_BOX,CTRL ctrl
    class DXL actuator
```

---

## 상세 블럭도 (인지 → 판단 → 제어, WSL2 노드 내부)

```mermaid
flowchart TD
    CAM["📷 Pi5 카메라<br/>Image_Topic<br/>CompressedImage (JPEG, BestEffort)"]:::sensor

    subgraph WSL["🟩 WSL2 — linetracer_real (line_tracker_node)"]
        direction TB

        %% ============================================================
        %% 인지 (Perception)
        %% ============================================================
        subgraph PERC["🟢 인지 (Perception) — image_callback() 진입"]
            direction TB
            P0["⓪ 콜백 진입<br/>image_callback(CompressedImage::SharedPtr msg)<br/>startTime = steady_clock::now()"]
            P1["① 파라미터 런타임 갱신<br/>k_ = get_parameter('k').as_double()<br/>base_vel_ = get_parameter('base_vel').as_int()<br/>(매 프레임 갱신 → 튜닝 즉시 반영)"]
            P2["② JPEG 디코딩<br/>cv::imdecode(cv::Mat(msg->data), IMREAD_COLOR)<br/>→ BGR Frame<br/>실패 시 early return"]
            P3["③ ROI 추출 — setROI()<br/>cv::Rect(0, rows×3/4, cols, rows/4)<br/>하단 1/4 영역만 깊은 복사<br/>(원본 frame 훼손 방지)"]
            P4["④ 그레이스케일 변환<br/>cv::cvtColor(roi, roi, COLOR_BGR2GRAY)<br/>3채널(BGR) → 1채널(밝기)"]
            P5["⑤ 밝기 정규화<br/>roi += Scalar(100) − cv::mean(roi)<br/>ROI 평균 밝기를 100으로 강제<br/>→ 조명 변화에 강건"]
            P6["⑥ 이진화<br/>cv::threshold(roi, roi, 150, 255, THRESH_BINARY)<br/>픽셀 > 150 → 255 (흰색, 객체)<br/>픽셀 ≤ 150 → 0 (배경)"]
            P7["⑦ Connected Component 분석<br/>cv::connectedComponentsWithStats(<br/>  bin_roi, labels_, stats, centroids)<br/>→ n_labels 개의 blob<br/>  stats: [x,y,w,h,area]<br/>  centroids: (cx,cy)"]

            P0 --> P1 --> P2 --> P3 --> P4 --> P5 --> P6 --> P7
        end

        %% ============================================================
        %% 판단 (Decision)
        %% ============================================================
        subgraph DECI["🟡 판단 (Decision) — findLine() + 오차/모드 판정"]
            direction TB
            D1["① 후보 blob 필터링<br/>for i in 1..n_labels<br/>  if stats(i, CC_STAT_AREA) > 100"]
            D2["② [1단계] 최근접 후보 탐색<br/>for each blob:<br/>  dist = norm(centroid − (last_x, last_y))<br/>  if dist < min_dist && dist ≤ 150.0<br/>     min_index = i<br/>→ 넓은 그물(150px)로 추적 후보 선정"]
            D3["③ 추적점 1차 갱신<br/>if min_index ≠ −1 && min_dist ≤ 150:<br/>  last_line_x_ = centroids(min_index, 0)<br/>  last_line_y_ = centroids(min_index, 1)"]
            D4["④ [2단계] 재탐색 (면적 필터 없음)<br/>for i in 1..stats.rows:<br/>  d = norm(centroid − (last_x, last_y))<br/>  if d < best: idx = i<br/>→ 갱신된 추적점 기준 최근접"]
            D5{"⑤ best ≤ 30.0 px?"}
            D6["best_idx = idx<br/>(라인 유효 검출)"]
            D7["best_idx = −1<br/>(라인 손실 / 노이즈)<br/>last_line_ 은 유지"]
            D8["⑥ 오차 계산<br/>error = (bin_roi.cols / 2.0) − last_line_x_<br/>양수 → 라인이 왼쪽 치우침<br/>음수 → 라인이 오른쪽 치우침"]
            D9{"⑦ 주행 모드?<br/>mode_ (atomic bool)<br/>키보드 스레드가 설정"}

            D1 --> D2 --> D3 --> D4 --> D5
            D5 -->|Yes| D6
            D5 -->|No| D7
            D6 --> D8
            D7 --> D8
            D8 --> D9
        end

        %% ============================================================
        %% 제어 (Control)
        %% ============================================================
        subgraph CTRL["🔴 제어 (Control) — P-Controller + 퍼블리시"]
            direction TB
            C1["① 주행 (mode_ = true)<br/>vel_msg.x = base_vel_ − error × k_<br/>vel_msg.y = −(base_vel_ + error × k_)<br/>(차동 구동: 좌우 부호 반대)"]
            C2["② 정지 (mode_ = false)<br/>vel_msg.x = 0<br/>vel_msg.y = 0"]
            C3["③ 퍼블리시<br/>vel_pub_->publish(vel_msg)<br/>topic: topic_dxlpub<br/>type: geometry_msgs/Vector3<br/>QoS: Reliable KeepLast(10)"]
            C4["④ 성능 측정 / 로그<br/>totalTime = endTime − startTime (ms)<br/>RCLCPP_INFO('err:%.2lf lvel:%.2f<br/>rvel:%.2f time:%.2f', ...)"]

            C1 --> C3
            C2 --> C3
            C3 --> C4
        end

        %% ============================================================
        %% 디버그 시각화 (사이드 블록)
        %% ============================================================
        subgraph VIS["🖥️ 디버그 시각화 — drawResult()"]
            direction TB
            V1["① cvtColor(bin_roi, GRAY2BGR)<br/>컬러 도형을 그리기 위함"]
            V2["② for each blob (area ≥ 100):<br/>  if i == best_idx:<br/>    빨간 굵은 박스 (0,0,255) thick=2<br/>  else:<br/>    파란 얇은 박스 (255,0,0) thick=1"]
            V3["③ 추적점 원 표시<br/>cv::circle((last_x,last_y), r=3,<br/>  (0,0,255), −1)"]
            V4["④ cv::imshow('1. Raw Video', frame)<br/>cv::imshow('2. Binary Debug', display)<br/>cv::waitKey(1)"]
            V1 --> V2 --> V3 --> V4
        end

        %% ============================================================
        %% 키보드 입력 스레드 (별도 std::thread)
        %% ============================================================
        subgraph KEY["⌨️ 키보드 스레드 — keyboardLoop()"]
            direction TB
            K1["① termios 설정<br/>c_lflag &= ~(ICANON | ECHO)<br/>→ raw 모드 (즉시 문자 입력)"]
            K2["② while (running_):<br/>  select(STDIN, 100ms)"]
            K3{"입력 문자?"}
            K4["'s' → mode_ = true<br/>RCLCPP_INFO('START')"]
            K5["'q' → mode_ = false<br/>RCLCPP_WARN('STOP')"]
            K6["③ 종료 시 tcsetattr(oldt)<br/>터미널 원상복구"]
            K1 --> K2 --> K3
            K3 -->|s| K4
            K3 -->|q| K5
            K4 --> K2
            K5 --> K2
            K2 -.->|running_=false| K6
        end

        PERC ==>|"bin_roi<br/>stats<br/>centroids"| DECI
        D9 -->|"ON"| C1
        D9 -->|"OFF"| C2
    end

    DXL["⚙️ Pi5 Dynamixel 모터<br/>dxl_rapi5<br/>(Vector3.x = 좌측 모터<br/>Vector3.y = 우측 모터)"]:::actuator

    %% ============================================================
    %% 외부 연결
    %% ============================================================
    CAM ==>|"센싱정보<br/>BestEffort QoS"| P0
    C3 ==>|"제어명령<br/>Reliable QoS"| DXL

    %% 시각화 데이터 흐름
    P2 -.->|"frame"| V4
    P7 -.->|"bin_roi"| V1
    D6 -.->|"best_idx<br/>last_x, last_y"| V2
    D6 -.->|"last_x, last_y"| V3

    %% 키보드 → 판단 모드
    K4 -.->|"atomic mode_=true"| D9
    K5 -.->|"atomic mode_=false"| D9

    classDef sensor fill:#cce5ff,stroke:#0066cc,stroke-width:2px,color:#000
    classDef actuator fill:#f8d7da,stroke:#dc3545,stroke-width:2px,color:#000
    classDef wsl_outer fill:#e8f5e9,stroke:#1b5e20,stroke-width:4px,color:#000
    classDef perc fill:#c8e6c9,stroke:#2e7d32,stroke-width:2px,color:#000
    classDef deci fill:#fff9c4,stroke:#f57f17,stroke-width:2px,color:#000
    classDef ctrl fill:#ffcdd2,stroke:#c62828,stroke-width:2px,color:#000
    classDef debug fill:#fff3cd,stroke:#856404,stroke-width:1px,color:#000
    classDef thread fill:#e2d5f0,stroke:#6f42c1,stroke-width:1px,color:#000
    class WSL wsl_outer
    class PERC perc
    class DECI deci
    class CTRL ctrl
    class VIS debug
    class KEY thread
```

---

## 초기화 블럭도 (생성자 / 소멸자)

```mermaid
flowchart TD
    subgraph INIT["🟣 노드 초기화 — LineTrackerProcessor::LineTrackerProcessor()"]
        direction TB
        I1["① Node 부모 생성자<br/>Node('line_tracker_node')"]
        I2["② 멤버 초기화 리스트<br/>mode_ = false (정지 상태)<br/>k_ = 0.13, base_vel_ = 150<br/>running_ = true"]
        I3["③ 파라미터 선언<br/>declare_parameter('k', 0.14)<br/>declare_parameter('base_vel', 120)"]
        I4["④ 파라미터 초기값 로드<br/>k_ = get_parameter('k')<br/>base_vel_ = get_parameter('base_vel')"]
        I5["⑤ QoS 정의<br/>sub_qos = KeepLast(10).best_effort()<br/>pub_qos = KeepLast(10) [Reliable]"]
        I6["⑥ 구독자 생성<br/>raw_sub_ = create_subscription<br/>  ('Image_Topic', sub_qos, image_callback)"]
        I7["⑦ 퍼블리셔 생성<br/>vel_pub_ = create_publisher<br/>  ('topic_dxlpub', pub_qos)"]
        I8["⑧ 추적점 초기화<br/>last_line_x_ = 320.0 (화면 중앙)<br/>last_line_y_ = 45.0 (ROI 중앙)"]
        I9["⑨ 키보드 스레드 시작<br/>key_thread_ = std::thread(keyboardLoop)"]
        I1 --> I2 --> I3 --> I4 --> I5 --> I6 --> I7 --> I8 --> I9
    end

    subgraph TERM["🟥 노드 종료 — ~LineTrackerProcessor()"]
        direction TB
        T1["① running_ = false<br/>키보드 루프 탈출 신호"]
        T2["② key_thread_.join()<br/>스레드 종료 대기"]
        T1 --> T2
    end

    I9 -.-> TERM

    classDef init fill:#e1bee7,stroke:#6a1b9a,stroke-width:2px,color:#000
    classDef term fill:#ffccbc,stroke:#bf360c,stroke-width:2px,color:#000
    class INIT init
    class TERM term
```

---

## 단계별 상세 요약

### 🟢 인지 (Perception) — WSL2 내부
입력된 압축 이미지로부터 **라인 후보가 될 흰색 blob들**을 추출합니다.

| 단계 | 처리 내용 | 핵심 코드 |
|------|----------|-----------|
| ⓪ | 콜백 진입 / 시간 측정 시작 | `steady_clock::now()` |
| ① | 런타임 파라미터 갱신 | `get_parameter("k"/"base_vel")` |
| ② | JPEG → BGR 디코딩 | `cv::imdecode(msg->data, IMREAD_COLOR)` |
| ③ | ROI 추출 (하단 1/4) | `cv::Rect(0, rows*3/4, cols, rows/4)` |
| ④ | 그레이스케일 변환 | `cv::cvtColor(COLOR_BGR2GRAY)` |
| ⑤ | 밝기 정규화 (평균=100) | `roi += Scalar(100) - cv::mean(roi)` |
| ⑥ | 이진화 (threshold=150) | `cv::threshold(150, 255, THRESH_BINARY)` |
| ⑦ | 연결 요소 분석 | `cv::connectedComponentsWithStats` |

### 🟡 판단 (Decision) — WSL2 내부
이진 영상에서 **진짜 라인을 식별**하고 **오차**와 **주행모드**를 결정합니다.

| 단계 | 처리 내용 | 기준 / 수식 |
|------|----------|------|
| ① | 후보 blob 필터링 | `stats(i, CC_STAT_AREA) > 100` |
| ② | [1단계] 1차 최근접 탐색 | 직전 추적점 기준 `dist ≤ 150px` & 최소거리 |
| ③ | 추적점 1차 갱신 | `last_line_x_, last_line_y_ ← centroid` |
| ④ | [2단계] 재탐색 (면적 필터 X) | 갱신된 추적점 기준 전체 blob 중 최근접 |
| ⑤ | 유효성 판정 | `best ≤ 30.0` 이면 채택, 아니면 `idx = -1` |
| ⑥ | 오차 계산 | `error = (cols / 2.0) - last_line_x_` |
| ⑦ | 주행 모드 확인 | `atomic<bool> mode_` |

### 🔴 제어 (Control) — WSL2 내부
P 제어로 좌우 모터 속도를 계산해 Pi5로 퍼블리시합니다.

| 상태 | 좌측 모터 (x) | 우측 모터 (y) |
|------|--------------|--------------|
| 주행 (`mode_ = true`) | `base_vel_ − error × k_` | `−(base_vel_ + error × k_)` |
| 정지 (`mode_ = false`) | `0` | `0` |

- **P 게인 `k`** (기본 0.14) : 오차에 대한 반응 강도
- **기본 속도 `base_vel`** (기본 120) : 직진 속도
- **퍼블리시 토픽** : `topic_dxlpub` (`geometry_msgs/Vector3`, Reliable QoS)
- **로그** : `err / lvel / rvel / time(ms)` 매 프레임 출력

---

## 핵심 상수 / 임계값 요약

| 심볼 | 값 | 위치 | 의미 |
|------|------|------|------|
| ROI 비율 | 하단 1/4 | `setROI()` | 라인은 화면 하단에만 나타난다고 가정 |
| 목표 평균 밝기 | `100` | `setROI()` | 정규화 기준 |
| 이진화 임계값 | `150` | `setROI()` | 평균보다 +50 밝은 픽셀만 흰색 |
| 최소 면적 | `> 100` | `findLine()` 1단계 | 노이즈 blob 제거 |
| 1차 탐색 반경 | `≤ 150.0` | `findLine()` 1단계 | 프레임 간 움직임 허용 범위 |
| 2차 유효 반경 | `≤ 30.0` | `findLine()` 2단계 | 최종 라인 확정 기준 |
| 초기 추적점 | `(320, 45)` | 생성자 | 640×180 ROI 가정 중앙 |
| `k` 기본값 | `0.14` (선언) / `0.13` (리스트) | 파라미터 | P 제어 게인 |
| `base_vel` 기본값 | `120` (선언) / `150` (리스트) | 파라미터 | 직진 기본 속도 |

---

## 비동기 요소

| 요소 | 역할 | 구현 |
|------|------|------|
| `image_callback` | 인지→판단→제어 메인 루프 | `rclcpp::spin` 콜백 스레드 |
| `keyboardLoop` | `'s'`/`'q'` 키 입력 감지 | 별도 `std::thread` + `termios` raw 모드 + `select` 100ms |
| `mode_`, `running_` | 스레드 간 공유 상태 | `std::atomic<bool>` (lock-free) |
| `last_line_x_/y_` | 프레임 간 추적 상태 | 콜백 단일 스레드 내부 상태 |

---

## 주요 토픽 / QoS 요약

| 방향 | 토픽 | 타입 | QoS | 이유 |
|------|------|------|------|------|
| 구독 | `Image_Topic` | `sensor_msgs/CompressedImage` | **BestEffort** KeepLast(10) | 영상은 최신성이 중요, 손실 허용 |
| 퍼블리시 | `topic_dxlpub` | `geometry_msgs/Vector3` | **Reliable** KeepLast(10) | 모터 명령은 손실되면 안 됨 |
