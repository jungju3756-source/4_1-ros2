# linetracer_real

실제 로봇(Raspberry Pi5) 환경에서 카메라 영상을 수신해 라인을 감지하고, P 제어를 통해 Dynamixel 모터에 속도 명령을 퍼블리시하는 ROS 2 패키지입니다.

## 개요

`linetracer_sim`의 실제 로봇용 버전입니다. 시뮬레이션과 달리 Pi5 카메라에서 압축된 이미지를 수신하며, 더 높은 기본 속도(`base_vel` 기본값: 120)로 동작합니다.

## 시스템 구성

```
[Pi5: camera_ros2/pub]
  └─ image/compressed (CompressedImage) 퍼블리시
        │
        └──→ [WSL2: linetracer_real]  ← 이 패키지
                라인 감지 + P 제어
                퍼블리시: topic_dxlpub (Vector3)
                키보드: 's'=주행 시작, 'q'=정지
                        │
                        ▼
              [Pi5: dxl_rapi5]
                Dynamixel 모터 구동
```

## 토픽

| 방향 | 토픽명 | 메시지 타입 | QoS | 설명 |
|------|--------|------------|-----|------|
| Subscribe | `Image_Topic` | `sensor_msgs/CompressedImage` | BestEffort | Pi5 카메라 영상 수신 |
| Publish | `topic_dxlpub` | `geometry_msgs/Vector3` | Reliable | 좌우 모터 속도 명령 |

- `Vector3.x` : 좌측 모터 속도
- `Vector3.y` : 우측 모터 속도 (부호 반전)

## 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `k` | `0.14` | P 제어 비례 게인 |
| `base_vel` | `120` | 기본 전진 속도 |

런타임 중 파라미터 변경:

```bash
ros2 param set /line_tracker_node k 0.15
ros2 param set /line_tracker_node base_vel 100
```

## 이미지 처리 파이프라인

```
CompressedImage 수신
        │
        ▼
  하단 1/4 ROI 추출
        │
        ▼
  그레이스케일 변환
        │
        ▼
  밝기 정규화 (평균 → 100)
        │
        ▼
  이진화 (임계값: 150)
        │
        ▼
  Connected Component 분석
        │
        ▼
  직전 추적 위치 기준 최근접 Blob 선택
  (거리 150px 이내 후보 → 30px 이내 확정)
        │
        ▼
  오차 계산: error = (frame_width / 2) - centroid_x
        │
        ▼
  P 제어 출력
  left_vel  = base_vel - error × k
  right_vel = -(base_vel + error × k)
```

## 빌드 및 실행

```bash
# 빌드
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select linetracer_real

# 실행
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=<shared_id>
ros2 run linetracer_real linetracer_real
```

실행 후 터미널에서:
- `s` : 주행 시작
- `q` : 주행 정지

## 시각화

노드 실행 시 두 개의 OpenCV 창이 표시됩니다.

| 창 이름 | 내용 |
|---------|------|
| `1. Raw Video` | 수신된 원본 프레임 |
| `2. Binary Debug` | 이진화된 ROI + 감지된 라인(빨간색) + 노이즈 Blob(파란색) + 추적 지점(빨간 점) |

## 의존성

- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `cv_bridge`
- `OpenCV`

## linetracer_sim과의 차이점

| 항목 | linetracer_sim | linetracer_real |
|------|---------------|-----------------|
| 대상 환경 | WSL2 시뮬레이션 | 실제 로봇 (Pi5) |
| 기본 속도 (`base_vel`) | 50 | 120 |
| 기본 게인 (`k`) | 0.13 | 0.14 |
| 퍼블리시 토픽 | `vel_cmd_topic` | `topic_dxlpub` |
