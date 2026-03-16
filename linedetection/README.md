# ROS2 Line Detection

이 리포지토리는 ROS2 기반의 영상 처리 및 라인 트레이싱 알고리즘 테스트 패키지를 포함하고 있습니다. MP4 영상 파일에서 프레임을 읽어와 퍼블리시하고, 수신된 영상에서 OpenCV를 활용하여 라인을 검출하는 역할이 분리된 두 개의 로컬 패키지로 구성되어 있습니다.

## 주요 패키지 구성

### 1. `linedetect_nano` (Publisher)
- **노드명**: `video_publisher`
- **역할**: 지정된 경로의 MP4 영상 파일(`/home/aim88/simulation/7_lt_ccw_100rpm_in.mp4`)을 읽어와 프레임 단위로 ROS2 `Image` 메시지로 변환하여 퍼블리시합니다.
- **토픽**: `/video` (`sensor_msgs::msg::Image`)
- **실행 주기**: ~33Hz (30ms Timer 설정)

### 2. `linedetect_wsl` (Subscriber)
- **노드명**: `line_detector`
- **역할**: `/video` 토픽을 통해 수신된 영상을 OpenCV `cv_bridge`를 통해 `cv::Mat`으로 변환합니다. 이미지 처리(이진화 등) 및 Connected Components 분석을 수행하여 화면 하단 ROI에서 경로 라인을 안정적으로 추적합니다. 결과적으로 타겟 객체 위치와 화면 중심 간의 오차(error)를 터미널 정보로 디스플레이하며 OpenCV Window로 실시간 확인이 가능합니다.
- **토픽**: `/video` (`sensor_msgs::msg::Image`)
- **주요 파이프라인**:
  - `COLOR_BGR2GRAY` 활용 Grayscale 변환 및 전체 밝기 평균을 이용한 밝기 보정
  - `cv::threshold` 이진화 수행
  - 화면 하단 일부를(높이의 역 1/4) ROI(Region of Interest)로 자르기
  - `cv::connectedComponentsWithStats` 적용 및 면적 기반 1차 필터링
  - 첫 탐색과 추적 시점을 고려한 반경 기반 최소 거리 객체 트래킹 최적화

## 빌드 및 실행 방법

### 환경 정보
- **ROS2 Version**: Jazzy
- **Middleware**: CycloneDDS (또는 시스템의 기본 RMW)
- **Dependencies**: `rclcpp`, `sensor_msgs`, `std_msgs`, `cv_bridge`, `OpenCV`

### 빌드 절차
워크스페이스 폴더(`~/ros2_ws`)에서 빌드를 진행합니다.
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select linedetect_nano linedetect_wsl
```

### 실행 방법

**터미널 1 (영상 Publisher 실행)**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run linedetect_nano pub
```

**터미널 2 (Line 검출기 Subscriber 실행)**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run linedetect_wsl linedetect_wsl
```
