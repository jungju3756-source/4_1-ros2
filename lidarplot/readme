# 🛰️ lidarplot: ROS2 LiDAR Visualization Package

이 프로젝트는 ROS2 Humble 환경에서 LiDAR 센서 데이터를 수신하고, 이를 실시간으로 분석 및 시각화하기 위한 패키지입니다. `sensor_msgs/msg/LaserScan` 데이터를 처리하여 터미널 출력 및 그래픽 시각화를 제공합니다.

---

## 🛠️ 개발 환경
- **OS**: Ubuntu 22.04 LTS
- **ROS 버전**: ROS2 Humble
- **언어**: Python 3.10+
- **주요 라이브러리**: 
  - `rclpy` (ROS2 Client Library)
  - `sensor_msgs` (Standard LiDAR messages)
  - `matplotlib` (Data Visualization)

---

## 📂 패키지 구조
```text
lidarplot/
├── lidarplot/
│   ├── __init__.py
│   ├── lidar_sub.py         # LiDAR 데이터 구독 및 로직 처리 노드
│   └── lidar_plot.py        # Matplotlib 기반 실시간 시각화 노드
├── resource/
├── test/
├── package.xml              # 의존성 및 패키지 정보
├── setup.cfg
├── setup.py                 # 엔트리 포인트 및 설치 설정
└── README.md

시각화 기능을 위해 matplotlib가 설치되어 있어야 합니다.
sudo apt update
sudo apt install python3-matplotlib

빌드 
# 워크스페이스 이동
cd ~/colcon_ws

# 빌드 진행
colcon build --packages-select lidarplot

# 환경 설정 (새 터미널마다 실행)
source install/setup.bash

LiDAR 데이터 모니터링 (Subscriber)
ros2 run lidarplot lidar_sub

실시간 그래프 시각화
ros2 run lidarplot lidar_plot
