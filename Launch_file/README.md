# ROS2 Launch 파일 모음

ROS2 실습과제용 런치 파일 디렉토리입니다.

---

## 파일 목록

| 파일명 | 실행 위치 | 실행되는 노드 |
|--------|-----------|---------------|
| `wsl2_launch.py` | WSL2 | camera_ros2/sub + dxl_wsl/pub |
| `rapi5_launch.py` | Raspberry Pi5 | camera_ros2/pub + dxl_rapi5/dxl_sub |
| `sim_launch.py` | WSL2 | linedetect_rapi5/pub + linedetect_wsl/sub |

---

## 실습과제1 — 카메라를 보면서 키보드로 로봇 조종

카메라 영상을 WSL2 화면에 표시하면서, 키보드로 실제 로봇(Pi5)의 모터를 제어하는 시스템입니다.

### 시스템 구성

```
[Pi5] camera_ros2/pub  →  image/compressed  →  [WSL2] camera_ros2/sub
[Pi5] dxl_rapi5/dxl_sub  ←  topic_dxlpub  ←  [WSL2] dxl_wsl/pub
```

### WSL2에서 실행 (wsl2_launch.py)

```bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/Launch_file
ros2 launch wsl2_launch.py
```

- **camera_ros2/sub**: Pi5에서 전송된 카메라 영상(`image/compressed`)을 수신하여 화면에 표시
- **dxl_wsl/pub**: 키보드 입력을 받아 모터 속도 명령(`topic_dxlpub`)을 퍼블리시
  - `f` = 전진, `b` = 후진, `l` = 좌회전, `r` = 우회전, `s` / `space` = 정지

### Pi5에서 실행 (rapi5_launch.py)

```bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/Launch_file
ros2 launch rapi5_launch.py
```

- **camera_ros2/pub**: libcamera로 카메라 영상을 캡처하여 `image/compressed` 토픽으로 퍼블리시
- **dxl_rapi5/dxl_sub**: WSL2에서 전송된 `topic_dxlpub`을 수신하여 Dynamixel 모터 구동

---

## 실습과제2 — 시뮬레이션 (노드 2개 동시 실행)

MP4 영상을 카메라 대신 사용하여 WSL2 단독으로 라인 감지를 테스트하는 시스템입니다.

### 시스템 구성

```
[WSL2] linedetect_rapi5/pub  →  Image_Topic  →  [WSL2] linedetect_wsl/sub
         (MP4 파일 재생)                              (라인 감지 + 화면 표시)
```

### WSL2에서 실행 (sim_launch.py)

```bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/Launch_file
ros2 launch sim_launch.py
```

- **linedetect_rapi5/pub**: MP4 영상 파일을 읽어 `Image_Topic`(CompressedImage)으로 퍼블리시
- **linedetect_wsl/sub**: `Image_Topic`을 구독하여 라인 감지 결과를 화면에 표시 (디버그용)

### 실행 후 확인 명령어

```bash
# 노드 목록 확인
ros2 node list

# 토픽 목록 확인
ros2 topic list

# 토픽 상세 확인
ros2 topic info /Image_Topic
```

---

## 환경 설정

```bash
# ROS2 환경 소스
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 멀티 머신 통신 시 필요 (Pi5 ↔ WSL2)
export ROS_DOMAIN_ID=<공유_ID>
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
