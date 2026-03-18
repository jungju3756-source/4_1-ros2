# linetracer_real 실행 방법

## 시스템 구성

```
[Raspberry Pi5]                          [WSL2]
camera_ros2 (pub)                        linetracer_real
    │                                         │
    │──── image/compressed (토픽) ────────────►│
    │                                         │
    │◄─── topic_dxlpub (토픽) ────────────────│
    │
dxl_rapi5 (dxl_sub)
```

---

## 사전 준비

### Raspberry Pi5
```bash
# Dynamixel SDK 설치
sudo apt install ros-humble-dynamixel-sdk

# GStreamer libcamera 플러그인
sudo apt install gstreamer1.0-libcamera

# CycloneDDS 설치
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### WSL2
```bash
# OpenCV, cv_bridge
sudo apt install ros-humble-cv-bridge libopencv-dev

# CycloneDDS 설치
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

---

## 네트워크 설정 (Pi5 / WSL2 공통)

`~/.bashrc` 에 아래 내용 추가 후 `source ~/.bashrc`

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/$USER/cyclone_dds.xml
```

WSL2에서 `~/cyclone_dds.xml` 생성:

```xml
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="eth0"/>
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
```

---

## 빌드

### Raspberry Pi5
```bash
cd ~/ros2_ws
colcon build --packages-select camera_ros2 dxl_rapi5
source install/setup.bash
```

### WSL2
```bash
cd ~/ros2_ws
colcon build --packages-select linetracer_real
source install/setup.bash
```

---

## 실행

### Terminal 1 — Raspberry Pi5 (카메라 발행 + 다이나믹셀 제어)
```bash
ros2 launch dxl_rapi5 pi5_launch.py
```

### Terminal 2 — WSL2 (라인 검출 + 속도 명령 발행)
```bash
ros2 run linetracer_real linetracer_real
```

영상 화면이 뜨면:
- `s` 키 : 모터 속도 명령 전송 시작
- `q` 키 : 모터 정지

---

## 파라미터 (dxl_rapi5)

`dxl_rapi5/launch/pi5_launch.py` 에서 수정

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `device_name` | `/dev/ttyUSB0` | 시리얼 포트 |
| `baudrate` | `57600` | 통신 속도 |
| `left_id` | `1` | 좌측 모터 ID |
| `right_id` | `2` | 우측 모터 ID |

> 오른쪽 모터가 반대 방향 장착된 경우 `dxl_sub.cpp` 88번째 줄 `right_val` 부호를 반전하세요.

---

## 통신 연결 확인

Pi5에서:
```bash
ros2 topic pub /test std_msgs/msg/String "{data: 'hello'}"
```

WSL2에서:
```bash
ros2 topic echo /test
```

`hello` 메시지가 수신되면 정상입니다.
