# camera_ros2

# 확인사항
- RaspberryPi5-ubuntu24.04와 Wsl2-ubuntu24.04가 같은 네트워크에 연결되어 있는지?
```bash
$ ifconfig #ip주소의 앞3자리가 같아야함
```
- RaspberryPi5-ubuntu24.04와 Wsl2-ubuntu24.04의 환경변수 ROS_DOMAIN_ID가 자기 로봇번호와 동일한지?
```bash
$ echo $ROS_DOMAIN_ID
```
- ubuntu24.04에 cv_bridge 패키지를 설치
```bash
$ sudo apt update
$ sudo apt install ros-jazzy-cv-bridge
```
- ubuntu24.04에 C++ opencv 패키지를 설치
```bash
$ sudo apt update
$ sudo apt install libopencv-dev
```
# RaspberryPi5-ubuntu24.04에서 패키지 설치 및 실행

```bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/2sungryul/camera_ros2.git
$ ls # camera_ros2 생성확인
$ cd ~/ros2_ws
$ colcon build --symlink-install --packages-select camera_ros2
$ source install/local_setup.bash
$ ros2 run camera_ros2 pub
```
# Wsl2-ubuntu24.04에서 패키지 설치 및 실행

```bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/2sungryul/camera_ros2.git
$ ls # camera_ros2 생성확인
$ cd ~/ros2_ws
$ colcon build --symlink-install --packages-select camera_ros2
$ source install/local_setup.bash
$ ros2 run camera_ros2 sub
```

