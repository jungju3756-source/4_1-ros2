# dxl_nano

ros2 dxl test program on Raspberry Pi5

# 확인사항
- RaspberryPi5-ubuntu24.04와 Wsl2-ubuntu24.04가 같은 네트워크에 연결되어 있는지?
```bash
$ ifconfig #ip주소의 앞3자리가 같아야함
```
- RaspberryPi5-ubuntu24.04와 Wsl2-ubuntu24.04의 환경변수 ROS_DOMAIN_ID가 자기 로봇번호와 동일한지?
```bash
$ echo $ROS_DOMAIN_ID
```

# run dxl subscriber on Raspberry Pi5-ubuntu24.04
```bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/2sungryul/dxl_nano.git
$ cd ~/ros2_ws
$ colcon build --symlink-install --packages-select dxl_nano
$ source install/local_setup.bash
$ sudo chmod a+rw /dev/ttyUSB0
$ ros2 run dxl_nano sub
```

# run dxl publisher on WSL2-Ubuntu 20.04
```bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/2sungryul/dxl_wsl.git
$ cd ~/ros2_ws
$ colcon build --symlink-install --packages-select dxl_wsl
$ source install/local_setup.bash
$ ros2 run dxl_wsl pub
```
