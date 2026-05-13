# py_launch_example

ROS 2 Python 패키지로, launch 파일을 사용해 여러 노드를 동시에 실행하는 예제입니다.  
Publisher/Subscriber 노드를 2개의 네임스페이스로 나누어 독립적으로 동작하는 구조를 보여줍니다.

## 패키지 구성

```
py_launch_example/
├── launch/
│   └── my_launch.py          # 4개 노드를 한 번에 실행하는 launch 파일
├── py_launch_example/
│   ├── pub.py                # Publisher 노드
│   └── sub.py                # Subscriber 노드
└── setup.py
```

## 노드 설명

### pub (Publisher)
- 1초마다 `hello_topic`에 `"Hello world!"` 문자열 메시지 발행
- 메시지 타입: `std_msgs/String`

### sub (Subscriber)
- `hello_topic`을 구독하여 수신한 메시지를 로그로 출력
- 메시지 타입: `std_msgs/String`

## 빌드

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select py_launch_example
source install/setup.bash
```

## 실행

### launch 파일로 전체 실행 (권장)

```bash
ros2 launch py_launch_example my_launch.py
```

launch 파일은 다음 4개 노드를 동시에 실행합니다:

| 네임스페이스 | 노드 | 역할 |
|---|---|---|
| `py_launch_example1` | `pub` | 메시지 발행 |
| `py_launch_example1` | `sub` | 메시지 수신 |
| `py_launch_example2` | `pub` | 메시지 발행 |
| `py_launch_example2` | `sub` | 메시지 수신 |

네임스페이스가 다르기 때문에 두 그룹은 서로 독립적으로 통신합니다.

### 노드 개별 실행

```bash
# 터미널 1
ros2 run py_launch_example pub

# 터미널 2
ros2 run py_launch_example sub
```

## 토픽 확인

```bash
# 실행 중인 토픽 목록
ros2 topic list

# 메시지 확인 (네임스페이스 포함)
ros2 topic echo /py_launch_example1/hello_topic
ros2 topic echo /py_launch_example2/hello_topic
```
