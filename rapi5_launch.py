from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # camera_ros2 패키지의 pub 노드: libcamera로 카메라 영상을 캡처하여 image/compressed 토픽으로 퍼블리시
        Node(
            package='camera_ros2',
            executable='pub',
            name='camera_pub',
            output='screen'
        ),
        # dxl_rapi5 패키지의 dxl_sub 노드: WSL2에서 전송된 topic_dxlpub을 수신하여 Dynamixel 모터 구동
        Node(
            package='dxl_rapi5',
            executable='dxl_sub',
            name='dxl_sub',
            output='screen',
            parameters=[{
                'device_name': '/dev/ttyUSB0',
                'baudrate':    57600,
                'left_id':     1,
                'right_id':    2,
            }]
        ),
    ])
