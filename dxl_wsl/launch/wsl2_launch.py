from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # camera_ros2 패키지의 sub 노드: Pi5에서 전송된 image/compressed 토픽을 수신하여 화면에 표시
        Node(
            package='camera_ros2',
            executable='sub',
            name='camera_sub',
            output='screen'
        ),
        # dxl_wsl 패키지의 pub 노드: 키보드 입력(f/b/l/r/s)을 받아 topic_dxlpub으로 모터 속도 퍼블리시
        Node(
            package='dxl_wsl',
            executable='pub',
            name='dxl_pub',
            output='screen'
        ),
    ])
