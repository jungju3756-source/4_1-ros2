from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # linedetect_rapi5 패키지의 pub 노드: MP4 영상 파일을 읽어 Image_Topic(CompressedImage)으로 퍼블리시
        Node(
            package='linedetect_rapi5',
            executable='pub',
            name='image_publisher',
            output='screen'
        ),
        # linedetect_wsl 패키지의 sub 노드: Image_Topic을 구독하여 라인 감지 결과를 화면에 표시(디버그용)
        Node(
            package='linedetect_wsl',
            executable='sub',
            name='line_detector',
            output='screen'
        ),
    ])
