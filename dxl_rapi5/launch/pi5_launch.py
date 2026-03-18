from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros2',
            executable='pub',
            name='campub',
            output='screen'
        ),
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
