from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_launch_example',
            namespace='py_launch_example1',
            executable='pub',
            name='pub'
        ),
        Node(
            package='py_launch_example',
            namespace='py_launch_example1',
            executable='sub',
            name='sub'
        ),
        Node(
            package='py_launch_example',
            namespace='py_launch_example2',
            executable='pub',
            name='pub'
        ),
        Node(
            package='py_launch_example',
            namespace='py_launch_example2',
            executable='sub',
            name='sub'
        ),
    ])
