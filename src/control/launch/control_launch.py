from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control',
            name='control',
            executable='control',
            emulate_tty=True,
            output="screen"
        )
    ])