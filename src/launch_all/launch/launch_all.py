from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Launch file to include control and ros_tcp_endpoint launches
def generate_launch_description():
    control_launch = PathJoinSubstitution([FindPackageShare('control'), 'launch', 'control_launch.py'])
    endpoint_launch = PathJoinSubstitution([FindPackageShare('ros_tcp_endpoint'), 'launch', 'endpoint_launch.py'])
    return LaunchDescription([
        IncludeLaunchDescription([control_launch]),
        IncludeLaunchDescription([endpoint_launch]),
    ])
