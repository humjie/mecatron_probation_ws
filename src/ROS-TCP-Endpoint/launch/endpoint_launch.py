from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros_tcp_endpoint",
                executable="default_server_endpoint",
                # add this to assign new name (by this we can run multiple instance also if needed)
                name="tcp_endpoint",
                emulate_tty=True,
                parameters=[{"ROS_IP": "0.0.0.0"}, {"ROS_TCP_PORT": 10000}],
                # print the output to the terminal
                output="screen"
            ),
            # launch foxglove bridge
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                emulate_tty=True,
                output="screen"
            ),
            # wait 3 seconds and call the service to change mode to GUIDED
            TimerAction(
                period=3.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "service",
                            "call",
                            "/mavros/set_mode",
                            "mavros_msgs/srv/SetMode",
                            "{base_mode: 0, custom_mode: 'GUIDED'}"
                        ],
                        output="screen"
                    )
                ]
            )
        ]
    )
