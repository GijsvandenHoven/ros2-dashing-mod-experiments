from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="experiment_02",
            node_namespace="/",
            node_executable="simple_publisher",
            node_name="pub_1",
            prefix=['stdbuf -o L'],
            output='screen',
            parameters=[
                {"msg_size_bytes_": 0},
                {"launch_offset_time_ms_": 0},
                {"activate_rate_ms_": 500}
            ]
        ),
        Node(
            package="experiment_02",
            node_namespace="/",
            node_executable="simple_publisher",
            node_name="pub_2",
            prefix=['stdbuf -o L'],
            output='screen',
            parameters=[
                {"msg_size_bytes_": 65536},
                {"launch_offset_time_ms_": 167},
                {"activate_rate_ms_": 500}
            ]
        ),
        Node(
            package="experiment_02",
            node_namespace="/",
            node_executable="simple_publisher",
            node_name="pub_3",
            prefix=['stdbuf -o L'],
            output='screen',
            parameters=[
                {"msg_size_bytes_": 4194304},
                {"launch_offset_time_ms_": 333},
                {"activate_rate_ms_": 500}
            ]
        ),
        Node(
            package="experiment_02",
            node_namespace="/",
            node_executable="simple_subscriber",
            node_name="sub_1",
            prefix=['stdbuf -o L'],
            output='screen',
            parameters=[
                {"busy_wait_in_callback_ms_": 0}
            ]
        )
    ])