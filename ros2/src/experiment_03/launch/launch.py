from launch import LaunchDescription
from launch_ros.actions import Node

#running this with just 1 core shows a very crazy graph where system interrupts directly influence latency.

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
            node_executable="simple_subscriber",
            node_name="sub_1",
            prefix=['stdbuf -o L'],
            output='screen',
            parameters=[
                {"busy_wait_in_callback_ms_": 100}
            ]
        ),
        Node(
            package="experiment_02",
            node_namespace="/",
            node_executable="simple_subscriber",
            node_name="sub_2",
            prefix=['stdbuf -o L'],
            output='screen',
            parameters=[
                {"busy_wait_in_callback_ms_": 100}
            ]
        ),
        Node(
            package="experiment_02",
            node_namespace="/",
            node_executable="simple_subscriber",
            node_name="sub_3",
            prefix=['stdbuf -o L'],
            output='screen',
            parameters=[
                {"busy_wait_in_callback_ms_": 100}
            ]
        )
    ])