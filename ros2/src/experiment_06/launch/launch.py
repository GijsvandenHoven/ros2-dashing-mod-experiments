from launch import LaunchDescription
from launch_ros.actions import Node

# topic remappings, declared globally for renaming between experiments.
remap_A = [
    ("/simple_publishing", "/A_to_B")
]
remap_B = [
    ("/simple_passing_sub", "/A_to_B"),
    ("/simple_passing_pub", "/B_to_C")
]
remap_C = [
    ("/simple_passing_sub", "/B_to_C"),
    ("/simple_passing_pub", "/C_to_D")
]
remap_D = [
    ("/simple_publishing", "/C_to_D")
]

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="experiment_02",
            node_namespace="/",
            node_executable="simple_publisher",
            node_name="A",
            prefix=['stdbuf -o L'],
            output='screen',
            parameters=[
                {"activate_rate_ms_": 1000},
                {"launch_offset_time_ms_": 300}
            ],
            remappings=remap_A
        ),
        Node(
            package="experiment_06",
            node_namespace="/",
            node_executable="simple_passer",
            node_name="B",
            prefix=['stdbuf -o L'],
            output='screen',
            parameters=[
                {"busy_wait_in_callback_ms_": 300},
                {"launch_offset_time_ms_": 200}
            ],
            remappings=remap_B
        ),
        Node(
            package="experiment_06",
            node_namespace="/",
            node_executable="simple_passer",
            node_name="C",
            prefix=['stdbuf -o L'],
            output='screen',
            parameters=[
                {"busy_wait_in_callback_ms_": 300},
                {"launch_offset_time_ms_": 100}
            ],
            remappings=remap_C
        ),
        Node(
            package="experiment_02",
            node_namespace="/",
            node_executable="simple_subscriber",
            node_name="D",
            prefix=['stdbuf -o L'],
            output='screen',
            parameters=[
                {"busy_wait_in_callback_ms_": 0},
                {"launch_offset_time_ms_": 0}
            ],
            remappings=remap_D
        )
    ])