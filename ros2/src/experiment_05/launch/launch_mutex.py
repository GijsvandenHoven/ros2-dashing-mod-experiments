from launch import LaunchDescription
from launch_ros.actions import Node

# topic remappings, declared globally for renaming between experiments.
the_remap = [
    ("/simple_publishing", "/multi_timer")
]

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="experiment_05",
            node_namespace="/",
            node_executable="multi_timer_publisher_mutex",
            node_name="pub_1",
            prefix=['stdbuf -o L'],
            output='screen',
            parameters=[
                {"msg_size_bytes_": 0},
                {"launch_offset_time_ms_": 0},
                {"activate_rate_ms_": 100},
                {"busy_wait_in_callback_ms_": 30}
            ],
            remappings=the_remap
        ),
        Node(
            package="experiment_02",
            node_namespace="/",
            node_executable="simple_subscriber",
            node_name="sub_1",
            prefix=['stdbuf -o L'],
            output='screen',
            remappings=the_remap
        )
    ])
