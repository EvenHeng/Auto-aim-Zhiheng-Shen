from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='student_auto_aim',
            executable='aim_node',
            name='aim_node',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'control_rate': 50.0},
                {'extra_latency': 0.02},
            ],
        )
    ])
