import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Low-level PX4 Offboard controller bridge
        Node(
            package='agentic_drone_control',
            executable='offboard_controller',
            name='px4_offboard_controller',
            output='screen',
            emulate_tty=True
        ),
        # 2. Cognitive Agent Node (connects to local Ollama API)
        Node(
            package='agentic_drone_control',
            executable='agent_node',
            name='agent_node',
            output='screen',
            emulate_tty=True
        )
    ])
