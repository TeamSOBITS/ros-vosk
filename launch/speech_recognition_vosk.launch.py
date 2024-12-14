from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='speech_recognition_vosk',
            executable='vosk_node',
            name='vosk_node',
            output='screen',
        )
    ])

