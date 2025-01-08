from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='speech_recognition_vosk',
            executable='vosk_node',
            name='vosk_node',
            output='screen',
            parameters=[
                {
                    'sample_rate': 44100,
                    'blocksize': 16000,
                    'model': os.path.join(get_package_share_directory("speech_recognition_vosk"), 'models', 'vosk-model-small-en-us-0.15'),  # Lightweight wideband model for US English
                    # 'model': os.path.join(get_package_share_directory("speech_recognition_vosk"), 'models', 'vosk-model-en-us-0.22'),  # Big model for US English
                    # 'model': os.path.join(get_package_share_directory("speech_recognition_vosk"), 'models', 'vosk-model-small-ja-0.22'), # Lightweight wideband model for Japanese
                    # 'model': os.path.join(get_package_share_directory("speech_recognition_vosk"), 'models', 'vosk-model-ja-0.22'), # Big model for Japanese
                    # 'model': os.path.join(get_package_share_directory("speech_recognition_vosk"), 'models', 'vosk-model-spk-0.4'), # Model for speaker language identification
                },
            ],
        )
    ])

