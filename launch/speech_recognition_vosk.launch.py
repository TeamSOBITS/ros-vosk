from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'lang',
            default_value='ja',
            description='Language setting for the VOSK recognizer'
        ),
        DeclareLaunchArgument(
            'recognition_mode',
            default_value='service',
            description='Recognition mode for the VOSK recognizer'
        ),
        
        Node(
            package='speech_recognition_vosk',
            executable='vosk_node',
            name='vosk_node',
            output='screen',
            parameters=[{'recognition_mode': LaunchConfiguration('recognition_mode')}]
        )
    ])

