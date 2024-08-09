from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description(){
    return LaunchDescription([
        DeclareLaunchArgument(
            'lang',
            default_value = 'ja'
        ),
        DeclareLaunchArgument([
            'trigger_word',
            default_value = 'stop Stop step staff top',
            description = 'トリガーになってほしい単語のリストを、上のDefaultのクォーテーション内に単語ごとに空白をあけて記入'
        ])

        Node(
            package='speech_recognition_vosk',
            executable='vosk_node',
            name='vosk_engine',
            output='screen',
            parameters='recognition_mode': LaunchConfiguration('trigger')
        ),
        Node(
            package='speech_recognition_vosk',
            executable='trigger',
            name='voice_trigger',
            output='screen',
            parameters='trigger_word': LaunchConfiguration('trigger_word')
        )

    ])
}