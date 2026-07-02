from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_index', default_value='2'),
        DeclareLaunchArgument('formation', default_value='colonne'),
        DeclareLaunchArgument('namespace', default_value='tortuga2'),
        Node(
            package='formation_control',
            executable='follower',
            name='follower',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'robot_index': LaunchConfiguration('robot_index'),
                'formation': LaunchConfiguration('formation'),
            }],
        ),
    ])
