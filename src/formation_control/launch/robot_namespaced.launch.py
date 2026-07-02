from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            GroupAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='tortuga2'),
        GroupAction([
            PushRosNamespace(ns),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('turtlebot3_bringup'),
                    'launch', 'robot.launch.py'])),
            ),
            Node(
                package='camera_ros',
                executable='camera_node',
                name='camera',
                parameters=[{'format': 'BGR888',
                             'width': 640, 'height': 480}],
                output='screen',
            ),
        ]),
    ])
