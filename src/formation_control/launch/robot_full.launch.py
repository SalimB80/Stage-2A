from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    idx = LaunchConfiguration('robot_index')
    formation = LaunchConfiguration('formation')
    role = LaunchConfiguration('role')   # 'leader' | 'follower' | 'tracker'

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='tortuga1'),
        DeclareLaunchArgument('robot_index', default_value='1'),
        DeclareLaunchArgument('formation', default_value='colonne'),
        DeclareLaunchArgument('role', default_value='follower'),

        GroupAction([
            PushRosNamespace(ns),

            # Bringup TurtleBot3 (moteurs, lidar, odometrie)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('turtlebot3_bringup'),
                    'launch', 'robot.launch.py'])),
            ),

            # Camera (camera_ros) - capteur CSI, BGR888 640x480
            Node(
                package='camera_ros',
                executable='camera_node',
                name='camera',
                parameters=[{
                    'format': 'BGR888',
                    'width': 640,
                    'height': 480,
                }],
                remappings=[('~/image_raw', 'camera/image_raw')],
            ),

            # FOLLOWER : suit un leader avec offset de formation (role == follower)
            Node(
                package='formation_control',
                executable='follower',
                name='follower',
                condition=IfCondition(
                    PythonExpression(["'", role, "' == 'follower'"])),
                parameters=[{'robot_index': idx, 'formation': formation}],
            ),

            # TRACKER : autonome, cherche puis suit la couleur (role == tracker)
            Node(
                package='formation_control',
                executable='tracker',
                name='tracker',
                condition=IfCondition(
                    PythonExpression(["'", role, "' == 'tracker'"])),
            ),
        ]),
    ])
