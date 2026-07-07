from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            GroupAction, ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='tortuga1'),

        GroupAction([
            PushRosNamespace(ns),

            # Bringup (moteurs, lidar, odometrie)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('turtlebot3_bringup'),
                    'launch', 'robot.launch.py'])),
            ),

            # Camera dediee a l'enregistrement
            Node(
                package='camera_ros',
                executable='camera_node',
                name='camera',
                parameters=[{'format': 'BGR888', 'width': 640, 'height': 480}],
                remappings=[('~/image_raw', 'camera/image_raw')],
            ),

            # Errance aleatoire (lidar seul)
            Node(
                package='formation_control',
                executable='wander',
                name='wander',
            ),

            # Enregistreur video local + CSV d'horodatage par frame
            Node(
                package='formation_control',
                executable='recorder',
                name='recorder',
                parameters=[{'robot_name': ns, 'segment_minutes': 5.0}],
            ),
        ]),

        # Rosbag : scan + odom + imu, horodates nativement par message.
        # Hors GroupAction (pas namespacable) -> topics complets construits
        # avec le namespace. Sortie : ~/dataset/bag_<date>_<robot>/
        ExecuteProcess(
            cmd=['bash', '-c',
                 ['mkdir -p ~/dataset && exec ros2 bag record '
                  '-o ~/dataset/bag_$(date +%Y%m%d_%H%M%S)_', ns,
                  ' /', ns, '/scan /', ns, '/odom /', ns, '/imu']],
            output='screen',
        ),
    ])
