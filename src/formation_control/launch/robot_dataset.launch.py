from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            GroupAction, ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    record = LaunchConfiguration('record')
    slam = LaunchConfiguration('slam')

    # record : video + rosbag (scan/odom/imu).
    # slam   : lance slam_toolbox pour cartographier la piece (map a la
    #          demande). Independant de record. Namespace pose 1 seule fois.

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='tortuga1'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false'),

        GroupAction([
            PushRosNamespace(ns),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('turtlebot3_bringup'),
                    'launch', 'robot.launch.py'])),
            ),

            Node(
                package='camera_ros', executable='camera_node', name='camera',
                parameters=[{'format': 'BGR888', 'width': 640, 'height': 480,
                             'FrameDurationLimits': [33333, 33333]}],
                remappings=[('~/image_raw', 'camera/image_raw')],
            ),

            Node(package='formation_control', executable='wander',
                 name='wander'),

            Node(package='formation_control', executable='recorder',
                 name='recorder', condition=IfCondition(record),
                 parameters=[{'robot_name': ns, 'segment_minutes': 5.0}]),

            # SLAM a la demande (cartographie) — slam_toolbox en mode async
            Node(
                package='slam_toolbox', executable='async_slam_toolbox_node',
                name='slam_toolbox', condition=IfCondition(slam),
                parameters=[{'use_sim_time': False,
                             'odom_frame': 'odom', 'base_frame': 'base_footprint',
                             'scan_topic': 'scan', 'mode': 'mapping'}],
            ),
        ]),

        ExecuteProcess(
            condition=IfCondition(record),
            cmd=['bash', '-c',
                 ['mkdir -p ~/dataset && exec ros2 bag record '
                  '-o ~/dataset/bag_$(date +%Y%m%d_%H%M%S)_', ns,
                  ' /', ns, '/scan /', ns, '/odom /', ns, '/imu']],
            output='screen',
        ),
    ])

