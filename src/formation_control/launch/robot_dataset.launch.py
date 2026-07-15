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
    #          demande). Independant de record.
    #
    # NAMESPACE : le bringup TurtleBot3 applique DEJA le namespace via son
    # propre argument (comme le GUI qui l'appelle avec namespace:=tortugaX).
    # Il ne faut donc PAS l'envelopper en plus dans PushRosNamespace, sinon le
    # namespace est applique DEUX fois -> /tortugaX/tortugaX/scan (double NS) :
    # wander/rosbag ecoutent /tortugaX/scan et ne recoivent alors rien.
    # -> On passe namespace:=ns a l'include (une seule fois), et on met
    #    PushRosNamespace UNIQUEMENT autour de NOS noeuds.

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='tortuga1'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false'),

        # Bringup TurtleBot3 : namespace applique par SON argument (1 seule fois)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('turtlebot3_bringup'),
                'launch', 'robot.launch.py'])),
            launch_arguments={'namespace': ns}.items(),
        ),

        # Nos noeuds a nous : un seul PushRosNamespace.
        GroupAction([
            PushRosNamespace(ns),

            Node(
                package='camera_ros', executable='camera_node', name='camera',
                # Dataset : 640x480 @ 55 fps. 18181 us/image = 55 fps (au-dessus
                # du plancher hardware 16971, donc valide). Le recorder ecrit les
                # JPEG NATIFS (image_raw/compressed) sans re-encoder -> le Pi
                # tient 55 fps la ou un re-encodage video plafonnait a ~30.
                # La borne force l'auto-expo a une pose courte (compense en gain).
                parameters=[{'format': 'BGR888', 'width': 640, 'height': 480,
                             'FrameDurationLimits': [18181, 18181]}],
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
