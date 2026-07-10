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
                # Dataset. IMPORTANT sur IMX219 : le mode 640x480 haute-frequence
                # est un CROP central (zoom, champ etroit). Pour un champ LARGE
                # et HOMOGENE sur tous les robots (necessaire a la detection), on
                # FIXE sensor_mode=1920:1080 (recadrage 16:9 modere) -> ~47 fps.
                # Fixer sensor_mode ecrase aussi tout crop "bidouille" sur un
                # robot. FrameDurationLimits en us/image : 22000 = ~45 fps, marge
                # sure au-dessus du plancher du mode (une valeur trop basse est
                # REJETEE -> l'auto-expo reprend et retombe a ~16 fps). La borne
                # force l'auto-expo a garder une pose courte (compense en gain).
                # Le recorder mesure et affiche le FPS reel.
                parameters=[{'format': 'BGR888', 'width': 640, 'height': 480,
                             'sensor_mode': '1920:1080',
                             'FrameDurationLimits': [22000, 22000]}],
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

