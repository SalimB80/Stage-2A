from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration('namespace')

    # COUCHE 1 — BRINGUP. Demarre tout le materiel : moteurs, lidar, camera.
    # Reste allume en permanence ; les comportements (errance/cascade/dataset)
    # se lancent SEPAREMENT par-dessus, sans couper ceci.
    # Le bringup TurtleBot3 pousse deja le namespace -> on lui passe en
    # argument, on ne le re-pousse pas. Camera avec namespace=ns.

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='tortuga1'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('turtlebot3_bringup'),
                'launch', 'robot.launch.py'])),
            launch_arguments={'namespace': ns}.items(),
        ),

        Node(
            package='camera_ros', executable='camera_node', name='camera',
            namespace=ns,
            parameters=[{'format': 'BGR888', 'width': 640, 'height': 480,
                         'FrameDurationLimits': [33333, 33333]}],
            remappings=[('~/image_raw', 'camera/image_raw')],
        ),
    ])
