from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration('namespace')

    # LAYER 1 — BRINGUP. Starts all the hardware: motors, lidar, camera.
    # It stays up permanently; the behaviours (wander/cascade/dataset) are
    # launched SEPARATELY on top of it, without shutting this down.
    # The TurtleBot3 bringup already pushes the namespace -> we pass it as an
    # argument instead of pushing it again. Camera uses namespace=ns.

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
            # 640x480 @ 55 fps: 18181 us/frame (see robot_dataset.launch.py).
            parameters=[{'format': 'BGR888', 'width': 640, 'height': 480,
                         'FrameDurationLimits': [18181, 18181]}],
            remappings=[('~/image_raw', 'camera/image_raw')],
        ),
    ])
