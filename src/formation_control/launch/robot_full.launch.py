from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    idx = LaunchConfiguration('robot_index')
    formation = LaunchConfiguration('formation')
    role = LaunchConfiguration('role')
    target_color = LaunchConfiguration('target_color')
    desired_bearing = LaunchConfiguration('desired_bearing')
    target_distance = LaunchConfiguration('target_distance')

    # IMPORTANT : plus de PushRosNamespace ici.
    # Le namespace est applique UNE SEULE FOIS, via l'attribut namespace=
    # de chaque Node et via l'argument du bringup. PushRosNamespace + le
    # namespace du bringup se cumulaient en /tortugaX/tortugaX/... .

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='tortuga1'),
        DeclareLaunchArgument('robot_index', default_value='1'),
        DeclareLaunchArgument('formation', default_value='colonne'),
        DeclareLaunchArgument('role', default_value='follower'),
        DeclareLaunchArgument('target_color', default_value='jaune'),
        DeclareLaunchArgument('desired_bearing', default_value='0.0'),
        DeclareLaunchArgument('target_distance', default_value='0.6'),

        # Bringup TurtleBot3 : on lui passe le namespace directement.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('turtlebot3_bringup'),
                'launch', 'robot.launch.py'])),
            launch_arguments={'namespace': ns}.items(),
        ),

        # Camera : namespace pose une seule fois via l'attribut namespace=
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            namespace=ns,
            parameters=[{'format': 'BGR888', 'width': 640, 'height': 480}],
            remappings=[('~/image_raw', 'camera/image_raw')],
        ),

        # FOLLOWER
        Node(
            package='formation_control',
            executable='follower',
            name='follower',
            namespace=ns,
            condition=IfCondition(
                PythonExpression(["'", role, "' == 'follower'"])),
            parameters=[{'robot_index': idx, 'formation': formation}],
        ),

        # TRACKER
        Node(
            package='formation_control',
            executable='tracker',
            name='tracker',
            namespace=ns,
            condition=IfCondition(
                PythonExpression(["'", role, "' == 'tracker'"])),
            parameters=[{
                'target_color': target_color,
                'desired_bearing': desired_bearing,
                'target_distance': target_distance,
            }],
        ),
    ])
