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

    # IMPORTANT: no more PushRosNamespace here.
    # The namespace is applied ONLY ONCE, through each Node's namespace=
    # attribute and through the bringup argument. PushRosNamespace plus the
    # bringup namespace used to stack into /tortugaX/tortugaX/... .

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='tortuga1'),
        DeclareLaunchArgument('robot_index', default_value='1'),
        DeclareLaunchArgument('formation', default_value='colonne'),
        DeclareLaunchArgument('role', default_value='follower'),
        DeclareLaunchArgument('target_color', default_value='jaune'),
        DeclareLaunchArgument('desired_bearing', default_value='0.0'),
        DeclareLaunchArgument('target_distance', default_value='0.6'),

        # TurtleBot3 bringup: the namespace is passed to it directly.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('turtlebot3_bringup'),
                'launch', 'robot.launch.py'])),
            launch_arguments={'namespace': ns}.items(),
        ),

        # Camera: namespace applied only once, via the namespace= attribute
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
