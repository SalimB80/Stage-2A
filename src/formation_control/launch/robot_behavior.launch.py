from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    mode = LaunchConfiguration('mode')          # errance | cascade | dataset
    idx = LaunchConfiguration('robot_index')
    role = LaunchConfiguration('role')          # leader | tracker (cascade mode)
    target_color = LaunchConfiguration('target_color')
    desired_bearing = LaunchConfiguration('desired_bearing')
    target_distance = LaunchConfiguration('target_distance')
    record = LaunchConfiguration('record')

    # LAYER 2 — BEHAVIOUR. Plugs into an ALREADY RUNNING bringup.
    # It (re)starts NEITHER bringup NOR camera. It can be killed/relaunched to
    # switch mode without touching layer 1.
    #   errance  -> wander alone
    #   dataset  -> wander + recorder + rosbag
    #   cascade  -> tracker (when role=tracker; the leader runs nothing, it is
    #               simply driven by hand)

    is_wander = PythonExpression(
        ["'", mode, "' == 'errance' or '", mode, "' == 'dataset'"])
    is_tracker = PythonExpression(
        ["'", mode, "' == 'cascade' and '", role, "' == 'tracker'"])
    is_dataset = PythonExpression(["'", mode, "' == 'dataset'"])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='tortuga1'),
        DeclareLaunchArgument('mode', default_value='errance'),
        DeclareLaunchArgument('robot_index', default_value='1'),
        DeclareLaunchArgument('role', default_value='tracker'),
        DeclareLaunchArgument('target_color', default_value='jaune'),
        DeclareLaunchArgument('desired_bearing', default_value='0.0'),
        DeclareLaunchArgument('target_distance', default_value='0.6'),
        DeclareLaunchArgument('record', default_value='false'),

        # Wander / dataset -> wander
        Node(package='formation_control', executable='wander', name='wander',
             namespace=ns, condition=IfCondition(is_wander)),

        # Cascade tracker -> tracker
        Node(package='formation_control', executable='tracker', name='tracker',
             namespace=ns, condition=IfCondition(is_tracker),
             parameters=[{'target_color': target_color,
                          'desired_bearing': desired_bearing,
                          'target_distance': target_distance}]),

        # Dataset -> recorder ONLY (no rosbag). The recorder already writes the
        # JPEG frames + frames.csv / odom.csv / scan.csv (timestamped, aligned
        # with the video). The .db3 rosbag was redundant (scan/odom already
        # covered, only the IMU was extra) and is not wanted -> removed.
        Node(package='formation_control', executable='recorder', name='recorder',
             namespace=ns, condition=IfCondition(is_dataset),
             parameters=[{'robot_name': ns, 'segment_minutes': 5.0}]),
    ])
