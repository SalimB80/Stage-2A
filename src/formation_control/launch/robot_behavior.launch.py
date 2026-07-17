from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    mode = LaunchConfiguration('mode')          # errance | cascade | dataset
    idx = LaunchConfiguration('robot_index')
    role = LaunchConfiguration('role')          # leader | tracker (mode cascade)
    target_color = LaunchConfiguration('target_color')
    desired_bearing = LaunchConfiguration('desired_bearing')
    target_distance = LaunchConfiguration('target_distance')
    record = LaunchConfiguration('record')

    # COUCHE 2 — COMPORTEMENT. Se branche sur un bringup DEJA actif.
    # Ne (re)demarre NI bringup NI camera. On peut le tuer/relancer pour
    # changer de mode sans toucher a la couche 1.
    #   errance  -> wander seul
    #   dataset  -> wander + recorder + rosbag
    #   cascade  -> tracker (si role=tracker ; leader = rien, juste pilote)

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

        # Errance / dataset -> wander
        Node(package='formation_control', executable='wander', name='wander',
             namespace=ns, condition=IfCondition(is_wander)),

        # Cascade tracker -> tracker
        Node(package='formation_control', executable='tracker', name='tracker',
             namespace=ns, condition=IfCondition(is_tracker),
             parameters=[{'target_color': target_color,
                          'desired_bearing': desired_bearing,
                          'target_distance': target_distance}]),

        # Dataset -> recorder SEUL (pas de rosbag). Le recorder ecrit deja les
        # frames JPEG + frames.csv / odom.csv / scan.csv (horodates, alignes a la
        # video). Le rosbag .db3 faisait doublon (scan/odom deja couverts, seul
        # l'IMU etait en plus) et n'est pas voulu -> supprime.
        Node(package='formation_control', executable='recorder', name='recorder',
             namespace=ns, condition=IfCondition(is_dataset),
             parameters=[{'robot_name': ns, 'segment_minutes': 5.0}]),
    ])
