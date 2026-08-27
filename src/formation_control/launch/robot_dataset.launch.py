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
    # slam   : starts slam_toolbox to map the room (map on demand).
    #          Independent of record.
    #
    # NAMESPACE: the TurtleBot3 bringup ALREADY applies the namespace through
    # its own argument (as the GUI does when calling it with namespace:=tortugaX).
    # So it must NOT be wrapped in PushRosNamespace on top of that, otherwise the
    # namespace is applied TWICE -> /tortugaX/tortugaX/scan (double NS): wander
    # and the rosbag listen on /tortugaX/scan and then receive nothing.
    # -> We pass namespace:=ns to the include (once), and use PushRosNamespace
    #    ONLY around OUR OWN nodes.

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='tortuga1'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false'),

        # TurtleBot3 bringup: namespace applied by ITS argument (only once)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('turtlebot3_bringup'),
                'launch', 'robot.launch.py'])),
            launch_arguments={'namespace': ns}.items(),
        ),

        # Our own nodes: a single PushRosNamespace.
        GroupAction([
            PushRosNamespace(ns),

            Node(
                package='camera_ros', executable='camera_node', name='camera',
                # Dataset: 640x480 @ 55 fps. 18181 us/frame = 55 fps (above the
                # 16971 hardware floor, so valid). The recorder writes the NATIVE
                # JPEGs (image_raw/compressed) without re-encoding -> the Pi
                # sustains 55 fps where video re-encoding capped out at ~30.
                # The limit forces auto-exposure to a short exposure time
                # (compensated by gain).
                parameters=[{'format': 'BGR888', 'width': 640, 'height': 480,
                             'FrameDurationLimits': [18181, 18181]}],
                remappings=[('~/image_raw', 'camera/image_raw')],
            ),

            Node(package='formation_control', executable='wander',
                 name='wander'),

            Node(package='formation_control', executable='recorder',
                 name='recorder', condition=IfCondition(record),
                 parameters=[{'robot_name': ns, 'segment_minutes': 5.0}]),

            # SLAM on demand (mapping) — slam_toolbox in async mode
            Node(
                package='slam_toolbox', executable='async_slam_toolbox_node',
                name='slam_toolbox', condition=IfCondition(slam),
                parameters=[{'use_sim_time': False,
                             'odom_frame': 'odom', 'base_frame': 'base_footprint',
                             'scan_topic': 'scan', 'mode': 'mapping'}],
            ),
        ]),
        # No rosbag: the recorder writes camera JPEGs + scan.csv + odom.csv
        # itself (timestamped, human-readable) -> lighter on the Pi, and the
        # CSVs are the directly-exploitable dataset layer.
    ])
