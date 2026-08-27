#!/usr/bin/env python3
"""Launch FAST-LIVO2-RTK (ROS 2).

Starts the FAST-LIVO2 local front-end, independent real-time RTK/LIVO ESIKF and
asynchronous GTSAM iSAM2 global back-end. RTK and Scan Context/GICP loop closures
are optimized online; no stdin or Enter key is required. Call
`/global_backend/save_map` to rebuild the final optimized map, or stop the node
normally to save it automatically.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('fast_livo')
    default_params = os.path.join(pkg_share, 'config', 'HH-LVGO.yaml')
    rviz_cfg = os.path.join(pkg_share, 'rviz_cfg', 'fast_livo2.rviz')
    default_output = os.path.join(os.getcwd(), 'fast_livo_output')

    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('rviz')
    output_dir = LaunchConfiguration('output_dir')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Path to the parameter YAML.'),
        DeclareLaunchArgument('output_dir', default_value=default_output,
                              description='Where TUM trajectories / global maps are written '
                                          '(overrides outputfilepath in the YAML).'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Launch RViz.'),
        Node(
            package='fast_livo',
            executable='fastlivo_mapping',
            name='laserMapping',
            output='screen',
            emulate_tty=True,
            # The dict override wins over the YAML, so a stale/Docker-only
            # outputfilepath cannot send results to an unwritable path.
            parameters=[params_file, {'laserMapping.outputfilepath': output_dir}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(use_rviz),
        ),
    ])
