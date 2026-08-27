import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    adapter_share = get_package_share_directory('chcnav_gnss_adapter')
    fast_livo_share = get_package_share_directory('fast_livo')
    default_params = os.path.join(adapter_share, 'config', 'mid360_a0gc_rtk.yaml')
    # Use a dedicated RTK mapping layout.  The generic FAST-LIVO2 launch keeps
    # using fast_livo2.rviz, while this profile exposes local, RTK/ESIKF and
    # GTSAM trajectories as opt-in displays for side-by-side comparison.
    rviz_config = os.path.join(fast_livo_share, 'rviz_cfg', 'mid360_chcnav_rtk.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('output_dir', default_value=os.path.join(os.getcwd(), 'fast_livo_output')),
        DeclareLaunchArgument('chcnav_topic', default_value='/chcnav/devpvt'),
        DeclareLaunchArgument('accept_float', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='chcnav_gnss_adapter',
            executable='chcnav_to_gnss',
            name='chcnav_gnss_adapter',
            parameters=[{
                'input_topic': LaunchConfiguration('chcnav_topic'),
                'output_topic': '/ublox_driver/receiver_pvt',
                'accept_float': LaunchConfiguration('accept_float'),
            }],
            output='screen',
        ),
        Node(
            package='fast_livo',
            executable='fastlivo_mapping',
            name='laserMapping',
            parameters=[
                LaunchConfiguration('params_file'),
                {'laserMapping.outputfilepath': LaunchConfiguration('output_dir')},
            ],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
