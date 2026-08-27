from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包目录路径
    pkg_dir = get_package_share_directory('mvs_ros2_driver')
    
    # 左相机节点配置
    left_camera_node = Node(
        package='mvs_ros2_driver',
        executable='mvs_camera_node',
        name='left_camera',
        arguments=[os.path.join(pkg_dir, 'config/left_camera_trigger.yaml'), '--ros-args', '--log-level', 'info'],
        respawn=True,
        output='screen',
        # 调试模式 (取消注释以启用)
        # launch_prefix=['xterm', '-e', 'gdb', '-ex', 'run', '--args']
    )
    
    # 右相机节点配置
    right_camera_node = Node(
        package='mvs_ros2_driver',
        executable='mvs_camera_node',
        name='right_camera',
        arguments=[os.path.join(pkg_dir, 'config/right_camera_trigger.yaml')],
        respawn=True,
        output='screen',
        # 调试模式 (取消注释以启用)
        # launch_prefix=['xterm', '-e', 'gdb', '-ex', 'run', '--args']
    )
    
    # RViz2节点配置
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz_cfg/mvs_camera.rviz')],
        output='screen'
    )
    
    return LaunchDescription([
        left_camera_node,
        right_camera_node,
        rviz_node
    ])