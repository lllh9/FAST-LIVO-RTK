from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包目录路径
    pkg_dir = get_package_share_directory('mvs_ros2_driver')
    
    # 配置相机节点
    camera_node = Node(
        package='mvs_ros2_driver',
        executable='mvs_camera_node',
        name='mvs_camera_trigger',
        arguments=[os.path.join(pkg_dir, 'config/left_camera_trigger.yaml'), '--ros-args', '--log-level', 'info'],
        respawn=True,
        output='screen',
        # 调试模式 (取消注释以启用)
        # launch_prefix=['xterm', '-e', 'gdb', '-ex', 'run', '--args']
    )
    
    # 配置RViz节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz_cfg/mvs_camera.rviz')]
    )
    
    return LaunchDescription([
        camera_node,
        #rviz_node
    ])