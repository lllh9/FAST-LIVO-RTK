from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("output_topic", default_value="/livox/points"),
            Node(
                package="livox_pointcloud_converter",
                executable="livox_pointcloud_converter_node",
                name="livox_pointcloud_converter",
                output="screen",
                parameters=[
                    {
                        "input_topic": input_topic,
                        "output_topic": output_topic,
                    }
                ],
            ),
        ]
    )
