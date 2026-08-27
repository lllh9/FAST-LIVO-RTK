from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    lidar_id = LaunchConfiguration("lidar_id")

    return LaunchDescription([
        DeclareLaunchArgument("input_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("output_topic", default_value="/livox/lidar_custom"),
        DeclareLaunchArgument("lidar_id", default_value="0"),
        Node(
            package="livox_pointcloud_converter",
            executable="pointcloud2_to_custom_node",
            name="pointcloud2_to_custom",
            output="screen",
            parameters=[{
                "input_topic": input_topic,
                "output_topic": output_topic,
                "lidar_id": ParameterValue(lidar_id, value_type=int),
            }],
        ),
    ])
