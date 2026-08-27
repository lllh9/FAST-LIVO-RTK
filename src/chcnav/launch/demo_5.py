from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
   type_arg = DeclareLaunchArgument(
      'type', default_value=TextSubstitution(text='file')
   )
   rate_arg = DeclareLaunchArgument(
      'rate', default_value=TextSubstitution(text='1000')
   )
   path_arg = DeclareLaunchArgument(
      #注意文件路径不要出现“~”等异常字符
      'path', default_value=TextSubstitution(text='/home/zxy/ros2_ws/src/chcnav/doc/record.txt')
   )
   
   return LaunchDescription([
      type_arg,
      rate_arg,
      path_arg,
      Node(
         package='chcnav',
         node_executable='RecordMsgToFile',
         name='record_msg_to_file',
         output='screen',
      ),
      Node(
         package='chcnav',
         node_executable='HcCgiProtocolProcessNode',
         name='hc_topic_driver',
         output='screen',
      ),
      Node(
         package='chcnav',
         node_executable='HcMsgParserLaunchNode',
         name='file',
         output='screen',
         parameters=[{
            'type': LaunchConfiguration('type'),
            'rate': LaunchConfiguration('rate'),
            'path': LaunchConfiguration('path'),
         }]
      ),
   ])

