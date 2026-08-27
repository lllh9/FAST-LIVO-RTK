from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
      Node(
         package = 'chcnav',
         executable = 'HcCgiProtocolProcessNode',
         name = 'hc_topic_driver',
         output = 'screen',
      ),
      Node(
         package = 'chcnav',
         executable = 'HcMsgParserLaunchNode',
         name = 'udp_7531',
         output = 'screen',
         parameters =[
            {"type": "udp"},
            #节点每秒解析最大协议数量
            {"rate": 1000},
            #端口号            
            {"port": 7531},
         ]
      ),
   ])
