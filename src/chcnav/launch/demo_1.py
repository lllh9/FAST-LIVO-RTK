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
         name = 'c_rs232',
         output = 'screen',
         parameters =[
            {"type": "serial"},
            #节点每秒解析最大协议数量
            {"rate": 1000},
            #串口路径
            {"port": "/dev/ttyUSB0"},
            #波特率
            {"baudrate": 460800},
         ]
      ),
   ])

