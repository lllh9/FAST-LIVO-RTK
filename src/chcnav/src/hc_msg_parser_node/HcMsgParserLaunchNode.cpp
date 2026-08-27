#include "rclcpp/rclcpp.hpp"

// msgs insclude
#include "msg_interfaces/msg/hc_sentence.hpp"
#include "msg_interfaces/msg/int8_array.hpp"
#include "msg_interfaces/msg/string.hpp"
#include "device_connector.hpp"
#include "hc_msg_parser.hpp"
#include "serial_common.hpp"
#include "std_msgs/msg/string.hpp"
#include "tcp_common.hpp"
#include "udp_common.hpp"
#include "can_common.hpp"

#include <signal.h>
#include <string.h>
#include <string>

#include "file_common.hpp"

using namespace std;

static void signal_exit(int sigo)
{
    exit(0);
    return;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
     // 初始化ROS 2节点指针
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("HcMsgParserLaunchNode");
    // 创建私有ROS 2节点指针
    rclcpp::Node::SharedPtr private_nh = nh;

    signal(SIGTERM, signal_exit); // signal to eixt
    signal(SIGINT, signal_exit);  // signal to eixt
    signal(SIGKILL, signal_exit); // signal to eixt

    hc__device_connector *device_connector;

    std::string type = ""; // 参数配置类型
    // 声明参数，如果参数未在Launch文件中设置，则使用默认值
    private_nh->declare_parameter<std::string>("type", type);   
    if (private_nh->get_parameter<std::string>("type", type) == false)
    {
        RCLCPP_ERROR(nh->get_logger(), "chcnav: type is not set.");
        return -1;
    }

    int rate = 1000; // 获取处理频率
    // 声明参数，如果参数未在Launch文件中设置，则使用默认值
    private_nh->declare_parameter<int>("rate", rate);
    if (private_nh->get_parameter<int>("rate", rate) == rate)
        RCLCPP_INFO(nh->get_logger(), "rate [0], No message pub deal subscribe only");
    RCLCPP_INFO(nh->get_logger(), "rate : %d\n", rate);

    if (type.compare("serial") == 0)
    {
        // 如果参数配置为串口
        std::string port = "", parity = "None";
        int baudrate = 115200, databits = 8, stopbits = 1;
        // 声明参数，如果参数未在Launch文件中设置，则使用默认值
        private_nh->declare_parameter("port", port);   
        if (private_nh->get_parameter("port", port) == false)
        {
            RCLCPP_ERROR(nh->get_logger(), "chcnav: port is not set.");
            return -1;
        }
        private_nh->declare_parameter<int>("baudrate", baudrate);
        private_nh->declare_parameter<int>("databits", databits);
        private_nh->declare_parameter<int>("stopbits", stopbits);
        private_nh->declare_parameter<std::string>("parity", parity);

        private_nh->get_parameter<int>("baudrate", baudrate);
        private_nh->get_parameter<int>("databits", databits);
        private_nh->get_parameter<int>("stopbits", stopbits);
        private_nh->get_parameter<std::string>("parity", parity);

        RCLCPP_INFO(nh->get_logger(), "serial config port[%s] baudrate[%d] databits[%d] stopbits[%d] parity[%s] rate [%d]",
                 port.c_str(), baudrate, databits, stopbits, parity.c_str(), rate);

        device_connector = new serial_common(port, baudrate, databits, stopbits, parity);
    }
    else if (type.compare("tcp") == 0)
    {
        string host = "";
        int port = 0;
        private_nh->declare_parameter<std::string>("host", host);
        private_nh->declare_parameter<int>("port", port);

        if (private_nh->get_parameter("host", host) == false)
        {
            RCLCPP_ERROR(nh->get_logger(), "chcnav: host is not set.");
            return -1;
        }

        if (private_nh->get_parameter("port", port) == false)
        {
            RCLCPP_ERROR(nh->get_logger(), "chcnav: port is not set.");
            return -1;
        }

        RCLCPP_INFO(nh->get_logger(), "tcp config host[%s] port[%d]", host.c_str(), port);

        device_connector = new tcp_common(host, port);
    }
    else if (type.compare("file") == 0)
    {
        string path = "";
        // 声明参数，如果参数未在Launch文件中设置，则使用默认值
        private_nh->declare_parameter("path", path);  
        if (private_nh->get_parameter("path", path) == false)
        {
            RCLCPP_ERROR(nh->get_logger(), "chcnav: path is not set.");
            return -1;
        }

        RCLCPP_INFO(nh->get_logger(), "file path[%s]", path.c_str());

        usleep(500 * 1000); // 很重要，等待其他节点都启动完毕再解析文件，不然会造成先解析文件，导致一些在别的节点启动前就被解析出的数据未处理。
        device_connector = new file_common(path);
    }
    else if (type.compare("udp") == 0)
    {
        int port = 0;
        private_nh->declare_parameter<int>("port", port);
        
        if (private_nh->get_parameter("port", port) == false)
        {
            RCLCPP_ERROR (nh->get_logger(), "chcnav: port is not set.");
            return -1;
        }

        RCLCPP_INFO(nh->get_logger(), "udp config port[%d]", port);

        device_connector = new udp_common(port);
    }
    else if (type.compare("can") == 0)
    {
        std::string dev = "";
        int can_rate = 0;     
        std::string data_format = "";
        int time_id = 0;
        int angrate_rawIMU_id = 0;
        int accel_rawIMU_id = 0;
        int sys_status_id = 0;
        int altitude_id = 0;
        int pos_sigma_id = 0;
        int velocity_level_id = 0;
        int velocity_level_sigma_id = 0;
        int accel_vehicle_id = 0;
        int heading_pitch_roll_id = 0;
        int heading_pitch_roll_sigma_id = 0;
        int angrate_vehicle_id = 0;     
        int longitude_id = 0;
        int latitude_id = 0;             

        private_nh->declare_parameter("dev", dev);
        private_nh->declare_parameter<int>("can_rate", can_rate);        
        private_nh->declare_parameter("data_format", data_format);
        private_nh->declare_parameter<int>("time_id", time_id);
        private_nh->declare_parameter<int>("angrate_rawIMU_id", angrate_rawIMU_id);
        private_nh->declare_parameter<int>("accel_rawIMU_id", accel_rawIMU_id);
        private_nh->declare_parameter<int>("sys_status_id", sys_status_id);
        private_nh->declare_parameter<int>("altitude_id", altitude_id);
        private_nh->declare_parameter<int>("pos_sigma_id", pos_sigma_id);
        private_nh->declare_parameter<int>("velocity_level_id", velocity_level_id);
        private_nh->declare_parameter<int>("velocity_level_sigma_id", velocity_level_sigma_id);
        private_nh->declare_parameter<int>("accel_vehicle_id", accel_vehicle_id);
        private_nh->declare_parameter<int>("heading_pitch_roll_id", heading_pitch_roll_id);
        private_nh->declare_parameter<int>("heading_pitch_roll_sigma_id", heading_pitch_roll_sigma_id);
        private_nh->declare_parameter<int>("angrate_vehicle_id", angrate_vehicle_id);
        private_nh->declare_parameter<int>("longitude_id", longitude_id);
        private_nh->declare_parameter<int>("latitude_id", latitude_id);

        if (private_nh->get_parameter("dev", dev) == false)
        {
            RCLCPP_ERROR (nh->get_logger(), "chcnav: dev is not set.");
            return -1;
        }

        if (private_nh->get_parameter("can_rate", can_rate) == false)
        {
            RCLCPP_ERROR (nh->get_logger(), "chcnav: can_rate is not set.");
            return -1;
        }

        if (private_nh->get_parameter("data_format", data_format) == false)
        {
            RCLCPP_ERROR (nh->get_logger(), "chcnav: data_format is not set.");
            return -1;
        }
        else if(!((data_format.compare("motorola") == 0) | (data_format.compare("intel") == 0)))
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "The data_format parameter should be motorola or intel!");
            return -1;
        }

        if (private_nh->get_parameter("time_id", time_id) == false)
        {
            RCLCPP_ERROR (nh->get_logger(), "chcnav: time_id is not set.");
            return -1;
        }

        if (private_nh->get_parameter("angrate_rawIMU_id", angrate_rawIMU_id) == false)
        {
            RCLCPP_ERROR (nh->get_logger(), "chcnav: angrate_rawIMU_id is not set.");
            return -1;
        }

        if (private_nh->get_parameter("accel_rawIMU_id", accel_rawIMU_id) == false)
        {
            RCLCPP_ERROR (nh->get_logger(), "chcnav: accel_rawIMU_id is not set.");
            return -1;
        }

        if (private_nh->get_parameter("sys_status_id", sys_status_id) == false)
        {
            RCLCPP_ERROR (nh->get_logger(), "chcnav: sys_status_id is not set.");
            return -1;
        }   

        if (private_nh->get_parameter("altitude_id", altitude_id) == false)
        {
            RCLCPP_ERROR (nh->get_logger(), "chcnav: altitude_id is not set.");
            return -1;
        }

        if (private_nh->get_parameter("pos_sigma_id", pos_sigma_id) == false)
        {
            RCLCPP_ERROR (nh->get_logger(), "chcnav: pos_sigma_id is not set.");
            return -1;
        }

        if (private_nh->get_parameter("velocity_level_id", velocity_level_id) == false)
        {
            RCLCPP_ERROR (nh->get_logger(), "chcnav: velocity_level_id is not set.");
            return -1;
        }

        if (private_nh->get_parameter("velocity_level_sigma_id", velocity_level_sigma_id) == false)
        {
            RCLCPP_ERROR (nh->get_logger(), "chcnav: velocity_level_sigma_id is not set.");
            return -1;
        }  

        if (private_nh->get_parameter("accel_vehicle_id", accel_vehicle_id) == false)
        {
            RCLCPP_ERROR (nh->get_logger(), "chcnav: accel_vehicle_id is not set.");
            return -1;
        }

        if (private_nh->get_parameter("heading_pitch_roll_id", heading_pitch_roll_id) == false)
        {
            RCLCPP_ERROR (nh->get_logger(), "chcnav: heading_pitch_roll_id is not set.");
            return -1;
        }   

        if (private_nh->get_parameter("heading_pitch_roll_sigma_id", heading_pitch_roll_sigma_id) == false)
        {
            RCLCPP_ERROR (nh->get_logger(), "chcnav: heading_pitch_roll_sigma_id is not set.");
            return -1;
        }

        if (private_nh->get_parameter("angrate_vehicle_id", angrate_vehicle_id) == false)
        {
            RCLCPP_ERROR (nh->get_logger(), "chcnav: angrate_vehicle_id is not set.");
            return -1;
        }

        if (private_nh->get_parameter("longitude_id", longitude_id) == false)
        {
            RCLCPP_ERROR (nh->get_logger(), "chcnav: longitude_id is not set.");
            return -1;
        }

        if (private_nh->get_parameter("latitude_id", latitude_id) == false)
        {
            RCLCPP_ERROR (nh->get_logger(), "chcnav: latitude_id is not set.");
            return -1;
        }  

        RCLCPP_INFO(nh->get_logger(), "CAN config dev[%s], can_rate[%d], data_format[%s], time_id[%d], angrate_rawIMU_id[%d],\
            accel_rawIMU_id[%d], sys_status_id[%d], altitude_id[%d], pos_sigma_id[%d], velocity_level_id[%d],\
            velocity_level_sigma_id[%d], accel_vehicle_id[%d], heading_pitch_roll_id[%d], heading_pitch_roll_sigma_id[%d],\
            angrate_vehicle_id[%d], longitude_id[%d], latitude_id[%d]", dev.c_str(), can_rate, data_format.c_str(), time_id, angrate_rawIMU_id,\
            accel_rawIMU_id, sys_status_id, altitude_id, pos_sigma_id, velocity_level_id, velocity_level_sigma_id,\
            accel_vehicle_id, heading_pitch_roll_id, heading_pitch_roll_sigma_id, angrate_vehicle_id, longitude_id, latitude_id);

        device_connector = new can_common(dev, can_rate, data_format, time_id, angrate_rawIMU_id, accel_rawIMU_id, sys_status_id,\
            altitude_id, pos_sigma_id, velocity_level_id, velocity_level_sigma_id, accel_vehicle_id, heading_pitch_roll_id,\
            heading_pitch_roll_sigma_id, angrate_vehicle_id, longitude_id,latitude_id);
    }

   // 获取节点名
    string node_path = nh->get_name();
    string node_name = node_path.substr(node_path.find_last_of("/") + 1);

    // 初始化信息解析节点
    hc__msg_parser_node parser_node(rate, device_connector, nh, private_nh, node_name);

    parser_node.device_connector->connect();
    parser_node.node_start();

    rclcpp::shutdown();
    return 0;
}
