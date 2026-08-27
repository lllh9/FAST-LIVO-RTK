#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <array>
#include <algorithm> // 包含算法头文件
#include "msg_interfaces/msg/hc_sentence.hpp"
#include "msg_interfaces/msg/hcinspvatzcb.hpp"
#include "msg_interfaces/msg/hcrawimub.hpp"
#include "hc_cgi_protocol.h"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "chcnav/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/msg/quaternion.hpp>
#include "autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp"

#define M_PI 3.14159265358979323846
static void pvt_callback(const msg_interfaces::msg::Hcinspvatzcb::ConstPtr msg);

static rclcpp::Publisher<msg_interfaces::msg::Hcinspvatzcb>::SharedPtr gs_devpvt_pub;
static rclcpp::Publisher<msg_interfaces::msg::Hcrawimub>::SharedPtr gs_devimu_pub;

static rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gs_fix_pub;
static rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr gs_imu_pub;
static rclcpp::Publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr gs_Orientation_pub;

static unsigned int g_leaps = 18;
static std::string imu_frame_id;
static std::string gnss_frame_id;
/**
 * @brief 处理华测协议的回调函数
 *
 * @param msg 接收到的数据
 * */
static void hc_sentence_callback(const msg_interfaces::msg::HcSentence::ConstPtr msg);

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("HcCgiProtocolProcessNode");
    std::shared_ptr<rclcpp::Node> private_nh = nh;

    private_nh->declare_parameter<std::string>("imu_frame", imu_frame_id);
    private_nh->get_parameter("imu_frame", imu_frame_id); 
    //std::cout<<" imu_frame_id "<<imu_frame_id<<std::endl;

    private_nh->declare_parameter<std::string>("gnss_frame", gnss_frame_id);
    private_nh->get_parameter("gnss_frame", gnss_frame_id); 

     //std::cout<<" gnss_frame_id "<<gnss_frame_id<<std::endl;

    auto pvt_source = private_nh->create_subscription<msg_interfaces::msg::Hcinspvatzcb>("/chcnav/devpvt", 1000, pvt_callback);
    auto serial_suber = nh->create_subscription<msg_interfaces::msg::HcSentence>("/chcnav/hc_sentence", 1000, hc_sentence_callback);

    gs_fix_pub = private_nh->create_publisher<sensor_msgs::msg::NavSatFix>("/sensing/gnss/ublox/nav_sat_fix", 1000);
    gs_imu_pub = private_nh->create_publisher<sensor_msgs::msg::Imu>("/sensing/imu/tamagawa/imu_raw", 1000);
    gs_Orientation_pub=private_nh->create_publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>("/autoware_orientation", 1000);

    gs_devpvt_pub = nh->create_publisher<msg_interfaces::msg::Hcinspvatzcb>("/chcnav/devpvt", 1000);
    gs_devimu_pub = nh->create_publisher<msg_interfaces::msg::Hcrawimub>("/chcnav/devimu", 1000);

    rclcpp::spin(nh);
    rclcpp::shutdown();
    return 0;
}

// 处理各个协议的函数
static void msg_deal__hcrawgnsspvatb(const msg_interfaces::msg::HcSentence::ConstPtr &msg);
static void msg_deal__hcrawimuib(const msg_interfaces::msg::HcSentence::ConstPtr &msg);
static void msg_deal__hcrawodob(const msg_interfaces::msg::HcSentence::ConstPtr &msg);
static void msg_deal__hcrawrtcmpb(const msg_interfaces::msg::HcSentence::ConstPtr &msg);
static void msg_deal__hcrawrtcmsb(const msg_interfaces::msg::HcSentence::ConstPtr &msg);
static void msg_deal__hcrawrtcmb(const msg_interfaces::msg::HcSentence::ConstPtr &msg);
static void msg_deal__hcrawimub(const msg_interfaces::msg::HcSentence::ConstPtr &msg);
static void msg_deal__hcrawimuvb(const msg_interfaces::msg::HcSentence::ConstPtr &msg);
static void msg_deal__hcrawgsvb(const msg_interfaces::msg::HcSentence::ConstPtr &msg);
static void msg_deal__hcrawnmeab(const msg_interfaces::msg::HcSentence::ConstPtr &msg);
static void msg_deal__hcinspvatb(const msg_interfaces::msg::HcSentence::ConstPtr &msg);
static void msg_deal__hcinspvatzcb(const msg_interfaces::msg::HcSentence::ConstPtr &msg);
static void msg_deal__hcpinfoltsb(const msg_interfaces::msg::HcSentence::ConstPtr &msg);
static void msg_deal__hcpinfonzb(const msg_interfaces::msg::HcSentence::ConstPtr &msg);

/**
 * @brief 处理华测协议的回调函数
 *
 * @param msg 接收到的数据
 * */
static void hc_sentence_callback(const msg_interfaces::msg::HcSentence::ConstPtr msg)
{
    if (hc__cgi_check_crc32((uint8_t *)(&msg->data[0]), msg->data.size()) != 0)
    {
        fprintf(stderr, "crc32 check failed!\n");
        return;
    }

    switch (msg->msg_id)
    {
        case INSPVATZCB:
            msg_deal__hcinspvatzcb(msg);
            break;
        case RAWIMUIB:
            msg_deal__hcrawimuib(msg);
            break;
        default:
            break;
    }

    return;
}

static void msg_deal__hcinspvatzcb(const msg_interfaces::msg::HcSentence::ConstPtr &msg)
{
    msg_interfaces::msg::Hcinspvatzcb devpvt;

    // devpvt的header使用原msg的header
    devpvt.header = msg->header;
 
    // 如果长度不对，不解析发布
    if (msg->data.size() == 296)
    {
        // 润秒
        devpvt.leaps = *((unsigned short *)(&msg->data[162]));
        g_leaps = devpvt.leaps;

        // gps 周 周内秒
        devpvt.week = *((unsigned short *)(&msg->data[22]));
        devpvt.second = *((double *)(&msg->data[24]));
        // 计算时间戳，注意ROS2使用纳秒而不是秒,所以 * 1e9
        devpvt.header.stamp = rclcpp::Time((devpvt.week * 7.0 * 24.0 * 3600.0 + devpvt.second + 315964800.0 - g_leaps) * 1e9);
        
        // 经纬高
        devpvt.latitude = *((double *)(&msg->data[32]));
        devpvt.longitude = *((double *)(&msg->data[40]));
        devpvt.altitude = *((float *)(&msg->data[48]));

        devpvt.position_stdev[0] = *((float *)(&msg->data[80]));
        devpvt.position_stdev[1] = *((float *)(&msg->data[84]));
        devpvt.position_stdev[2] = *((float *)(&msg->data[88]));

        devpvt.undulation = *((float *)(&msg->data[52]));

        // 姿态角
        devpvt.roll = *((float *)(&msg->data[72]));
        devpvt.pitch = *((float *)(&msg->data[68]));
        devpvt.yaw = *((float *)(&msg->data[76]));

        devpvt.euler_stdev[0] = *((float *)(&msg->data[108])); // std_roll
        devpvt.euler_stdev[1] = *((float *)(&msg->data[104])); // std_pitch
        devpvt.euler_stdev[2] = *((float *)(&msg->data[112])); // std_yaw

        devpvt.speed = *((float *)(&msg->data[140]));
        devpvt.heading = *((float *)(&msg->data[144]));
        devpvt.heading2 = *((float *)(&msg->data[148]));

        // vehicle velocity and acceleration
        devpvt.vehicle_angular_velocity.x = *((float *)(&msg->data[116]));
        devpvt.vehicle_angular_velocity.y = *((float *)(&msg->data[120]));
        devpvt.vehicle_angular_velocity.z = *((float *)(&msg->data[124]));

        devpvt.vehicle_linear_velocity.x = *((float *)(&msg->data[208]));
        devpvt.vehicle_linear_velocity.y = *((float *)(&msg->data[212]));
        devpvt.vehicle_linear_velocity.z = *((float *)(&msg->data[216]));

        devpvt.vehicle_linear_acceleration.x = *((float *)(&msg->data[196]));
        devpvt.vehicle_linear_acceleration.y = *((float *)(&msg->data[200]));
        devpvt.vehicle_linear_acceleration.z = *((float *)(&msg->data[204]));

        devpvt.vehicle_linear_acceleration_without_g.x = *((float *)(&msg->data[128]));
        devpvt.vehicle_linear_acceleration_without_g.y = *((float *)(&msg->data[132]));
        devpvt.vehicle_linear_acceleration_without_g.z = *((float *)(&msg->data[136]));

        devpvt.enu_velocity.x = *((float *)(&msg->data[56]));
        devpvt.enu_velocity.y = *((float *)(&msg->data[60]));
        devpvt.enu_velocity.z = *((float *)(&msg->data[64]));

        devpvt.enu_velocity_stdev[0] = *((float *)(&msg->data[92]));
        devpvt.enu_velocity_stdev[1] = *((float *)(&msg->data[96]));
        devpvt.enu_velocity_stdev[2] = *((float *)(&msg->data[100]));

        // 原始imu数据
        devpvt.raw_angular_velocity.x = *((float *)(&msg->data[172]));
        devpvt.raw_angular_velocity.y = *((float *)(&msg->data[176]));
        devpvt.raw_angular_velocity.z = *((float *)(&msg->data[180]));

        devpvt.raw_acceleration.x = *((float *)(&msg->data[184]));
        devpvt.raw_acceleration.y = *((float *)(&msg->data[188]));
        devpvt.raw_acceleration.z = *((float *)(&msg->data[192]));

        // stat, warning and flags
        devpvt.stat[0] = msg->data[152] & 0x0f;
        devpvt.stat[1] = (msg->data[152] >> 4) & 0x0f;

        devpvt.age = *((float *)(&msg->data[154]));

        devpvt.ns = *((unsigned short *)(&msg->data[158]));
        devpvt.ns2 = *((unsigned short *)(&msg->data[160]));

        // dop
        devpvt.hdop = *((float *)(&msg->data[164]));
        devpvt.pdop = *((float *)(&msg->data[220]));
        devpvt.vdop = *((float *)(&msg->data[224]));
        devpvt.tdop = *((float *)(&msg->data[228]));
        devpvt.gdop = *((float *)(&msg->data[232]));

        // warning
        devpvt.warning = *((unsigned short *)(&msg->data[168]));
        devpvt.sensor_used = *((unsigned short *)(&msg->data[170]));

        // body
        devpvt.ins2gnss_vector.x = *((float *)(&msg->data[236]));
        devpvt.ins2gnss_vector.y = *((float *)(&msg->data[240]));
        devpvt.ins2gnss_vector.z = *((float *)(&msg->data[244]));

        devpvt.ins2body_angle.x = *((float *)(&msg->data[248]));
        devpvt.ins2body_angle.y = *((float *)(&msg->data[252]));
        devpvt.ins2body_angle.z = *((float *)(&msg->data[256]));

        devpvt.gnss2body_vector.x = *((float *)(&msg->data[260]));
        devpvt.gnss2body_vector.y = *((float *)(&msg->data[264]));
        devpvt.gnss2body_vector.z = *((float *)(&msg->data[268]));

        devpvt.gnss2body_angle_z = *((float *)(&msg->data[272]));

        for (int index = 0; index < 16; index++)
            devpvt.receiver[index] = *((unsigned char *)(&msg->data[276 + index]));

        gs_devpvt_pub->publish(devpvt);
    }
}

static void msg_deal__hcrawimuib(const msg_interfaces::msg::HcSentence::ConstPtr &msg)
{
    msg_interfaces::msg::Hcrawimub devimu;

    // devpvt的header使用原msg的header
    devimu.header = msg->header;
    devimu.header.frame_id=imu_frame_id;
    // 如果长度不对，不解析发布
    if (msg->data.size() == 68)
    {
        // header的时间设置为gps时间
        devimu.week = *((unsigned short *)(&msg->data[22]));
        devimu.second = *((double *)(&msg->data[24]));
        // 计算时间戳，注意ROS2使用纳秒而不是秒,所以 * 1e9
        devimu.header.stamp = rclcpp::Time((devimu.week * 7.0 * 24.0 * 3600.0 + devimu.second + 315964800.0 - g_leaps) * 1e9);

        // xyz角速度
        devimu.angular_velocity.x = *((float *)(&msg->data[32]));
        devimu.angular_velocity.y = *((float *)(&msg->data[36]));
        devimu.angular_velocity.z = *((float *)(&msg->data[40]));

        // xyz角角速度 g
        devimu.angular_acceleration.x = *((float *)(&msg->data[44]));
        devimu.angular_acceleration.y = *((float *)(&msg->data[48]));
        devimu.angular_acceleration.z = *((float *)(&msg->data[52]));

        // 温度
        devimu.temp = *((float *)(&msg->data[56]));

        // 异常表示
        devimu.err_status = *((unsigned char *)(&msg->data[60]));

        // Z轴陀螺积分航向, 180~180 系数 0.01
        devimu.yaw = *((short *)(&msg->data[61]));

        // 预留
        devimu.receiver = *((unsigned char *)(&msg->data[63]));

        gs_devimu_pub->publish(devimu);
    }
    //short 
    if (msg->data.size() == 34)
    {
        // header的时间设置为gps时间
        devimu.week = *((unsigned short *)(&msg->data[8]));
        devimu.second = ((double)*((unsigned int *)(&msg->data[10]))) / 1000.0;
        // 计算时间戳，注意ROS2使用纳秒而不是秒,所以 * 1e9
        devimu.header.stamp = rclcpp::Time((devimu.week * 7.0 * 24.0 * 3600.0 + devimu.second + 315964800.0 - g_leaps) * 1e9);

        // xyz角速度
        devimu.angular_velocity.x = ((float)*((short *)(&msg->data[14]))) / 80.0;
        devimu.angular_velocity.y = ((float)*((short *)(&msg->data[16]))) / 80.0;
        devimu.angular_velocity.z = ((float)*((short *)(&msg->data[18]))) / 80.0;

        // xyz角角速度 g
        devimu.angular_acceleration.x = ((float)*((short *)(&msg->data[20]))) / 5000.0;
        devimu.angular_acceleration.y = ((float)*((short *)(&msg->data[22]))) / 5000.0;
        devimu.angular_acceleration.z = ((float)*((short *)(&msg->data[24]))) / 5000.0;

        // 温度
        devimu.temp = ((float)*((short *)(&msg->data[26]))) / 100.0;

        // 异常表示
        devimu.err_status = *((unsigned char *)(&msg->data[28]));

        // Z轴陀螺积分航向, 180~180 系数 0.01
        //devimu.yaw = *((short *)(&msg->data[61]));

        // 预留
        devimu.receiver = *((unsigned char *)(&msg->data[29]));

        gs_devimu_pub->publish(devimu);
    }
}


static void pvt_callback(const msg_interfaces::msg::Hcinspvatzcb::ConstPtr msg)
{
    sensor_msgs::msg::Imu imu;
    sensor_msgs::msg::NavSatFix fix;

    // publish NavSatFix msg
    fix.header = msg->header;
    fix.header.frame_id=gnss_frame_id;
    fix.altitude = msg->altitude;
    fix.longitude = msg->longitude;
    fix.latitude = msg->latitude;
    fix.status.service = fix.status.SERVICE_GPS;
    std::array<double, 9> position_covariance_msg={
                            msg->position_stdev[1]*msg->position_stdev[1],0.0,0.0,
                             0.0,msg->position_stdev[0]*msg->position_stdev[0],0.0,
                             0.0,0.0,msg->position_stdev[2]*msg->position_stdev[2]};
    std::copy(position_covariance_msg.begin(), position_covariance_msg.end(), fix.position_covariance.begin());

    fix.position_covariance_type=sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
                           
    if (msg->stat[1] == 4 || msg->stat[1] == 8)
        fix.status.status = fix.status.STATUS_FIX;
    else
        fix.status.status = fix.status.STATUS_NO_FIX;

    gs_fix_pub->publish(fix);

    // publish imu msg
    imu.header = msg->header;
    imu.header.frame_id=imu_frame_id;
    imu.header.stamp = rclcpp::Clock().now();

    imu.angular_velocity.x = msg->vehicle_angular_velocity.x / 180 * M_PI;
    imu.angular_velocity.y = msg->vehicle_angular_velocity.y / 180 * M_PI;
    imu.angular_velocity.z = msg->vehicle_angular_velocity.z / 180 * M_PI;

    imu.linear_acceleration.x = msg->vehicle_linear_acceleration.x*9.8;
    imu.linear_acceleration.y = msg->vehicle_linear_acceleration.y*9.8;
    imu.linear_acceleration.z = msg->vehicle_linear_acceleration.z*9.8;

    // yaw: 车体坐标系下双天线航向角，取值范围 [-180, +180]，遵循右手定则，逆时针为正。
    // heading: 为车体坐标系下速度航向角，也称航迹角。取值范围 [0, 360] ，顺时针为正。
    // heading2: 为车体坐标系下双天线航向角，取值范围 [0, 360] ，顺时针为正。
    float yaw = 0.0;
    if (msg->yaw <= 180)
        yaw = -1 * msg->heading2;
    else
        yaw = 360 - msg->heading2;

    tf2::Quaternion qtn;
    qtn.setRPY(msg->roll / 180 * M_PI, -msg->pitch / 180 * M_PI, yaw / 180 * M_PI);
    // 将tf2::Quaternion对象转换为geometry_msgs::msg::Quaternion消息
    geometry_msgs::msg::Quaternion qtn_msg;
    tf2::convert(qtn, qtn_msg);
    imu.orientation = qtn_msg;

    gs_imu_pub->publish(imu);

    autoware_sensing_msgs::msg::GnssInsOrientationStamped gs_Orientation;
    tf2::Quaternion tf2_quaternion;
    geometry_msgs::msg::Quaternion geometry_quaternion;
    tf2_quaternion.setRPY(msg->roll, msg->pitch, msg->yaw);
    tf2::convert(tf2_quaternion, geometry_quaternion);
    gs_Orientation.header=msg->header;
    gs_Orientation.orientation.orientation=geometry_quaternion;  
    gs_Orientation.orientation.rmse_rotation_x=msg->euler_stdev[0];
    gs_Orientation.orientation.rmse_rotation_y=msg->euler_stdev[1];
    gs_Orientation.orientation.rmse_rotation_z=msg->euler_stdev[2];

    gs_Orientation_pub->publish(gs_Orientation);
}