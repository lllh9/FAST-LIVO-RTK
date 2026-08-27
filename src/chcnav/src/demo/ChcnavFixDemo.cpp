#include "msg_interfaces/msg/hc_sentence.hpp"
#include "msg_interfaces/msg/hcinspvatzcb.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "chcnav/tf2_geometry_msgs.h"

#include <map>
#include <string>

using namespace std;

#define M_PI 3.14159265358979323846

static void pvt_callback(const msg_interfaces::msg::Hcinspvatzcb::ConstPtr msg);

static rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr gs_imu_pub;
static rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gs_fix_pub;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("ChcnavFixDemo");
    rclcpp::Node::SharedPtr private_nh = nh;

    tf2_ros::StaticTransformBroadcaster stf_pub(nh);
    tf2::Quaternion qtn;

    geometry_msgs::msg::TransformStamped ts;

    //ts.header.seq = 1;
    ts.header.stamp = rclcpp::Clock().now();
    ts.header.frame_id = "map";
    ts.child_frame_id = "chcnav";

    ts.transform.translation.x = 0;
    ts.transform.translation.y = 0;
    ts.transform.translation.z = 0;

    qtn.setRPY(0, 0, 0);

    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    ts.transform.rotation.w = qtn.getW();

    stf_pub.sendTransform(ts);

    ts.header.frame_id = "chcnav";
    ts.child_frame_id = "rs232";

    stf_pub.sendTransform(ts);

    gs_imu_pub = private_nh->create_publisher<sensor_msgs::msg::Imu>("imu", 1000);
    gs_fix_pub = private_nh->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 1000);

    auto pvt_source = private_nh->create_subscription<msg_interfaces::msg::Hcinspvatzcb>("/chcnav/devpvt111", 1000, pvt_callback);
    rclcpp::spin(nh);

    return 0;
}

static void pvt_callback(const msg_interfaces::msg::Hcinspvatzcb::ConstPtr msg)
{
    sensor_msgs::msg::Imu imu;
    sensor_msgs::msg::NavSatFix fix;

    // publish NavSatFix msg
    fix.header = msg->header;
    fix.altitude = msg->altitude;
    fix.longitude = msg->longitude;
    fix.latitude = msg->latitude;
    fix.status.service = fix.status.SERVICE_COMPASS;

    if (msg->stat[1] == 4 || msg->stat[1] == 8)
        fix.status.status = fix.status.STATUS_FIX;
    else
        fix.status.status = fix.status.STATUS_NO_FIX;

    gs_fix_pub->publish(fix);

    // publish imu msg
    imu.header = msg->header;
    imu.header.stamp = rclcpp::Clock().now();

    imu.angular_velocity.x = msg->vehicle_angular_velocity.x / 180 * M_PI;
    imu.angular_velocity.y = msg->vehicle_angular_velocity.y / 180 * M_PI;
    imu.angular_velocity.z = msg->vehicle_angular_velocity.z / 180 * M_PI;

    imu.linear_acceleration.x = msg->vehicle_linear_acceleration.x;
    imu.linear_acceleration.y = msg->vehicle_linear_acceleration.y;
    imu.linear_acceleration.z = msg->vehicle_linear_acceleration.z;

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
}
