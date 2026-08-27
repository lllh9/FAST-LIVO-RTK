#include "msg_interfaces/msg/hc_sentence.hpp"
#include "msg_interfaces/msg/hcinspvatzcb.hpp"
#include "msg_interfaces/msg/hcrawimub.hpp"
#include "msg_interfaces/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <libgen.h>
#include <map>
#include <signal.h>
#include <stdio.h>
#include <string>

using namespace std;

static FILE *gs_fp__hc_time_record;
static FILE *gs_fp__nmea_time_record;
static FILE *gs_fp__devpvt_time_record;
static FILE *gs_fp__devimu_time_record;

static void hc_sentence_callback(const msg_interfaces::msg::HcSentence::ConstPtr msg);
static void nmea_sentence_callback(const msg_interfaces::msg::String::ConstPtr msg);
static void devpvt_sentence_callback(const msg_interfaces::msg::Hcinspvatzcb::ConstPtr msg);
static void devimu_sentence_callback(const msg_interfaces::msg::Hcrawimub::ConstPtr msg);

static void signal_exit(int sigo)
{
    fclose(gs_fp__hc_time_record);
    fclose(gs_fp__nmea_time_record);
    fclose(gs_fp__devpvt_time_record);
    fclose(gs_fp__devimu_time_record);

    return;
}

int main(int argc, char **argv)
{
    signal(SIGTERM, signal_exit); // signal to eixt
    signal(SIGINT, signal_exit);  // signal to eixt
    signal(SIGKILL, signal_exit); // signal to eixt

    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("TimeUniformityTestNode");
    rclcpp::Node::SharedPtr private_nh = nh;

    auto nmea_sentence_sub = private_nh->create_subscription<msg_interfaces::msg::String>("/chcnav/nmea_sentence", 1000, nmea_sentence_callback);
    auto devpvt_sentence_sub = private_nh->create_subscription<msg_interfaces::msg::Hcinspvatzcb>("/chcnav/devpvt333", 1000, devpvt_sentence_callback);
    auto devimu_sentence_sub = private_nh->create_subscription<msg_interfaces::msg::Hcrawimub>("/chcnav/devimu", 1000, devimu_sentence_callback);
    auto hc_sentence_sub = private_nh->create_subscription<msg_interfaces::msg::HcSentence>("/chcnav/hc_sentence", 1000, hc_sentence_callback);

    char fp_path[1024];
    char *dir_path = dirname(argv[0]);
    snprintf(fp_path, sizeof(fp_path), "%s/%s", dir_path, "hc_time_record");
    gs_fp__hc_time_record = fopen(fp_path, "w+");
    printf("%s\n", fp_path);

    snprintf(fp_path, sizeof(fp_path), "%s/%s", dir_path, "nmea_time_record");
    gs_fp__nmea_time_record = fopen(fp_path, "w+");
    printf("%s\n", fp_path);

    snprintf(fp_path, sizeof(fp_path), "%s/%s", dir_path, "devpvt_time_record");
    gs_fp__devpvt_time_record = fopen(fp_path, "w+");
    printf("%s\n", fp_path);

    snprintf(fp_path, sizeof(fp_path), "%s/%s", dir_path, "devimu_time_record");
    gs_fp__devimu_time_record = fopen(fp_path, "w+");
    printf("%s\n", fp_path);

    rclcpp::spin(nh);

    return 0;
}

static void hc_sentence_callback(const msg_interfaces::msg::HcSentence::ConstPtr msg)
{
    rclcpp::Time time = msg->header.stamp;
    // 获取秒数
    double seconds = time.seconds(); 
    fprintf(gs_fp__hc_time_record, "%lf\n", seconds);
}

static void nmea_sentence_callback(const msg_interfaces::msg::String::ConstPtr msg)
{
    rclcpp::Time time = msg->header.stamp;
    // 获取秒数
    double seconds = time.seconds(); 
    fprintf(gs_fp__nmea_time_record, "%lf\n", seconds);
}

static void devpvt_sentence_callback(const msg_interfaces::msg::Hcinspvatzcb::ConstPtr msg)
{
    rclcpp::Time time = msg->header.stamp;
    // 获取秒数
    double seconds = time.seconds(); 
    fprintf(gs_fp__devpvt_time_record, "%lf\n", seconds);
}

static void devimu_sentence_callback(const msg_interfaces::msg::Hcrawimub::ConstPtr msg)
{
    rclcpp::Time time = msg->header.stamp;
    // 获取秒数
    double seconds = time.seconds(); 
    fprintf(gs_fp__devimu_time_record, "%lf\n", seconds);
}
