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

static FILE *gs_fp__nmea_time_record;
static FILE *gs_fp__devpvt_time_record;
static FILE *gs_fp__devimu_time_record;

static void nmea_sentence_callback(const msg_interfaces::msg::String::ConstPtr msg);
static void devpvt_sentence_callback(const msg_interfaces::msg::Hcinspvatzcb::ConstPtr msg);
static void devimu_sentence_callback(const msg_interfaces::msg::Hcrawimub::ConstPtr msg);

static void signal_exit(int sigo)
{
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
    auto nh = rclcpp::Node::make_shared("RecordMsgToFile");
    auto private_nh = nh;

    auto nmea_sentence_sub = private_nh->create_subscription<msg_interfaces::msg::String>("/chcnav/nmea_sentence", 1000, nmea_sentence_callback);
    auto devpvt_sentence_sub = private_nh->create_subscription<msg_interfaces::msg::Hcinspvatzcb>("/chcnav/devpvt222", 1000, devpvt_sentence_callback);
    auto devimu_sentence_sub = private_nh->create_subscription<msg_interfaces::msg::Hcrawimub>("/chcnav/devimu", 1000, devimu_sentence_callback);
 
    char fp_path[1024];
    char *dir_path = dirname(argv[0]);
    snprintf(fp_path, sizeof(fp_path), "%s/%s", dir_path, "nmea_sentence_record");
    gs_fp__nmea_time_record = fopen(fp_path, "w+");
    printf("%s\n", fp_path);

    snprintf(fp_path, sizeof(fp_path), "%s/%s", dir_path, "devpvt_sentence_record");
    gs_fp__devpvt_time_record = fopen(fp_path, "w+");
    printf("%s\n", fp_path);

    snprintf(fp_path, sizeof(fp_path), "%s/%s", dir_path, "devimu_sentence_record");
    gs_fp__devimu_time_record = fopen(fp_path, "w+");
    printf("%s\n", fp_path);

    rclcpp::spin(nh);

    return 0;
}

static void nmea_sentence_callback(const msg_interfaces::msg::String::ConstPtr msg)
{
    fprintf(gs_fp__nmea_time_record, "%d--", msg->header.stamp);
    fprintf(gs_fp__nmea_time_record, "%s\n", msg->sentence.c_str());
    printf("%s\n", msg->sentence.c_str());
}

static void devpvt_sentence_callback(const msg_interfaces::msg::Hcinspvatzcb::ConstPtr msg)
{
    rclcpp::Time time = msg->header.stamp;
    // 获取秒数
    double seconds = time.seconds(); 
    fprintf(gs_fp__devpvt_time_record, "%lf,", seconds);
    fprintf(gs_fp__devpvt_time_record, "%lf,", msg->latitude);
    fprintf(gs_fp__devpvt_time_record, "%lf,", msg->longitude);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->altitude);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->undulation);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->enu_velocity.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->enu_velocity.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->enu_velocity.z);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->roll);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->pitch);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->yaw);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->enu_velocity_stdev[0]);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->enu_velocity_stdev[1]);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->enu_velocity_stdev[2]);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->euler_stdev[0]);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->euler_stdev[1]);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->euler_stdev[2]);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_angular_velocity.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_angular_velocity.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_angular_velocity.z);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_linear_acceleration_without_g.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_linear_acceleration_without_g.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_linear_acceleration_without_g.z);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->speed);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->heading);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->heading2);
    fprintf(gs_fp__devpvt_time_record, "%d,", msg->stat[1] << 4 + msg->stat[0]);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->age);
    fprintf(gs_fp__devpvt_time_record, "%d,", msg->ns);
    fprintf(gs_fp__devpvt_time_record, "%d,", msg->ns2);
    fprintf(gs_fp__devpvt_time_record, "%d,", msg->leaps);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->hdop);
    fprintf(gs_fp__devpvt_time_record, "%d,", msg->warning);
    fprintf(gs_fp__devpvt_time_record, "%d,", msg->sensor_used);

    fprintf(gs_fp__devpvt_time_record, "%f,", msg->raw_angular_velocity.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->raw_angular_velocity.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->raw_angular_velocity.z);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->raw_acceleration.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->raw_acceleration.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->raw_acceleration.z);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_linear_acceleration.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_linear_acceleration.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_linear_acceleration.z);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_linear_velocity.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_linear_velocity.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vehicle_linear_velocity.z);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->pdop);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->vdop);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->tdop);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->gdop);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->ins2gnss_vector.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->ins2gnss_vector.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->ins2gnss_vector.z);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->ins2body_angle.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->ins2body_angle.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->ins2body_angle.z);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->gnss2body_vector.x);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->gnss2body_vector.y);
    fprintf(gs_fp__devpvt_time_record, "%f,", msg->gnss2body_vector.z);
    fprintf(gs_fp__devpvt_time_record, "%f", msg->gnss2body_angle_z);

    fprintf(gs_fp__devpvt_time_record, "\n");
}

static void devimu_sentence_callback(const msg_interfaces::msg::Hcrawimub::ConstPtr msg)
{
    rclcpp::Time time = msg->header.stamp;
    // 获取秒数
    double seconds = time.seconds(); 
    fprintf(gs_fp__devimu_time_record, "%lf,", seconds);
    fprintf(gs_fp__devimu_time_record, "%lf,", msg->angular_velocity.x);
    fprintf(gs_fp__devimu_time_record, "%lf,", msg->angular_velocity.y);
    fprintf(gs_fp__devimu_time_record, "%lf,", msg->angular_velocity.z);
    fprintf(gs_fp__devimu_time_record, "%lf,", msg->angular_acceleration.x);
    fprintf(gs_fp__devimu_time_record, "%lf,", msg->angular_acceleration.y);
    fprintf(gs_fp__devimu_time_record, "%lf,", msg->angular_acceleration.z);
    fprintf(gs_fp__devimu_time_record, "%lf,", msg->temp);
    fprintf(gs_fp__devimu_time_record, "%d,", msg->err_status);
    fprintf(gs_fp__devimu_time_record, "%d", msg->yaw);

    fprintf(gs_fp__devimu_time_record, "\n");
}
