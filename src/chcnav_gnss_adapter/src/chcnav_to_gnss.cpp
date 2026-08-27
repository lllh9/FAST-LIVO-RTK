#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <gnss_comm/msg/gnss_pvt_soln_msg.hpp>
#include <msg_interfaces/msg/hcinspvatzcb.hpp>

class ChcnavGnssAdapter final : public rclcpp::Node
{
public:
  ChcnavGnssAdapter() : Node("chcnav_gnss_adapter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/chcnav/devpvt");
    output_topic_ = declare_parameter<std::string>("output_topic", "/ublox_driver/receiver_pvt");
    accept_float_ = declare_parameter<bool>("accept_float", false);

    pub_ = create_publisher<gnss_comm::msg::GnssPVTSolnMsg>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(2000)));
    sub_ = create_subscription<msg_interfaces::msg::Hcinspvatzcb>(
      input_topic_, rclcpp::QoS(rclcpp::KeepLast(2000)),
      std::bind(&ChcnavGnssAdapter::callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "CHCNAV adapter: %s -> %s, accept_float=%s",
      input_topic_.c_str(), output_topic_.c_str(), accept_float_ ? "true" : "false");
  }

private:
  void callback(const msg_interfaces::msg::Hcinspvatzcb::ConstSharedPtr msg)
  {
    // CHCNAV stat[1]: 4/8 fixed, 5/9 float. Other modes are not suitable
    // as centimetre-level position factors.
    const auto status = msg->stat[1];
    const bool fixed = status == 4 || status == 8;
    const bool floating = status == 5 || status == 9;
    if (!fixed && !(accept_float_ && floating)) {
      ++rejected_;
      if (rejected_ % 100 == 1) {
        RCLCPP_WARN(get_logger(), "Ignoring non-RTK solution: stat[1]=%u", status);
      }
      return;
    }
    if (msg->week == 0 || !std::isfinite(msg->second) ||
        !std::isfinite(msg->latitude) || !std::isfinite(msg->longitude)) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid CHCNAV time/position sample");
      return;
    }

    gnss_comm::msg::GnssPVTSolnMsg out;
    out.time.week = msg->week;
    out.time.tow = msg->second;
    out.fix_type = 3;
    out.valid_fix = true;
    out.diff_soln = true;
    out.carr_soln = fixed ? 2 : 1;
    out.num_sv = static_cast<uint8_t>(std::min<uint16_t>(msg->ns, 255));
    out.latitude = msg->latitude;
    out.longitude = msg->longitude;
    out.altitude = msg->altitude;
    out.height_msl = msg->altitude - msg->undulation;
    out.h_acc = std::max(msg->position_stdev[0], msg->position_stdev[1]);
    out.v_acc = msg->position_stdev[2];
    out.p_dop = msg->pdop;
    out.vel_e = msg->enu_velocity.x;
    out.vel_n = msg->enu_velocity.y;
    out.vel_d = -msg->enu_velocity.z;
    out.vel_acc = std::max({msg->enu_velocity_stdev[0],
                            msg->enu_velocity_stdev[1],
                            msg->enu_velocity_stdev[2]});
    // CHCNAV status 4 表示固定定位且双天线定向有效；status 8 只有固定位置。
    out.heading_valid = status == 4 && std::isfinite(msg->heading2) &&
                        msg->heading2 >= 0.0 && msg->heading2 <= 360.0 &&
                        std::isfinite(msg->euler_stdev[2]) && msg->euler_stdev[2] > 0.0;
    // heading2 明确定义为从北顺时针；ENU 数学 yaw 为从东逆时针，故 yaw=90°-heading2。
    constexpr double degrees_to_radians = 0.01745329251994329577;
    out.heading = (90.0 - msg->heading2) * degrees_to_radians;
    out.heading_acc = std::max(0.01 * degrees_to_radians,
                               msg->euler_stdev[2] * degrees_to_radians);
    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  bool accept_float_{false};
  size_t rejected_{0};
  rclcpp::Publisher<gnss_comm::msg::GnssPVTSolnMsg>::SharedPtr pub_;
  rclcpp::Subscription<msg_interfaces::msg::Hcinspvatzcb>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChcnavGnssAdapter>());
  rclcpp::shutdown();
  return 0;
}
