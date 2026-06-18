/*
 * ros1_compat.hpp — a thin ROS 1 → ROS 2 compatibility layer for the
 * FAST-LIVO2-RTK port.
 *
 * The upstream code is ROS 1 (roscpp). Rather than rewrite every logging call
 * and every timestamp manipulation, this header provides drop-in shims so the
 * algorithm code stays byte-for-byte identical:
 *
 *   - ROS_INFO / ROS_WARN / ROS_ERROR / ROS_DEBUG / ROS_ASSERT  -> rclcpp
 *   - ros::Time / ros::Duration / ros::Rate                     -> light shims
 *   - toSec(builtin_interfaces::msg::Time)                       -> free helper
 *
 * Message *types*, publishers/subscribers, parameters, tf and the node object
 * itself are migrated explicitly in the source (this header deliberately does
 * NOT try to shim those — doing so hides real API differences).
 */
#ifndef FAST_LIVO_ROS1_COMPAT_HPP
#define FAST_LIVO_ROS1_COMPAT_HPP

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <builtin_interfaces/msg/duration.hpp>

#include <cassert>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <thread>
#include <algorithm>
#include <string>

// ---------------------------------------------------------------------------
// Logging: map the ROS 1 printf-style macros onto rclcpp with a shared logger.
// (All upstream ROS_* calls in this project are printf-style, not _STREAM.)
// ---------------------------------------------------------------------------
namespace fastlivo_compat {
inline rclcpp::Logger logger()
{
  static rclcpp::Logger lg = rclcpp::get_logger("fast_livo");
  return lg;
}
}  // namespace fastlivo_compat

#define ROS_INFO(...)  RCLCPP_INFO(fastlivo_compat::logger(), __VA_ARGS__)
#define ROS_WARN(...)  RCLCPP_WARN(fastlivo_compat::logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(fastlivo_compat::logger(), __VA_ARGS__)
#define ROS_DEBUG(...) RCLCPP_DEBUG(fastlivo_compat::logger(), __VA_ARGS__)
#define ROS_INFO_STREAM(args)  RCLCPP_INFO_STREAM(fastlivo_compat::logger(), args)
#define ROS_WARN_STREAM(args)  RCLCPP_WARN_STREAM(fastlivo_compat::logger(), args)
#define ROS_ERROR_STREAM(args) RCLCPP_ERROR_STREAM(fastlivo_compat::logger(), args)
#define ROS_ASSERT(cond) assert(cond)

// ---------------------------------------------------------------------------
// ros::Time / ros::Duration / ros::Rate shims.
//
// ros::Time is backed by a double (seconds), exactly like the upstream code
// treats it (everything goes through .toSec() / .fromSec()). Implicit
// conversion to/from builtin_interfaces::msg::Time lets message header stamps
// be assigned to/from these shims transparently.
// ---------------------------------------------------------------------------
namespace ros {

class Duration
{
public:
  double sec_{0.0};
  Duration() = default;
  Duration(double s) : sec_(s) {}                 // NOLINT(runtime/explicit)
  Duration(int32_t s, uint32_t nsec) : sec_(static_cast<double>(s) + nsec * 1e-9) {}
  double toSec() const { return sec_; }
  operator builtin_interfaces::msg::Duration() const
  {
    builtin_interfaces::msg::Duration d;
    d.sec = static_cast<int32_t>(std::floor(sec_));
    d.nanosec = static_cast<uint32_t>(std::llround((sec_ - static_cast<double>(d.sec)) * 1e9));
    if (d.nanosec >= 1000000000u) { d.sec += 1; d.nanosec -= 1000000000u; }
    return d;
  }
};

class Time
{
public:
  double sec_{0.0};
  Time() = default;
  Time(double s) : sec_(s) {}                      // NOLINT(runtime/explicit)
  Time(const builtin_interfaces::msg::Time & t) : sec_(t.sec + t.nanosec * 1e-9) {}  // NOLINT
  Time & fromSec(double s) { sec_ = s; return *this; }
  double toSec() const { return sec_; }
  double toNSec() const { return sec_ * 1e9; }

  static Time now()
  {
    const auto d = std::chrono::system_clock::now().time_since_epoch();
    return Time(std::chrono::duration<double>(d).count());
  }

  operator builtin_interfaces::msg::Time() const
  {
    builtin_interfaces::msg::Time t;
    double s = sec_ < 0.0 ? 0.0 : sec_;
    t.sec = static_cast<int32_t>(std::floor(s));
    t.nanosec = static_cast<uint32_t>(std::llround((s - static_cast<double>(t.sec)) * 1e9));
    if (t.nanosec >= 1000000000u) { t.sec += 1; t.nanosec -= 1000000000u; }
    return t;
  }

  Time operator+(const Duration & d) const { return Time(sec_ + d.sec_); }
  Time operator-(const Duration & d) const { return Time(sec_ - d.sec_); }
  Duration operator-(const Time & o) const { return Duration(sec_ - o.sec_); }
  bool operator<(const Time & o) const { return sec_ < o.sec_; }
  bool operator>(const Time & o) const { return sec_ > o.sec_; }
  bool operator<=(const Time & o) const { return sec_ <= o.sec_; }
  bool operator>=(const Time & o) const { return sec_ >= o.sec_; }
};

// Simple wall-clock rate limiter (no node / sim-time coupling needed; the
// upstream busy loops just want a steady cadence cap).
class Rate
{
public:
  explicit Rate(double hz) : period_(hz > 0.0 ? 1.0 / hz : 0.0),
                             last_(std::chrono::steady_clock::now()) {}
  void sleep()
  {
    const auto target = last_ + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(period_));
    const auto now = std::chrono::steady_clock::now();
    if (now < target) std::this_thread::sleep_for(target - now);
    last_ = std::chrono::steady_clock::now();
  }
private:
  double period_;
  std::chrono::steady_clock::time_point last_;
};

}  // namespace ros

// ---------------------------------------------------------------------------
// Free helper: extract seconds from a message header stamp.
// Upstream wrote `msg->header.stamp.toSec()`; the port rewrites those call
// sites to `toSec(msg->header.stamp)`.
// ---------------------------------------------------------------------------
inline double toSec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}
inline double toSec(const ros::Time & t) { return t.toSec(); }

// ---------------------------------------------------------------------------
// Parameter helper that mirrors ROS 1's nh.param<T>(name, var, default).
//
// The node is created with automatically_declare_parameters_from_overrides(true)
// so values from the loaded YAML are already declared; params absent from YAML
// are declared here with the supplied default. ROS 2 parameter names cannot
// contain '/', so the ROS 1 "group/key" names are mapped to "group.key".
// ---------------------------------------------------------------------------
namespace fastlivo_compat {
template <typename T>
void get_param(const rclcpp::Node::SharedPtr & node, const std::string & name, T & var, const T & def)
{
  std::string n = name;
  std::replace(n.begin(), n.end(), '/', '.');
  if (!node->has_parameter(n)) {
    node->declare_parameter<T>(n, def);
  }
  node->get_parameter(n, var);
}
}  // namespace fastlivo_compat

#endif  // FAST_LIVO_ROS1_COMPAT_HPP
