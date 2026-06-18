/*
 * tf_compat.hpp — minimal ROS 1 `tf::` shims backed by tf2, so the handful of
 * upstream quaternion/RPY helper calls compile unchanged. Include this only in
 * translation units that used tf:: (LIVMapper.cpp, optimization.cpp,
 * voxel_map.cpp). The TransformBroadcaster path (publish_mavros) is rewritten
 * directly against tf2_ros and does not rely on this header.
 */
#ifndef FAST_LIVO_TF_COMPAT_HPP
#define FAST_LIVO_TF_COMPAT_HPP

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace tf {

using Quaternion = tf2::Quaternion;
using Matrix3x3 = tf2::Matrix3x3;

inline geometry_msgs::msg::Quaternion createQuaternionMsgFromRollPitchYaw(double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::toMsg(q);
}

inline tf2::Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return q;
}

inline void quaternionMsgToTF(const geometry_msgs::msg::Quaternion & msg, tf2::Quaternion & q)
{
  tf2::fromMsg(msg, q);
}

}  // namespace tf

#endif  // FAST_LIVO_TF_COMPAT_HPP
