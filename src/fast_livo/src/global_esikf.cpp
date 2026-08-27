#include "global_esikf.h"

#include <gtsam/geometry/Rot3.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>

namespace
{
constexpr double kPi = 3.14159265358979323846;            // 不依赖平台 M_PI 定义。
}

void RealtimeGlobalEsikf::configure(const GlobalEsikfConfig &config)
{
  std::lock_guard<std::mutex> lock(mutex_);                // 配置与状态重置必须原子完成。
  config_ = config;
  rtk_queue_.clear();
  global_pose_ = gtsam::Pose3::Identity();
  velocity_.setZero();
  angular_velocity_body_.setZero();
  covariance_.setIdentity();
  covariance_.diagonal() << 0.25, 0.25, 1.0, 100.0, 100.0, 100.0, 4.0, 4.0, 4.0;
  last_livo_stamp_ = 0.0;
  have_livo_ = false;
  globally_initialized_ = false;
}

void RealtimeGlobalEsikf::pushRtk(const GlobalRtkObservation &observation)
{
  std::lock_guard<std::mutex> lock(mutex_);                // RTK 与高频 LIVO 回调可能来自不同线程。
  if (!std::isfinite(observation.stamp)) return;
  const auto insertion = std::upper_bound(
      rtk_queue_.begin(), rtk_queue_.end(), observation.stamp,
      [](double stamp, const GlobalRtkObservation &sample) { return stamp < sample.stamp; });
  rtk_queue_.insert(insertion, observation);               // 即使消息乱序，时间匹配仍按时间递增处理。
  while (rtk_queue_.size() > 2000) rtk_queue_.pop_front(); // 有界缓存避免时间异常导致内存增长。
}

bool RealtimeGlobalEsikf::processLivo(double stamp,
                                      const gtsam::Pose3 &local_pose,
                                      gtsam::Pose3 &global_pose,
                                      Eigen::Vector3d &global_velocity,
                                      Matrix9d &covariance,
                                      bool &rtk_updated)
{
  std::lock_guard<std::mutex> lock(mutex_);                // 一次传播与更新作为不可分割操作。
  rtk_updated = false;
  if (!config_.enabled || !std::isfinite(stamp)) return false;

  if (!have_livo_)
  {
    last_local_pose_ = local_pose;                         // 第一帧只建立相对里程计参考。
    global_pose_ = local_pose;
    last_livo_stamp_ = stamp;
    have_livo_ = true;
  }
  else
  {
    const double dt = stamp - last_livo_stamp_;
    if (dt <= 0.0 || dt > 1.0)                            // 时间回跳或长断流时重置增量参考。
    {
      last_local_pose_ = local_pose;
      last_livo_stamp_ = stamp;
    }
    else
    {
      const gtsam::Pose3 relative = last_local_pose_.between(local_pose);
      predict(relative, dt);                              // 只使用纯 LIVO 增量传播全局状态。
      last_local_pose_ = local_pose;
      last_livo_stamp_ = stamp;
    }
  }

  while (!rtk_queue_.empty() && rtk_queue_.front().stamp < stamp - config_.sync_tolerance)
    rtk_queue_.pop_front();                               // 丢弃已经不可能匹配的过期观测。

  auto best = rtk_queue_.end();                           // 在门限内选择时间差最小而非队首观测。
  double best_difference = config_.sync_tolerance;
  for (auto iterator = rtk_queue_.begin(); iterator != rtk_queue_.end(); ++iterator)
  {
    const double difference = std::abs(iterator->stamp - stamp);
    if (difference <= best_difference)
    {
      best = iterator;
      best_difference = difference;
    }
    if (iterator->stamp > stamp + config_.sync_tolerance) break;
  }

  if (best != rtk_queue_.end())
  {
    const GlobalRtkObservation observation = *best;
    rtk_queue_.erase(rtk_queue_.begin(), std::next(best)); // 每个 RTK 最多更新一次，避免重复计权。
    if (!globally_initialized_)
      initializeFromRtk(observation, local_pose);          // 首次 RTK 建立 ENU 锚点。
    else
    {
      const bool position_used = updatePosition(observation);
      const bool velocity_used = observation.velocity_valid && updateVelocity(observation);
      const bool heading_used = observation.heading_valid && updateHeading(observation);
      rtk_updated = position_used || velocity_used || heading_used;
    }
  }

  global_pose = global_pose_;                             // 返回不可变快照供 ROS 发布。
  global_velocity = velocity_;
  covariance = covariance_;
  return globally_initialized_;
}

bool RealtimeGlobalEsikf::globallyInitialized() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return globally_initialized_;
}

void RealtimeGlobalEsikf::predict(const gtsam::Pose3 &local_relative, double dt)
{
  const gtsam::Point3 old_position = global_pose_.translation();
  const Eigen::Matrix3d old_rotation = global_pose_.rotation().matrix();
  angular_velocity_body_ = gtsam::Rot3::Logmap(local_relative.rotation()) / dt;
  global_pose_ = global_pose_.compose(local_relative);    // 保留 LIVO 的高频短时运动约束。
  velocity_ = (global_pose_.translation() - old_position) / dt;

  Matrix9d transition = Matrix9d::Identity();             // 对 Pose3 复合运动线性化误差传播。
  transition.block<3, 3>(0, 0) = local_relative.rotation().matrix().transpose();
  transition.block<3, 3>(3, 0) = -old_rotation * skew(local_relative.translation());
  Matrix9d process_noise = Matrix9d::Zero();
  process_noise.block<3, 3>(0, 0).diagonal().setConstant(config_.rotation_process_variance * dt);
  process_noise.block<3, 3>(3, 3).diagonal().setConstant(config_.position_process_variance * dt);
  process_noise.block<3, 3>(6, 6).diagonal().setConstant(config_.velocity_process_variance * dt);
  covariance_ = transition * covariance_ * transition.transpose() + process_noise;
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

void RealtimeGlobalEsikf::initializeFromRtk(const GlobalRtkObservation &observation,
                                             const gtsam::Pose3 &local_pose)
{
  gtsam::Rot3 rotation = local_pose.rotation();           // 无航向时保持 FAST-LIVO2 当前姿态。
  if (observation.heading_valid)
    rotation = replaceYaw(rotation, wrapAngle(observation.heading + config_.heading_offset));
  const Eigen::Vector3d antenna = observation.position;
  const Eigen::Vector3d imu_position = antenna - rotation.matrix() * config_.imu_to_antenna;
  global_pose_ = gtsam::Pose3(rotation, gtsam::Point3(imu_position));
  // RTK 给出天线速度；扣除转动杆臂速度后才是 IMU 原点速度。
  if (observation.velocity_valid)
    velocity_ = observation.velocity -
                rotation.matrix() * angular_velocity_body_.cross(config_.imu_to_antenna);
  else
    velocity_.setZero();
  covariance_.setIdentity();
  covariance_.block<3, 3>(0, 0).diagonal().setConstant(
      observation.heading_valid ? observation.heading_variance : 1.0);
  covariance_.block<3, 3>(3, 3) = observation.position_variance.asDiagonal();
  if (observation.velocity_valid)
    covariance_.block<3, 3>(6, 6) = observation.velocity_variance.asDiagonal();
  else
    covariance_.block<3, 3>(6, 6) = 4.0 * Eigen::Matrix3d::Identity();
  globally_initialized_ = true;
}

bool RealtimeGlobalEsikf::updatePosition(const GlobalRtkObservation &observation)
{
  const Eigen::Matrix3d rotation = global_pose_.rotation().matrix();
  const Eigen::Vector3d predicted = global_pose_.translation() + rotation * config_.imu_to_antenna;
  Eigen::Matrix<double, 3, 9> jacobian = Eigen::Matrix<double, 3, 9>::Zero();
  jacobian.block<3, 3>(0, 0) = -rotation * skew(config_.imu_to_antenna);
  jacobian.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
  return kalmanUpdate(observation.position - predicted,
                      jacobian,
                      observation.position_variance.asDiagonal(),
                      config_.position_nis_gate);
}

bool RealtimeGlobalEsikf::updateVelocity(const GlobalRtkObservation &observation)
{
  const Eigen::Matrix3d rotation = global_pose_.rotation().matrix();
  const Eigen::Vector3d lever_velocity_body = angular_velocity_body_.cross(config_.imu_to_antenna);
  const Eigen::Vector3d predicted = velocity_ + rotation * lever_velocity_body;
  Eigen::Matrix<double, 3, 9> jacobian = Eigen::Matrix<double, 3, 9>::Zero();
  jacobian.block<3, 3>(0, 0) = -rotation * skew(lever_velocity_body);
  jacobian.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
  return kalmanUpdate(observation.velocity - predicted,
                      jacobian,
                      observation.velocity_variance.asDiagonal(),
                      config_.velocity_nis_gate);
}

bool RealtimeGlobalEsikf::updateHeading(const GlobalRtkObservation &observation)
{
  Eigen::Matrix<double, 1, 9> jacobian = Eigen::Matrix<double, 1, 9>::Zero();
  // 本滤波器采用右乘姿态误差；有 roll/pitch 时 yaw 对机体系三轴小角度均可能敏感。
  constexpr double derivative_step = 1e-6;
  const double nominal_yaw = yawOf(global_pose_.rotation());
  for (int axis = 0; axis < 3; ++axis)
  {
    Eigen::Vector3d perturbation = Eigen::Vector3d::Zero();
    perturbation(axis) = derivative_step;
    const gtsam::Rot3 perturbed = global_pose_.rotation().compose(
        gtsam::Rot3::Expmap(perturbation));
    jacobian(0, axis) = wrapAngle(yawOf(perturbed) - nominal_yaw) / derivative_step;
  }
  Eigen::VectorXd innovation(1);
  innovation(0) = wrapAngle(observation.heading + config_.heading_offset -
                            nominal_yaw);
  Eigen::Matrix<double, 1, 1> noise;
  noise(0, 0) = std::max(1e-6, observation.heading_variance);
  return kalmanUpdate(innovation, jacobian, noise, config_.heading_nis_gate);
}

bool RealtimeGlobalEsikf::kalmanUpdate(const Eigen::VectorXd &innovation,
                                       const Eigen::MatrixXd &jacobian,
                                       const Eigen::MatrixXd &measurement_covariance,
                                       double nis_gate)
{
  const Eigen::MatrixXd innovation_covariance =
      jacobian * covariance_ * jacobian.transpose() + measurement_covariance;
  const Eigen::LDLT<Eigen::MatrixXd> decomposition(innovation_covariance);
  if (decomposition.info() != Eigen::Success) return false;
  const double nis = innovation.dot(decomposition.solve(innovation));
  if (!std::isfinite(nis) || nis > nis_gate) return false; // 鲁棒门控拒绝跳点和错误航向。

  const Eigen::MatrixXd gain = covariance_ * jacobian.transpose() *
                               decomposition.solve(Eigen::MatrixXd::Identity(
                                   innovation.size(), innovation.size()));
  const Eigen::Matrix<double, 9, 1> error = gain * innovation;
  injectError(error);
  const Matrix9d identity = Matrix9d::Identity();
  const Matrix9d residual_projection = identity - gain * jacobian;
  covariance_ = residual_projection * covariance_ * residual_projection.transpose() +
                gain * measurement_covariance * gain.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  return true;
}

void RealtimeGlobalEsikf::injectError(const Eigen::Matrix<double, 9, 1> &error)
{
  const gtsam::Rot3 corrected_rotation =
      global_pose_.rotation().compose(gtsam::Rot3::Expmap(error.segment<3>(0)));
  const gtsam::Point3 corrected_position = global_pose_.translation() + error.segment<3>(3);
  global_pose_ = gtsam::Pose3(corrected_rotation, corrected_position);
  velocity_ += error.segment<3>(6);
}

Eigen::Matrix3d RealtimeGlobalEsikf::skew(const Eigen::Vector3d &vector)
{
  Eigen::Matrix3d matrix;
  matrix << 0.0, -vector.z(), vector.y(),
            vector.z(), 0.0, -vector.x(),
            -vector.y(), vector.x(), 0.0;
  return matrix;
}

double RealtimeGlobalEsikf::wrapAngle(double angle)
{
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

double RealtimeGlobalEsikf::yawOf(const gtsam::Rot3 &rotation)
{
  const Eigen::Matrix3d matrix = rotation.matrix();
  return std::atan2(matrix(1, 0), matrix(0, 0));
}

gtsam::Rot3 RealtimeGlobalEsikf::replaceYaw(const gtsam::Rot3 &rotation, double yaw)
{
  const Eigen::Matrix3d matrix = rotation.matrix();
  const double pitch = std::asin(std::clamp(-matrix(2, 0), -1.0, 1.0));
  const double roll = std::atan2(matrix(2, 1), matrix(2, 2));
  const Eigen::Matrix3d result =
      (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  return gtsam::Rot3(result);
}
