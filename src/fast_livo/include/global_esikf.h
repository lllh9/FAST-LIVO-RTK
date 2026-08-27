#pragma once

// 独立实时全局误差状态滤波器：不改变 FAST-LIVO2 的局部状态和局部地图。
#include <gtsam/geometry/Pose3.h>
#include <Eigen/Core>

#include <deque>
#include <mutex>

// 清洗并转换到 ENU 后的 RTK 观测，同时供实时 ESIKF 与 GTSAM 后端使用。
struct GlobalRtkObservation
{
  double stamp = 0.0;                                      // 修正时间偏移后的 Unix 时间戳。
  gtsam::Point3 position{0.0, 0.0, 0.0};                  // ENU 中的天线位置。
  Eigen::Vector3d position_variance = Eigen::Vector3d::Ones();  // ENU 位置方差。
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();      // ENU 天线速度。
  Eigen::Vector3d velocity_variance = Eigen::Vector3d::Ones();  // ENU 速度方差。
  double heading = 0.0;                                   // ENU 数学航向角，逆时针为正。
  double heading_variance = 1.0;                          // 航向角方差。
  bool velocity_valid = false;                            // 速度字段是否可用于更新。
  bool heading_valid = false;                             // 双天线航向是否可用于更新。
};

// 参数集中保存，便于 YAML 调整而不触碰滤波器数学实现。
struct GlobalEsikfConfig
{
  bool enabled = true;                                    // 是否发布实时全局 ESIKF 状态。
  double sync_tolerance = 0.10;                           // RTK 与 LIVO 时间匹配门限 [s]。
  double rotation_process_variance = 2.5e-5;              // LIVO 相对旋转过程方差。
  double position_process_variance = 2.5e-3;              // LIVO 相对平移过程方差。
  double velocity_process_variance = 2.5e-2;              // 差分速度过程方差。
  double position_nis_gate = 16.27;                       // 三维位置更新卡方门限。
  double velocity_nis_gate = 16.27;                       // 三维速度更新卡方门限。
  double heading_nis_gate = 9.0;                          // 单维航向更新卡方门限。
  double heading_offset = 0.0;                            // RTK 航向到 LIVO 机体航向的安装偏角 [rad]。
  Eigen::Vector3d imu_to_antenna = Eigen::Vector3d::Zero();  // IMU 到天线杆臂，IMU 系 [m]。
};

// 状态为姿态、IMU 位置和速度；九维误差为 [dtheta, dp, dv]。
class RealtimeGlobalEsikf
{
public:
  using Matrix9d = Eigen::Matrix<double, 9, 9>;

  // 保存配置并清空状态，保证参数重载后的初值一致。
  void configure(const GlobalEsikfConfig &config);

  // RTK 回调只入队；真正的更新在 LIVO 时刻执行，实现确定性的时间匹配。
  void pushRtk(const GlobalRtkObservation &observation);

  // 用纯 LIVO 相对位姿传播，在匹配时刻依次加入 RTK 位置、速度和航向更新。
  bool processLivo(double stamp,
                   const gtsam::Pose3 &local_pose,
                   gtsam::Pose3 &global_pose,
                   Eigen::Vector3d &global_velocity,
                   Matrix9d &covariance,
                   bool &rtk_updated);

  // 指示是否已由有效 RTK 建立 ENU 全局状态。
  bool globallyInitialized() const;

private:
  // 传播名义状态和九维误差协方差。
  void predict(const gtsam::Pose3 &local_relative, double dt);

  // 首个匹配 RTK 负责确定 ENU 平移和可用时的绝对航向。
  void initializeFromRtk(const GlobalRtkObservation &observation,
                         const gtsam::Pose3 &local_pose);

  // 三类量测使用独立鲁棒门控，某一类异常不会阻断其他有效量测。
  bool updatePosition(const GlobalRtkObservation &observation);
  bool updateVelocity(const GlobalRtkObservation &observation);
  bool updateHeading(const GlobalRtkObservation &observation);

  // 执行通用 Joseph 形式误差状态更新，保持协方差对称半正定。
  bool kalmanUpdate(const Eigen::VectorXd &innovation,
                    const Eigen::MatrixXd &jacobian,
                    const Eigen::MatrixXd &measurement_covariance,
                    double nis_gate);

  // 将九维小误差注入名义姿态、位置和速度，再把误差状态重置为零。
  void injectError(const Eigen::Matrix<double, 9, 1> &error);

  // 基础 SO(3) 与角度工具函数。
  static Eigen::Matrix3d skew(const Eigen::Vector3d &vector);
  static double wrapAngle(double angle);
  static double yawOf(const gtsam::Rot3 &rotation);
  static gtsam::Rot3 replaceYaw(const gtsam::Rot3 &rotation, double yaw);

  GlobalEsikfConfig config_;                              // 当前滤波参数。
  mutable std::mutex mutex_;                              // 保护 RTK 队列和滤波状态。
  std::deque<GlobalRtkObservation> rtk_queue_;            // 等待与 LIVO 匹配的 RTK 观测。
  gtsam::Pose3 last_local_pose_;                          // 上一帧纯 LIVO 位姿。
  gtsam::Pose3 global_pose_;                              // 当前 ENU 中的 IMU 位姿。
  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();    // 当前 ENU 速度。
  Eigen::Vector3d angular_velocity_body_ = Eigen::Vector3d::Zero();  // 最近 LIVO 角速度。
  Matrix9d covariance_ = Matrix9d::Identity();            // 九维误差协方差。
  double last_livo_stamp_ = 0.0;                         // 上一帧 LIVO 时间。
  bool have_livo_ = false;                               // 是否已收到第一帧 LIVO。
  bool globally_initialized_ = false;                    // 是否已有有效 RTK 全局锚点。
};
