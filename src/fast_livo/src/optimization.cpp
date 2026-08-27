#include "optimization.h"

// GTSAM 鲁棒噪声模型用于降低偶发 RTK 粗差和错误回环的影响。
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/numericalDerivative.h>

// PCL GICP 结合局部协方差，用于对 Scan Context 回环候选进行几何验证。
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>

// ROS/PCL 转换及回环可视化消息。
#include <pcl_conversions/pcl_conversions.h>

// 文件、数值和格式化输出工具。
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>

namespace
{
// 数学常量集中定义，避免依赖平台是否暴露 M_PI。
constexpr double kPi = 3.14159265358979323846;

// 天线位置因子：预测值为 IMU 位姿加旋转后的 IMU→天线杆臂。
class AntennaPositionFactor final : public gtsam::NoiseModelFactorN<gtsam::Pose3>
{
public:
  // 构造因子并保存 ENU 天线位置及机体系杆臂。
  AntennaPositionFactor(gtsam::Key pose_key,
                        const gtsam::Point3 &measurement,
                        const gtsam::Point3 &lever_arm,
                        const gtsam::SharedNoiseModel &noise)
      : gtsam::NoiseModelFactorN<gtsam::Pose3>(noise, pose_key),
        measurement_(measurement),
        lever_arm_(lever_arm)
  {
  }

  // 返回“预测天线位置－RTK天线位置”残差，并给出对 Pose3 的解析雅可比。
  gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                              gtsam::Matrix *jacobian = nullptr) const override
  {
    gtsam::Matrix36 pose_jacobian;  // transformFrom 对六维位姿扰动的雅可比。
    const gtsam::Point3 predicted = pose.transformFrom(lever_arm_, pose_jacobian);
    if (jacobian) *jacobian = pose_jacobian;  // 将解析雅可比交给 GTSAM 线性化。
    return predicted - measurement_;          // GTSAM 约定残差为预测减测量。
  }

private:
  gtsam::Point3 measurement_;  // map/ENU 坐标系中的 RTK 天线位置。
  gtsam::Point3 lever_arm_;    // IMU 坐标系中的 IMU→RTK 天线杆臂。
};

// RTK 速度因子：用相邻关键帧天线位置差分约束 ENU 速度，不向图中引入额外速度变量。
class AntennaVelocityFactor final : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>
{
public:
  AntennaVelocityFactor(gtsam::Key previous_key,
                        gtsam::Key current_key,
                        const Eigen::Vector3d &velocity,
                        const gtsam::Point3 &lever_arm,
                        double dt,
                        const gtsam::SharedNoiseModel &noise)
      : gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>(noise, previous_key, current_key),
        velocity_(velocity), lever_arm_(lever_arm), inverse_dt_(1.0 / dt)
  {
  }

  gtsam::Vector evaluateError(const gtsam::Pose3 &previous_pose,
                              const gtsam::Pose3 &current_pose,
                              gtsam::Matrix *previous_jacobian = nullptr,
                              gtsam::Matrix *current_jacobian = nullptr) const override
  {
    gtsam::Matrix36 previous_antenna_jacobian;
    gtsam::Matrix36 current_antenna_jacobian;
    const gtsam::Point3 previous_antenna = previous_pose.transformFrom(lever_arm_, previous_antenna_jacobian);
    const gtsam::Point3 current_antenna = current_pose.transformFrom(lever_arm_, current_antenna_jacobian);
    if (previous_jacobian) *previous_jacobian = -inverse_dt_ * previous_antenna_jacobian;
    if (current_jacobian) *current_jacobian = inverse_dt_ * current_antenna_jacobian;
    return (current_antenna - previous_antenna) * inverse_dt_ - velocity_;
  }

private:
  Eigen::Vector3d velocity_;                              // 接收机给出的 ENU 天线速度。
  gtsam::Point3 lever_arm_;                              // IMU 到定位天线的杆臂。
  double inverse_dt_ = 1.0;                              // 相邻关键帧时间间隔的倒数。
};

// 双天线航向因子：直接约束 Pose3 的 ENU yaw，安装偏角已在测量侧补偿。
class HeadingFactor final : public gtsam::NoiseModelFactorN<gtsam::Pose3>
{
public:
  HeadingFactor(gtsam::Key pose_key, double heading, const gtsam::SharedNoiseModel &noise)
      : gtsam::NoiseModelFactorN<gtsam::Pose3>(noise, pose_key), heading_(heading)
  {
  }

  gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                              gtsam::Matrix *jacobian = nullptr) const override
  {
    const auto error_function = [this](const gtsam::Pose3 &candidate) {
      gtsam::Vector1 residual;
      residual << wrap(yaw(candidate.rotation()) - heading_);
      return residual;
    };
    if (jacobian) *jacobian = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(error_function, pose);
    return error_function(pose);
  }

private:
  static double yaw(const gtsam::Rot3 &rotation)
  {
    const Eigen::Matrix3d matrix = rotation.matrix();
    return std::atan2(matrix(1, 0), matrix(0, 0));
  }

  static double wrap(double angle)
  {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
  }

  double heading_ = 0.0;                                 // 补偿安装角后的 ENU 航向。
};

// 将 GTSAM Pose3 变换矩阵转换为 PCL ICP 所需的单精度矩阵。
Eigen::Matrix4f poseToMatrix4f(const gtsam::Pose3 &pose)
{
  return pose.matrix().cast<float>();
}

// 将 PCL ICP 输出的齐次矩阵转换为 GTSAM Pose3。
gtsam::Pose3 matrix4fToPose(const Eigen::Matrix4f &matrix)
{
  const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0).cast<double>();
  const Eigen::Vector3d translation = matrix.block<3, 1>(0, 3).cast<double>();
  return gtsam::Pose3(gtsam::Rot3(rotation), gtsam::Point3(translation));
}

// 保留 FAST-LIVO2 的 roll/pitch，只用双天线绝对航向替换 yaw 作为图优化初值。
gtsam::Rot3 rotationWithYaw(const gtsam::Rot3 &rotation, double yaw)
{
  const Eigen::Matrix3d matrix = rotation.matrix();
  const double pitch = std::asin(std::clamp(-matrix(2, 0), -1.0, 1.0));
  const double roll = std::atan2(matrix(2, 1), matrix(2, 2));
  return gtsam::Rot3((Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix());
}
}  // namespace

optimization::optimization(const rclcpp::Node::SharedPtr &node)
    : node_(node),
      smoothed_map_to_odom_(gtsam::Pose3::Identity()),
      target_map_to_odom_(gtsam::Pose3::Identity())
{
  // 读取输出路径，并提前建立最终轨迹、地图和调试目录。
  fastlivo_compat::get_param<std::string>(node_, "laserMapping/outputfilepath", output_directory_, "output");
  std::error_code directory_error;  // 使用 error_code，避免目录问题抛异常终止定位。
  for (const auto &subdirectory : {"/TUM", "/global_pcd", "/scan_pcd", "/debug"})
    std::filesystem::create_directories(output_directory_ + subdirectory, directory_error);

  // 读取 RTK 输入格式、时间偏移、精度门限和 IMU→天线杆臂。
  fastlivo_compat::get_param<std::string>(node_, "gps/gps_topic", gps_topic_, "/ublox_driver/receiver_pvt");
  fastlivo_compat::get_param<std::string>(node_, "gps/message_type", gps_message_type_, "pvt");
  fastlivo_compat::get_param<bool>(node_, "gps/gps_en", gps_enabled_, true);
  fastlivo_compat::get_param<double>(node_, "gps/gps_time_offset", gps_time_offset_, 0.0);
  fastlivo_compat::get_param<double>(node_, "backend/rtk_sync_tolerance", rtk_sync_tolerance_, 0.10);
  fastlivo_compat::get_param<double>(node_, "backend/rtk_max_horizontal_std", rtk_max_horizontal_std_, 0.50);
  fastlivo_compat::get_param<double>(node_, "backend/rtk_max_vertical_std", rtk_max_vertical_std_, 1.00);
  std::vector<double> lever_arm;  // YAML 数组先读入标准容器，再校验长度。
  fastlivo_compat::get_param<std::vector<double>>(node_, "gps/extrinsic_T", lever_arm, std::vector<double>{0.0, 0.0, 0.0});
  if (lever_arm.size() != 3) throw std::invalid_argument("gps.extrinsic_T must contain exactly three values");
  imu_to_antenna_ = gtsam::Point3(lever_arm[0], lever_arm[1], lever_arm[2]);

  // 实时全局 ESIKF 与后端共享 RTK 时间门限和杆臂，但拥有独立过程噪声及门控参数。
  GlobalEsikfConfig esikf_config;
  fastlivo_compat::get_param<bool>(node_, "realtime_esikf/enabled", esikf_config.enabled, true);
  esikf_config.sync_tolerance = rtk_sync_tolerance_;
  esikf_config.imu_to_antenna = Eigen::Vector3d(lever_arm[0], lever_arm[1], lever_arm[2]);
  fastlivo_compat::get_param<double>(node_, "realtime_esikf/rotation_process_variance", esikf_config.rotation_process_variance, 2.5e-5);
  fastlivo_compat::get_param<double>(node_, "realtime_esikf/position_process_variance", esikf_config.position_process_variance, 2.5e-3);
  fastlivo_compat::get_param<double>(node_, "realtime_esikf/velocity_process_variance", esikf_config.velocity_process_variance, 2.5e-2);
  fastlivo_compat::get_param<double>(node_, "realtime_esikf/position_nis_gate", esikf_config.position_nis_gate, 16.27);
  fastlivo_compat::get_param<double>(node_, "realtime_esikf/velocity_nis_gate", esikf_config.velocity_nis_gate, 16.27);
  fastlivo_compat::get_param<double>(node_, "realtime_esikf/heading_nis_gate", esikf_config.heading_nis_gate, 9.0);
  double heading_offset_degrees = 0.0;
  fastlivo_compat::get_param<double>(node_, "realtime_esikf/heading_offset_deg", heading_offset_degrees, 0.0);
  esikf_config.heading_offset = heading_offset_degrees * kPi / 180.0;
  rtk_heading_offset_ = esikf_config.heading_offset;
  realtime_global_esikf_.configure(esikf_config);

  // 读取关键帧阈值和 FAST-LIVO2 相对位姿因子方差。
  fastlivo_compat::get_param<double>(node_, "backend/keyframe_distance", keyframe_distance_, 0.50);
  fastlivo_compat::get_param<double>(node_, "backend/keyframe_angle", keyframe_angle_, 0.10);
  fastlivo_compat::get_param<double>(node_, "backend/keyframe_time", keyframe_time_, 1.00);
  fastlivo_compat::get_param<double>(node_, "backend/keyframe_sync_tolerance", keyframe_sync_tolerance_, 0.03);
  fastlivo_compat::get_param<double>(node_, "backend/odom_rotation_variance", odom_rotation_variance_, 1e-5);
  fastlivo_compat::get_param<double>(node_, "backend/odom_translation_variance", odom_translation_variance_, 1e-3);
  int max_pending_keyframes = static_cast<int>(max_pending_keyframes_);
  fastlivo_compat::get_param<int>(node_, "backend/max_pending_keyframes", max_pending_keyframes, 30);
  max_pending_keyframes_ = static_cast<size_t>(std::max(1, max_pending_keyframes));

  // 读取 Scan Context 粗检索和 GICP 几何验证参数。
  fastlivo_compat::get_param<bool>(node_, "backend/loop_enabled", loop_enabled_, true);
  fastlivo_compat::get_param<int>(node_, "backend/sc_rings", sc_rings_, 20);
  fastlivo_compat::get_param<int>(node_, "backend/sc_sectors", sc_sectors_, 60);
  fastlivo_compat::get_param<double>(node_, "backend/sc_max_radius", sc_max_radius_, 80.0);
  fastlivo_compat::get_param<double>(node_, "backend/sc_sensor_height", sc_sensor_height_, 2.0);
  int exclude_recent = static_cast<int>(sc_exclude_recent_);
  fastlivo_compat::get_param<int>(node_, "backend/sc_exclude_recent", exclude_recent, 30);
  sc_exclude_recent_ = static_cast<size_t>(std::max(1, exclude_recent));
  fastlivo_compat::get_param<double>(node_, "backend/sc_distance_threshold", sc_distance_threshold_, 0.18);
  fastlivo_compat::get_param<double>(node_, "backend/gicp_voxel_size", gicp_voxel_size_, 0.30);
  fastlivo_compat::get_param<double>(node_, "backend/gicp_max_correspondence", gicp_max_correspondence_, 2.0);
  fastlivo_compat::get_param<double>(node_, "backend/gicp_fitness_threshold", gicp_fitness_threshold_, 0.30);
  fastlivo_compat::get_param<int>(node_, "backend/gicp_max_iterations", gicp_max_iterations_, 50);
  fastlivo_compat::get_param<int>(node_, "backend/gicp_correspondence_randomness", gicp_correspondence_randomness_, 20);
  fastlivo_compat::get_param<double>(node_, "backend/gicp_transformation_epsilon", gicp_transformation_epsilon_, 1e-6);
  fastlivo_compat::get_param<double>(node_, "backend/gicp_rotation_epsilon", gicp_rotation_epsilon_, 1e-6);
  fastlivo_compat::get_param<double>(node_, "backend/loop_rotation_variance", loop_rotation_variance_, 1e-3);
  fastlivo_compat::get_param<double>(node_, "backend/loop_translation_variance", loop_translation_variance_, 5e-2);

  // 读取全局修正平滑系数和最终地图体素尺寸。
  fastlivo_compat::get_param<double>(node_, "backend/correction_smoothing_alpha", correction_smoothing_alpha_, 0.05);
  fastlivo_compat::get_param<double>(node_, "backend/map_voxel_size", map_voxel_size_, 0.10);
  correction_smoothing_alpha_ = std::clamp(correction_smoothing_alpha_, 0.001, 1.0);

  // 配置 iSAM2 每个关键帧都检查重线性化，保证 RTK/回环约束及时生效。
  gtsam::ISAM2Params isam_parameters;
  isam_parameters.relinearizeSkip = 1;
  isam_parameters.setRelinearizeThreshold(0.01);
  isam_ = std::make_unique<gtsam::ISAM2>(isam_parameters);

  // 使用较大队列同步 FAST-LIVO2 局部位姿和关键帧点云。
  const auto sync_qos = rclcpp::QoS(rclcpp::KeepLast(2000)).get_rmw_qos_profile();
  keyframe_odom_sub_.subscribe(node_, "/odometry/fast_livo2", sync_qos);
  keyframe_cloud_sub_.subscribe(node_, "/synced_cloud", sync_qos);
  SyncPolicy sync_policy(200);
  sync_policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(keyframe_sync_tolerance_));
  const SyncPolicy configured_sync_policy(sync_policy);    // const 强制选择“策略+两个订阅器”构造函数。
  keyframe_sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      configured_sync_policy, keyframe_odom_sub_, keyframe_cloud_sub_);
  keyframe_sync_->registerCallback(std::bind(&optimization::syncedCallback, this, std::placeholders::_1, std::placeholders::_2));

  // 单独接收所有局部里程计，使全局定位发布频率不受后端关键帧频率限制。
  local_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/fast_livo2", rclcpp::QoS(rclcpp::KeepLast(2000)),
      std::bind(&optimization::localOdometryCallback, this, std::placeholders::_1));

  // 根据配置只创建一种 RTK 订阅，避免同一设备数据重复进入因子图。
  if (gps_enabled_ && gps_message_type_ == "navsatfix")
  {
    navsat_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic_, rclcpp::QoS(rclcpp::KeepLast(2000)),
        std::bind(&optimization::navSatFixHandler, this, std::placeholders::_1));
  }
  else if (gps_enabled_)
  {
    pvt_sub_ = node_->create_subscription<gnss_comm::msg::GnssPVTSolnMsg>(
        gps_topic_, rclcpp::QoS(rclcpp::KeepLast(2000)),
        std::bind(&optimization::gpsHandler, this, std::placeholders::_1));
  }

  // 实时话题由 ESIKF 输出；图优化平滑结果另设话题，避免混淆两个估计器。
  global_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odometry/fast_livo2_global", 2000);
  graph_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odometry/fast_livo2_graph", 2000);
  rtk_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/path_rtk", 10);
  realtime_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/path_global_esikf", 10);
  global_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/path_global", 10);
  optimized_keyposes_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/global_backend/keyposes", 10);
  global_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/global_backend/final_map", 1);
  loop_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/global_backend/loop_edges", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  // 创建异步最终地图重建服务；调用不会阻塞 FAST-LIVO2 前端。
  save_map_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/global_backend/save_map",
      std::bind(&optimization::saveMapService, this, std::placeholders::_1, std::placeholders::_2));

  // 初始化所有对比路径的 ENU 坐标系并启动独立后端线程。
  rtk_path_.header.frame_id = "map";
  realtime_path_.header.frame_id = "map";
  global_path_.header.frame_id = "map";
  backend_thread_ = std::thread(&optimization::backendWorker, this);
  RCLCPP_INFO(node_->get_logger(),
              "Global fusion: real-time ESIKF + online iSAM2/RTK/Scan Context/GICP; map service=/global_backend/save_map");
}

optimization::~optimization()
{
  // 请求线程停止，并唤醒可能正在等待关键帧的条件变量。
  stop_requested_.store(true);
  queue_cv_.notify_all();

  // 等待队列中已有关键帧处理完毕，保证最终地图包含完整轨迹。
  if (backend_thread_.joinable()) backend_thread_.join();

  // 正常退出时自动执行一次离线式最终地图重建，不依赖键盘回车。
  reconstructFinalMap();
}

void optimization::syncedCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg)
{
  BackendKeyframeInput input;                           // 先只读取时间和位姿，暂不复制大点云。
  input.stamp = rclcpp::Time(cloud_msg->header.stamp).seconds();
  input.local_pose = poseMsgToGtsam(odom_msg->pose.pose);

  // 在 ROS 回调侧先执行相同的位移/转角/时间筛选，避免每个高频帧都做 PointCloud2→PCL
  // 转换再交给后端丢弃；后端仍保留二次判断以防队列丢帧或时序异常。
  {
    std::lock_guard<std::mutex> lock(keyframe_selection_mutex_);
    if (have_enqueued_keyframe_)
    {
      const gtsam::Pose3 relative = last_enqueued_local_pose_.between(input.local_pose);
      const double distance = relative.translation().norm();
      const double angle = gtsam::Rot3::Logmap(relative.rotation()).norm();
      const double elapsed = input.stamp - last_enqueued_stamp_;
      if (distance < keyframe_distance_ && angle < keyframe_angle_ && elapsed < keyframe_time_)
        return;
    }
    if (static_cast<size_t>(cloud_msg->width) * cloud_msg->height < 100) return;
    last_enqueued_local_pose_ = input.local_pose;
    last_enqueued_stamp_ = input.stamp;
    have_enqueued_keyframe_ = true;
  }

  input.cloud.reset(new PointCloudXYZRGB());            // 后端拥有独立点云，避免消息生命周期问题。
  pcl::fromROSMsg(*cloud_msg, *input.cloud);             // 将 ROS PointCloud2 转换为 PCL 点云。
  if (input.cloud->size() < 100) return;                // 过小点云无法形成稳定描述子或 ICP 约束。

  std::lock_guard<std::mutex> lock(queue_mutex_);       // 队列操作保持极短临界区。
  if (keyframe_queue_.size() >= max_pending_keyframes_)
  {
    keyframe_queue_.pop_front();                        // 后端落后时丢最旧待处理帧，保护实时性和内存。
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "Global backend queue full; dropping oldest pending frame");
  }
  keyframe_queue_.push_back(std::move(input));          // 将点云所有权移动到工作队列。
  queue_cv_.notify_one();                               // 唤醒独立后端线程。
}

void optimization::localOdometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
{
  const double stamp = rclcpp::Time(odom_msg->header.stamp).seconds();
  const gtsam::Pose3 local_pose = poseMsgToGtsam(odom_msg->pose.pose);  // 始终是未融合 RTK 的纯 LIVO 位姿。

  gtsam::Pose3 esikf_pose;                               // 实时全局 ESIKF 的名义位姿。
  Eigen::Vector3d esikf_velocity;                        // 实时全局 ESIKF 的 ENU 速度。
  RealtimeGlobalEsikf::Matrix9d esikf_covariance;        // [姿态、位置、速度] 协方差。
  bool rtk_updated = false;
  const bool global_valid = realtime_global_esikf_.processLivo(
      stamp, local_pose, esikf_pose, esikf_velocity, esikf_covariance, rtk_updated);
  (void)rtk_updated;                                      // 预留给后续诊断状态话题。
  if (global_valid)
  {
    nav_msgs::msg::Odometry global_odom = *odom_msg;
    global_odom.header.frame_id = "map";                  // GTSAM map 与实时滤波器使用同一 ENU 原点。
    global_odom.child_frame_id = "body_global_esikf";
    gtsamToPoseMsg(esikf_pose, global_odom.pose.pose);
    // nav_msgs/Odometry 约定 twist 位于 child_frame_id，因此将内部 ENU 速度转到机体系。
    const Eigen::Matrix3d global_to_body = esikf_pose.rotation().matrix().transpose();
    const Eigen::Vector3d body_velocity = global_to_body * esikf_velocity;
    global_odom.twist.twist.linear.x = body_velocity.x();
    global_odom.twist.twist.linear.y = body_velocity.y();
    global_odom.twist.twist.linear.z = body_velocity.z();
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> ros_covariance(global_odom.pose.covariance.data());
    ros_covariance.setZero();
    ros_covariance.block<3, 3>(0, 0) = esikf_covariance.block<3, 3>(3, 3);
    ros_covariance.block<3, 3>(3, 3) = esikf_covariance.block<3, 3>(0, 0);
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> twist_covariance(global_odom.twist.covariance.data());
    twist_covariance.setZero();
    twist_covariance.block<3, 3>(0, 0) = global_to_body *
        esikf_covariance.block<3, 3>(6, 6) * global_to_body.transpose();
    global_odom_pub_->publish(global_odom);               // 高频输出不等待 GTSAM 关键帧优化。

    if (last_realtime_path_stamp_ == 0.0 || stamp - last_realtime_path_stamp_ >= 0.10)
    {
      geometry_msgs::msg::PoseStamped realtime_pose;
      realtime_pose.header = global_odom.header;
      realtime_pose.pose = global_odom.pose.pose;
      realtime_path_.header.stamp = global_odom.header.stamp;
      realtime_path_.poses.push_back(realtime_pose);
      realtime_path_pub_->publish(realtime_path_);        // 10 Hz 连续曲线用于 RViz 与离线对比。
      last_realtime_path_stamp_ = stamp;
    }
  }

  const gtsam::Pose3 correction = smoothCorrectionStep();               // 平滑逼近最新图优化修正。
  const gtsam::Pose3 graph_pose = correction.compose(local_pose);        // GTSAM 历史最优约束投影到当前帧。
  nav_msgs::msg::Odometry graph_odom = *odom_msg;
  graph_odom.header.frame_id = "map";
  graph_odom.child_frame_id = "body_global_graph";
  gtsamToPoseMsg(graph_pose, graph_odom.pose.pose);
  graph_odom_pub_->publish(graph_odom);                   // 图优化结果供建图和低频全局校正使用。

  geometry_msgs::msg::TransformStamped map_to_odom;      // 广播 map→camera_init 修正。
  map_to_odom.header.stamp = odom_msg->header.stamp;
  map_to_odom.header.frame_id = "map";
  map_to_odom.child_frame_id = "camera_init";
  const gtsam::Point3 translation = correction.translation();
  const Eigen::Quaterniond quaternion = correction.rotation().toQuaternion();
  map_to_odom.transform.translation.x = translation.x();
  map_to_odom.transform.translation.y = translation.y();
  map_to_odom.transform.translation.z = translation.z();
  map_to_odom.transform.rotation.x = quaternion.x();
  map_to_odom.transform.rotation.y = quaternion.y();
  map_to_odom.transform.rotation.z = quaternion.z();
  map_to_odom.transform.rotation.w = quaternion.w();
  tf_broadcaster_->sendTransform(map_to_odom);           // RViz 和下游模块可直接使用 map 坐标系。
}

void optimization::gpsHandler(const gnss_comm::msg::GnssPVTSolnMsg::ConstSharedPtr &gps_msg)
{
  // 只接受固定解，并检查精度字段为有限数；浮点解不作为厘米级全局因子。
  if (!gps_msg->valid_fix || !gps_msg->diff_soln || gps_msg->carr_soln != 2 ||
      gps_msg->time.week == 0 || !std::isfinite(gps_msg->time.tow) ||
      gps_msg->time.tow < 0.0 || gps_msg->time.tow >= 604800.0 ||
      !std::isfinite(gps_msg->latitude) || !std::isfinite(gps_msg->longitude) ||
      !std::isfinite(gps_msg->altitude) || std::abs(gps_msg->latitude) > 90.0 ||
      std::abs(gps_msg->longitude) > 180.0 ||
      !std::isfinite(gps_msg->h_acc) || !std::isfinite(gps_msg->v_acc) ||
      gps_msg->h_acc <= 0.0 || gps_msg->v_acc <= 0.0 ||
      gps_msg->h_acc > rtk_max_horizontal_std_ || gps_msg->v_acc > rtk_max_vertical_std_)
    return;

  // 将 GPS 周和周内秒转换为 Unix/ROS 时间，并应用传感器时间偏移。
  constexpr double gps_epoch_unix = 315964800.0;
  constexpr double seconds_per_week = 604800.0;
  constexpr double leap_seconds = 18.0;
  const double stamp = gps_epoch_unix + gps_msg->time.week * seconds_per_week +
                       gps_msg->time.tow - leap_seconds - gps_time_offset_;

  BackendRtkMeasurement measurement;                    // 创建后端 RTK 观测。
  measurement.stamp = stamp;
  double east = 0.0, north = 0.0, up = 0.0;
  {
    std::lock_guard<std::mutex> lock(enu_mutex_);
    if (!enu_origin_initialized_)
    {
      local_cartesian_.Reset(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
      enu_origin_lla_ = Eigen::Vector3d(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
      enu_origin_initialized_ = true;
    }
    local_cartesian_.Forward(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude,
                             east, north, up);
  }
  measurement.position = gtsam::Point3(east, north, up);
  measurement.position_variance = Eigen::Vector3d(
      std::max(1e-4, gps_msg->h_acc * gps_msg->h_acc),
      std::max(1e-4, gps_msg->h_acc * gps_msg->h_acc),
      std::max(2.5e-3, gps_msg->v_acc * gps_msg->v_acc));
  measurement.velocity = Eigen::Vector3d(gps_msg->vel_e, gps_msg->vel_n, -gps_msg->vel_d);
  measurement.velocity_valid = measurement.velocity.allFinite() &&
                               std::isfinite(gps_msg->vel_acc) && gps_msg->vel_acc > 0.0;
  const double velocity_std = std::max(0.02, gps_msg->vel_acc);
  measurement.velocity_variance.setConstant(velocity_std * velocity_std);
  measurement.heading_valid = gps_msg->heading_valid && std::isfinite(gps_msg->heading) &&
                              std::isfinite(gps_msg->heading_acc) && gps_msg->heading_acc > 0.0;
  measurement.heading = gps_msg->heading;
  measurement.heading_variance = std::max(1e-6, gps_msg->heading_acc * gps_msg->heading_acc);

  publishRtkPath(measurement);                            // 仅显示已通过上述质量门限的 RTK。
  realtime_global_esikf_.pushRtk(measurement);           // 实时滤波器保留独立副本并按 LIVO 时刻消费。
  {
    std::lock_guard<std::mutex> lock(rtk_mutex_);        // GTSAM 后端拥有自己的 RTK 时间队列。
    const auto insertion = std::upper_bound(
        rtk_queue_.begin(), rtk_queue_.end(), measurement.stamp,
        [](double stamp_value, const BackendRtkMeasurement &sample) {
          return stamp_value < sample.stamp;
        });
    rtk_queue_.insert(insertion, measurement);           // 接收乱序时仍保持队列有序。
    while (rtk_queue_.size() > 10000) rtk_queue_.pop_front();
  }
}

void optimization::navSatFixHandler(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &gps_msg)
{
  // 标准消息必须有有效状态、有限经纬高和有效协方差。
  if (gps_msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX ||
      !std::isfinite(gps_msg->latitude) || !std::isfinite(gps_msg->longitude) ||
      !std::isfinite(gps_msg->altitude) || std::abs(gps_msg->latitude) > 90.0 ||
      std::abs(gps_msg->longitude) > 180.0 ||
      !std::isfinite(gps_msg->position_covariance[0]) ||
      !std::isfinite(gps_msg->position_covariance[4]) ||
      !std::isfinite(gps_msg->position_covariance[8]) ||
      gps_msg->position_covariance[0] < 0.0 ||
      gps_msg->position_covariance[4] < 0.0 ||
      gps_msg->position_covariance[8] < 0.0)
    return;

  const double horizontal_std = std::sqrt(std::max(0.0, std::max(
      gps_msg->position_covariance[0], gps_msg->position_covariance[4])));
  const double vertical_std = std::sqrt(std::max(0.0, gps_msg->position_covariance[8]));
  if (horizontal_std > rtk_max_horizontal_std_ || vertical_std > rtk_max_vertical_std_) return;

  BackendRtkMeasurement measurement;                    // 构造带方差的 ENU 位置测量。
  measurement.stamp = rclcpp::Time(gps_msg->header.stamp).seconds() - gps_time_offset_;
  double east = 0.0, north = 0.0, up = 0.0;
  {
    std::lock_guard<std::mutex> lock(enu_mutex_);
    if (!enu_origin_initialized_)
    {
      local_cartesian_.Reset(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
      enu_origin_lla_ = Eigen::Vector3d(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
      enu_origin_initialized_ = true;
    }
    local_cartesian_.Forward(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude,
                             east, north, up);
  }
  measurement.position = gtsam::Point3(east, north, up);
  measurement.position_variance = Eigen::Vector3d(
      std::max(1e-4, gps_msg->position_covariance[0]),
      std::max(1e-4, gps_msg->position_covariance[4]),
      std::max(2.5e-3, gps_msg->position_covariance[8]));
  publishRtkPath(measurement);                            // NavSatFix 也输出通过门限的 ENU 轨迹。
  realtime_global_esikf_.pushRtk(measurement);           // NavSatFix 只提供位置更新。
  {
    std::lock_guard<std::mutex> lock(rtk_mutex_);
    const auto insertion = std::upper_bound(
        rtk_queue_.begin(), rtk_queue_.end(), measurement.stamp,
        [](double stamp_value, const BackendRtkMeasurement &sample) {
          return stamp_value < sample.stamp;
        });
    rtk_queue_.insert(insertion, measurement);
    while (rtk_queue_.size() > 10000) rtk_queue_.pop_front();
  }
}

void optimization::publishRtkPath(const BackendRtkMeasurement &measurement)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = ros::Time().fromSec(measurement.stamp);
  pose.pose.position.x = measurement.position.x();
  pose.pose.position.y = measurement.position.y();
  pose.pose.position.z = measurement.position.z();
  pose.pose.orientation.w = 1.0;                         // Path 比较只使用 RTK 天线位置。

  rtk_path_.header.stamp = pose.header.stamp;
  rtk_path_.poses.push_back(pose);
  rtk_path_pub_->publish(rtk_path_);
}

void optimization::backendWorker()
{
  while (true)
  {
    BackendKeyframeInput input;                          // 每轮最多取出一个关键帧。
    bool has_input = false;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] {
        return stop_requested_.load() || save_requested_.load() || !keyframe_queue_.empty();
      });
      if (!keyframe_queue_.empty())
      {
        input = std::move(keyframe_queue_.front());      // 移动点云，避免大对象复制。
        keyframe_queue_.pop_front();
        has_input = true;
      }
      else if (stop_requested_.load())
      {
        break;                                           // 队列排空后安全停止。
      }
    }

    if (has_input) processKeyframe(input);               // 所有耗时后端计算均在此线程完成。
    if (save_requested_.exchange(false)) reconstructFinalMap();  // 异步执行服务提交的保存任务。
  }
}

bool optimization::shouldCreateKeyframe(const gtsam::Pose3 &pose, double stamp) const
{
  if (local_keyposes_.empty()) return true;              // 第一帧必须成为图节点。
  const gtsam::Pose3 relative = local_keyposes_.back().between(pose);
  const double distance = relative.translation().norm(); // 平移增量。
  const double angle = gtsam::Rot3::Logmap(relative.rotation()).norm();  // 旋转增量。
  const double elapsed = stamp - keyframe_stamps_.back(); // 与上一关键帧时间间隔。
  return distance >= keyframe_distance_ || angle >= keyframe_angle_ || elapsed >= keyframe_time_;
}

void optimization::processKeyframe(const BackendKeyframeInput &input)
{
  if (!shouldCreateKeyframe(input.local_pose, input.stamp)) return;  // 后端关键帧降采样。

  const size_t index = local_keyposes_.size();              // 当前节点编号连续递增。
  const ScanContextEntry scan_context = makeScanContext(input.cloud);  // 当前帧场景描述子。
  gtsam::NonlinearFactorGraph new_factors;                   // 本轮新增因子集合。
  gtsam::Values new_values;                                  // 本轮新增变量初值。
  BackendRtkMeasurement rtk_measurement;                     // 与当前关键帧匹配的 RTK。
  const bool has_rtk = gps_enabled_ && findNearestRtk(input.stamp, rtk_measurement);

  gtsam::Pose3 initial_global_pose = input.local_pose;       // 无历史时先使用局部前端位姿。
  if (index == 0)
  {
    if (has_rtk)
    {
      // 航向有效时先修正 yaw，再用同一姿态进行杆臂补偿，保证初值内部一致。
      const gtsam::Rot3 initial_rotation = rtk_measurement.heading_valid
          ? rotationWithYaw(input.local_pose.rotation(),
                            rtk_measurement.heading + rtk_heading_offset_)
          : input.local_pose.rotation();
      const gtsam::Point3 rotated_lever = initial_rotation.rotate(imu_to_antenna_);
      initial_global_pose = gtsam::Pose3(
          initial_rotation, rtk_measurement.position - rotated_lever);
    }

    // roll/pitch 有较强先验，yaw 和平移保持宽松，让 RTK 轨迹决定全局航向与原点。
    const gtsam::Vector6 prior_variance =
        (gtsam::Vector6() << 1e-3, 1e-3, 10.0, 100.0, 100.0, 100.0).finished();
    new_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        index, initial_global_pose, gtsam::noiseModel::Diagonal::Variances(prior_variance)));
  }
  else
  {
    // FAST-LIVO2 只向后端提供相邻关键帧相对运动，不把局部绝对坐标当全局先验。
    const gtsam::Pose3 local_relative = local_keyposes_.back().between(input.local_pose);
    const gtsam::Vector6 odom_variance =
        (gtsam::Vector6() << odom_rotation_variance_, odom_rotation_variance_, odom_rotation_variance_,
                            odom_translation_variance_, odom_translation_variance_, odom_translation_variance_).finished();
    new_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
        index - 1, index, local_relative, gtsam::noiseModel::Diagonal::Variances(odom_variance)));
    initial_global_pose = latest_estimate_.at<gtsam::Pose3>(index - 1).compose(local_relative);
  }

  if (has_rtk)
  {
    // 每个 RTK 因子使用接收机自身协方差，并用 Huber 核降低少量粗差影响。
    const auto gaussian = gtsam::noiseModel::Diagonal::Variances(rtk_measurement.position_variance);
    const auto robust = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian);
    new_factors.add(std::make_shared<AntennaPositionFactor>(
        index, rtk_measurement.position, imu_to_antenna_, robust));

    // 相邻节点均有速度时，用梯形积分平均速度约束天线位移，避免把瞬时速度
    // 直接当成整个关键帧间隔的平均速度而在加减速阶段产生系统误差。
    if (index > 0 && rtk_measurement.velocity_valid && keyframe_rtk_.back().has_value() &&
        keyframe_rtk_.back()->velocity_valid)
    {
      const double dt = input.stamp - keyframe_stamps_.back();
      if (dt > 0.02)
      {
        const Eigen::Vector3d average_velocity =
            0.5 * (keyframe_rtk_.back()->velocity + rtk_measurement.velocity);
        const Eigen::Vector3d average_velocity_variance =
            0.25 * (keyframe_rtk_.back()->velocity_variance +
                    rtk_measurement.velocity_variance) +
            Eigen::Vector3d::Constant(2.5e-3);
        const auto velocity_noise = gtsam::noiseModel::Diagonal::Variances(
            average_velocity_variance);
        const auto velocity_robust = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.345), velocity_noise);
        new_factors.add(std::make_shared<AntennaVelocityFactor>(
            index - 1, index, average_velocity, imu_to_antenna_, dt, velocity_robust));
      }
    }

    // 双天线航向直接消除仅靠位置轨迹在低速或原地转向时的 yaw 弱可观问题。
    if (rtk_measurement.heading_valid)
    {
      const double corrected_heading = rtk_measurement.heading + rtk_heading_offset_;
      const auto heading_noise = gtsam::noiseModel::Isotropic::Variance(
          1, rtk_measurement.heading_variance);
      const auto heading_robust = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(1.345), heading_noise);
      new_factors.add(std::make_shared<HeadingFactor>(index, corrected_heading, heading_robust));
    }
  }

  // Scan Context 只负责候选检索；候选必须再通过点云 GICP 才能成为图约束。
  size_t loop_candidate = 0;
  double context_distance = std::numeric_limits<double>::infinity();
  gtsam::Pose3 loop_relative;
  double gicp_fitness = std::numeric_limits<double>::infinity();
  bool loop_accepted = false;
  if (loop_enabled_ && detectLoopCandidate(scan_context, index, loop_candidate, context_distance))
  {
    loop_accepted = verifyLoopWithGicp(loop_candidate, input, loop_relative, gicp_fitness);
    if (loop_accepted)
    {
      const gtsam::Vector6 loop_variance =
          (gtsam::Vector6() << loop_rotation_variance_, loop_rotation_variance_, loop_rotation_variance_,
                              loop_translation_variance_, loop_translation_variance_, loop_translation_variance_).finished();
      const auto loop_gaussian = gtsam::noiseModel::Diagonal::Variances(loop_variance);
      const auto loop_robust = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Cauchy::Create(1.0), loop_gaussian);
      new_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
          loop_candidate, index, loop_relative, loop_robust));
    }
  }

  new_values.insert(index, initial_global_pose);            // 给新节点提供连续的全局初值。
  isam_->update(new_factors, new_values);                   // 增量加入相对位姿、RTK和回环因子。
  if (loop_accepted)
  {
    isam_->update();                                        // 回环后额外迭代以快速传播全局修正。
    isam_->update();
  }
  latest_estimate_ = isam_->calculateEstimate();            // 获取当前所有关键帧最优位姿。

  local_keyposes_.push_back(input.local_pose);              // 保存前端局部位姿供后续相对因子使用。
  keyframe_clouds_.push_back(input.cloud);                  // 保存原始机体系点云供回环和最终建图。
  keyframe_stamps_.push_back(input.stamp);                  // 保存传感器时间供轨迹输出。
  keyframe_rtk_.push_back(has_rtk
      ? std::optional<BackendRtkMeasurement>(rtk_measurement)
      : std::nullopt);                                      // 保留真实时间匹配结果供速度积分使用。
  scan_context_database_.push_back(scan_context);           // 将描述子加入 Scan Context 数据库。
  if (loop_accepted) loop_edges_.emplace_back(loop_candidate, index);  // 保存回环边用于显示。

  const gtsam::Pose3 optimized_pose = latest_estimate_.at<gtsam::Pose3>(index);
  updateGlobalCorrection(index, input.local_pose, optimized_pose);      // 更新平滑全局修正目标。
  publishBackendState(input.stamp);                                    // 发布后端状态供监控和 RViz 使用。

  RCLCPP_INFO(node_->get_logger(),
              "Backend keyframe=%zu factors=%zu rtk=%s loop=%s sc=%.3f gicp=%.3f",
              index, new_factors.size(), has_rtk ? "yes" : "no",
              loop_accepted ? "yes" : "no", context_distance, gicp_fitness);
}

bool optimization::findNearestRtk(double stamp, BackendRtkMeasurement &measurement)
{
  std::lock_guard<std::mutex> lock(rtk_mutex_);             // 在短临界区内完成按时间查找。
  while (!rtk_queue_.empty() && rtk_queue_.front().stamp < stamp - 5.0)
    rtk_queue_.pop_front();                                 // 清理远早于当前关键帧的历史 RTK。
  if (rtk_queue_.empty()) return false;

  auto best = rtk_queue_.begin();                           // 从队首开始寻找时间差最小观测。
  double best_difference = std::abs(best->stamp - stamp);
  for (auto iterator = std::next(rtk_queue_.begin()); iterator != rtk_queue_.end(); ++iterator)
  {
    const double difference = std::abs(iterator->stamp - stamp);
    if (difference >= best_difference && iterator->stamp > stamp) break;
    if (difference < best_difference)
    {
      best = iterator;
      best_difference = difference;
    }
  }
  if (best_difference > rtk_sync_tolerance_) return false;  // 超过同步门限不创建 RTK 因子。
  measurement = *best;
  rtk_queue_.erase(rtk_queue_.begin(), std::next(best));     // 同一 RTK 观测只能对应一个图节点。
  return true;
}

ScanContextEntry optimization::makeScanContext(const PointCloudXYZRGB::Ptr &cloud) const
{
  ScanContextEntry result;                                  // 初始化空描述子。
  result.descriptor = Eigen::MatrixXf::Zero(sc_rings_, sc_sectors_);

  for (const auto &point : cloud->points)
  {
    const double radius = std::hypot(point.x, point.y);      // XY 平面极径。
    if (radius < 1e-3 || radius > sc_max_radius_) continue; // 忽略原点噪声和范围外点。
    double angle = std::atan2(point.y, point.x);             // 方位角范围先为 [-pi, pi]。
    if (angle < 0.0) angle += 2.0 * kPi;                    // 转换为 [0, 2pi)。
    const int ring = std::min(sc_rings_ - 1,
        static_cast<int>(radius / sc_max_radius_ * sc_rings_));
    const int sector = std::min(sc_sectors_ - 1,
        static_cast<int>(angle / (2.0 * kPi) * sc_sectors_));
    const float height = static_cast<float>(point.z + sc_sensor_height_);
    result.descriptor(ring, sector) = std::max(result.descriptor(ring, sector), height);
  }

  result.ring_key = result.descriptor.rowwise().mean();     // 环均值对 yaw 旋转保持不变。
  return result;
}

bool optimization::detectLoopCandidate(const ScanContextEntry &query,
                                       size_t current_index,
                                       size_t &candidate_index,
                                       double &context_distance) const
{
  if (current_index <= sc_exclude_recent_ || scan_context_database_.size() <= sc_exclude_recent_)
    return false;                                           // 关键帧太少时不尝试回环。

  const size_t candidate_limit = current_index - sc_exclude_recent_;
  std::vector<std::pair<float, size_t>> ring_candidates;    // ring key 距离及对应帧号。
  ring_candidates.reserve(candidate_limit);
  for (size_t index = 0; index < candidate_limit; ++index)
  {
    const float distance = (query.ring_key - scan_context_database_[index].ring_key).norm();
    ring_candidates.emplace_back(distance, index);
  }
  const size_t shortlist_size = std::min<size_t>(10, ring_candidates.size());
  std::partial_sort(ring_candidates.begin(), ring_candidates.begin() + shortlist_size,
                    ring_candidates.end(),
                    [](const auto &left, const auto &right) { return left.first < right.first; });

  context_distance = std::numeric_limits<double>::infinity();
  for (size_t rank = 0; rank < shortlist_size; ++rank)
  {
    const size_t index = ring_candidates[rank].second;
    const double distance = scanContextDistance(query.descriptor,
        scan_context_database_[index].descriptor);
    if (distance < context_distance)
    {
      context_distance = distance;
      candidate_index = index;
    }
  }
  return context_distance < sc_distance_threshold_;         // 只有足够相似才进入耗时 GICP。
}

double optimization::scanContextDistance(const Eigen::MatrixXf &query,
                                         const Eigen::MatrixXf &candidate) const
{
  double best_distance = 1.0;                               // 余弦距离理论范围接近 [0,1]。
  for (int shift = 0; shift < sc_sectors_; ++shift)
  {
    double cosine_sum = 0.0;
    int valid_columns = 0;
    for (int sector = 0; sector < sc_sectors_; ++sector)
    {
      const int shifted_sector = (sector + shift) % sc_sectors_;
      const Eigen::VectorXf query_column = query.col(sector);
      const Eigen::VectorXf candidate_column = candidate.col(shifted_sector);
      const float denominator = query_column.norm() * candidate_column.norm();
      if (denominator < 1e-6f) continue;                    // 两列任一为空时不参与相似度平均。
      cosine_sum += query_column.dot(candidate_column) / denominator;
      ++valid_columns;
    }
    if (valid_columns == 0) continue;
    best_distance = std::min(best_distance, 1.0 - cosine_sum / valid_columns);
  }
  return best_distance;
}

bool optimization::verifyLoopWithGicp(size_t candidate_index,
                                      const BackendKeyframeInput &current,
                                      gtsam::Pose3 &relative_pose,
                                      double &fitness) const
{
  if (candidate_index >= keyframe_clouds_.size()) return false;  // 防止无效候选索引。

  PointCloudXYZRGB::Ptr source(new PointCloudXYZRGB());       // 当前关键帧作为 GICP source。
  PointCloudXYZRGB::Ptr target(new PointCloudXYZRGB());       // 历史候选关键帧作为 GICP target。
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;              // 降采样控制协方差估计和配准开销。
  const float voxel_size = static_cast<float>(gicp_voxel_size_);
  voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel_filter.setInputCloud(current.cloud);
  voxel_filter.filter(*source);
  voxel_filter.setInputCloud(keyframe_clouds_[candidate_index]);
  voxel_filter.filter(*target);
  if (source->size() < 100 || target->size() < 100) return false;

  // FAST-LIVO2 局部位姿给出 GICP 初值 T_candidate_current，提高收敛域并减少假回环。
  const gtsam::Pose3 initial_relative = local_keyposes_[candidate_index].between(current.local_pose);
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
  gicp.setMaxCorrespondenceDistance(gicp_max_correspondence_);  // 排除相距过远的错误对应点。
  gicp.setMaximumIterations(gicp_max_iterations_);              // 限制单次回环验证的最坏耗时。
  gicp.setCorrespondenceRandomness(gicp_correspondence_randomness_);  // 邻域点数决定局部协方差稳定性。
  gicp.setTransformationEpsilon(gicp_transformation_epsilon_); // 平移增量收敛阈值。
  gicp.setRotationEpsilon(gicp_rotation_epsilon_);             // 旋转增量收敛阈值。
  gicp.setInputSource(source);                                  // 设置当前关键帧点云。
  gicp.setInputTarget(target);                                  // 设置历史候选关键帧点云。
  PointCloudXYZRGB aligned;                                     // 仅用于接收 GICP 输出，后续不保存。
  gicp.align(aligned, poseToMatrix4f(initial_relative));
  fitness = gicp.getFitnessScore(gicp_max_correspondence_);     // 在门限范围内计算均方对应距离。
  if (!gicp.hasConverged() || !std::isfinite(fitness) || fitness > gicp_fitness_threshold_)
    return false;

  // PCL 返回 source(current)→target(candidate)，数值恰为 X_candidate^-1*X_current，
  // 与 GTSAM BetweenFactor(candidate,current) 的测量约定一致。
  relative_pose = matrix4fToPose(gicp.getFinalTransformation());
  return true;
}

void optimization::updateGlobalCorrection(size_t,
                                          const gtsam::Pose3 &latest_local_pose,
                                          const gtsam::Pose3 &latest_global_pose)
{
  std::lock_guard<std::mutex> lock(correction_mutex_);       // 与高频发布回调同步修正状态。
  target_map_to_odom_ = latest_global_pose.compose(latest_local_pose.inverse());
  if (!correction_initialized_)
  {
    smoothed_map_to_odom_ = target_map_to_odom_;             // 第一次直接初始化，避免从单位阵缓慢漂移。
    correction_initialized_ = true;
  }
}

gtsam::Pose3 optimization::smoothCorrectionStep()
{
  std::lock_guard<std::mutex> lock(correction_mutex_);       // 原子读取并更新当前修正。
  if (!correction_initialized_) return gtsam::Pose3::Identity();
  const gtsam::Pose3 delta = smoothed_map_to_odom_.between(target_map_to_odom_);
  const gtsam::Vector6 scaled_delta = correction_smoothing_alpha_ * gtsam::Pose3::Logmap(delta);
  smoothed_map_to_odom_ = smoothed_map_to_odom_.compose(gtsam::Pose3::Expmap(scaled_delta));
  return smoothed_map_to_odom_;
}

void optimization::publishBackendState(double stamp)
{
  if (latest_estimate_.empty()) return;                      // 没有优化结果时无需发布。

  pcl::PointCloud<pcl::PointXYZ> keyposes;                   // 生成优化关键帧位置点云。
  global_path_.poses.clear();                                // 回环可能修改历史位姿，因此每次重建路径。
  global_path_.header.stamp = ros::Time().fromSec(stamp);
  for (size_t index = 0; index < local_keyposes_.size(); ++index)
  {
    const gtsam::Pose3 pose = latest_estimate_.at<gtsam::Pose3>(index);
    const gtsam::Point3 position = pose.translation();
    keyposes.emplace_back(position.x(), position.y(), position.z());
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "map";
    pose_stamped.header.stamp = ros::Time().fromSec(keyframe_stamps_[index]);
    gtsamToPoseMsg(pose, pose_stamped.pose);
    global_path_.poses.push_back(pose_stamped);
  }

  sensor_msgs::msg::PointCloud2 keyposes_message;            // 转换并发布优化关键帧点云。
  pcl::toROSMsg(keyposes, keyposes_message);
  keyposes_message.header.frame_id = "map";
  keyposes_message.header.stamp = ros::Time().fromSec(stamp);
  optimized_keyposes_pub_->publish(keyposes_message);
  global_path_pub_->publish(global_path_);                   // 发布完整优化路径。

  visualization_msgs::msg::MarkerArray markers;              // 将每条已接受回环显示为线段。
  visualization_msgs::msg::Marker lines;
  lines.header.frame_id = "map";
  lines.header.stamp = ros::Time().fromSec(stamp);
  lines.ns = "scan_context_loops";
  lines.id = 0;
  lines.type = visualization_msgs::msg::Marker::LINE_LIST;
  lines.action = visualization_msgs::msg::Marker::ADD;
  lines.scale.x = 0.08;
  lines.color.r = 1.0f;
  lines.color.g = 0.2f;
  lines.color.b = 0.1f;
  lines.color.a = 1.0f;
  for (const auto &edge : loop_edges_)
  {
    const gtsam::Point3 first = latest_estimate_.at<gtsam::Pose3>(edge.first).translation();
    const gtsam::Point3 second = latest_estimate_.at<gtsam::Pose3>(edge.second).translation();
    geometry_msgs::msg::Point first_point, second_point;
    first_point.x = first.x(); first_point.y = first.y(); first_point.z = first.z();
    second_point.x = second.x(); second_point.y = second.y(); second_point.z = second.z();
    lines.points.push_back(first_point);
    lines.points.push_back(second_point);
  }
  markers.markers.push_back(lines);
  loop_markers_pub_->publish(markers);                       // 发布 Scan Context/GICP 回环边。
}

void optimization::saveMapService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  save_requested_.store(true);                              // 仅置位，实际保存由后端线程完成。
  queue_cv_.notify_one();                                   // 即使没有新关键帧也唤醒线程。
  response->success = true;
  response->message = "Final map rebuild queued; watch backend log for completion.";
}

bool optimization::reconstructFinalMap()
{
  if (!isam_ || local_keyposes_.empty())
  {
    RCLCPP_WARN(node_->get_logger(), "Final map skipped: no optimized keyframes");
    return false;
  }

  latest_estimate_ = isam_->calculateEstimate();            // 保存前获取最新 iSAM2 估计。
  PointCloudXYZRGB::Ptr full_map(new PointCloudXYZRGB());    // 累积所有优化后的关键帧点云。
  std::ofstream trajectory(output_directory_ + "/TUM/global_optimized.txt");
  trajectory << "# timestamp tx ty tz qx qy qz qw\n";
  trajectory << std::fixed << std::setprecision(9);

  for (size_t index = 0; index < keyframe_clouds_.size(); ++index)
  {
    const gtsam::Pose3 pose = latest_estimate_.at<gtsam::Pose3>(index);
    PointCloudXYZRGB transformed;                            // 将机体系关键帧转换到优化 map 坐标系。
    pcl::transformPointCloud(*keyframe_clouds_[index], transformed, poseToMatrix4f(pose));
    *full_map += transformed;

    const gtsam::Point3 translation = pose.translation();
    const Eigen::Quaterniond quaternion = pose.rotation().toQuaternion();
    trajectory << keyframe_stamps_[index] << " "
               << translation.x() << " " << translation.y() << " " << translation.z() << " "
               << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << " "
               << quaternion.w() << "\n";
  }
  trajectory.close();                                       // 先落盘轨迹，便于地图异常时仍保留定位结果。

  // 同时保存 ENU 原点，否则点云虽在全局坐标中却无法恢复到 WGS84 地理位置。
  {
    std::lock_guard<std::mutex> lock(enu_mutex_);
    if (enu_origin_initialized_)
    {
      std::ofstream origin_file(output_directory_ + "/TUM/enu_origin_lla.txt");
      origin_file << "# latitude_deg longitude_deg altitude_ellipsoid_m\n"
                  << std::fixed << std::setprecision(10)
                  << enu_origin_lla_.x() << " " << enu_origin_lla_.y() << " "
                  << enu_origin_lla_.z() << "\n";
    }
  }

  PointCloudXYZRGB::Ptr filtered_map(new PointCloudXYZRGB()); // 对最终地图统一体素滤波去重。
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
  voxel_filter.setLeafSize(map_voxel_size_, map_voxel_size_, map_voxel_size_);
  voxel_filter.setInputCloud(full_map);
  voxel_filter.filter(*filtered_map);
  const std::string map_path = output_directory_ + "/global_pcd/final_optimized_map.pcd";
  const int save_result = pcl::io::savePCDFileBinary(map_path, *filtered_map);

  if (save_result == 0 && rclcpp::ok())
  {
    sensor_msgs::msg::PointCloud2 map_message;               // 在线保存时同步发布最终地图供 RViz 检查。
    pcl::toROSMsg(*filtered_map, map_message);
    map_message.header.frame_id = "map";
    map_message.header.stamp = node_->now();
    global_map_pub_->publish(map_message);
  }
  RCLCPP_INFO(node_->get_logger(), "Final map rebuilt: keyframes=%zu points=%zu path=%s",
              keyframe_clouds_.size(), filtered_map->size(), map_path.c_str());
  return save_result == 0;
}

gtsam::Pose3 optimization::poseMsgToGtsam(const geometry_msgs::msg::Pose &pose)
{
  const Eigen::Quaterniond quaternion(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  return gtsam::Pose3(gtsam::Rot3(quaternion.normalized()),
                      gtsam::Point3(pose.position.x, pose.position.y, pose.position.z));
}

void optimization::gtsamToPoseMsg(const gtsam::Pose3 &pose, geometry_msgs::msg::Pose &message)
{
  const gtsam::Point3 translation = pose.translation();      // 提取全局平移。
  const Eigen::Quaterniond quaternion = pose.rotation().toQuaternion();  // 提取单位四元数。
  message.position.x = translation.x();
  message.position.y = translation.y();
  message.position.z = translation.z();
  message.orientation.x = quaternion.x();
  message.orientation.y = quaternion.y();
  message.orientation.z = quaternion.z();
  message.orientation.w = quaternion.w();
}
