#pragma once

// GTSAM：在线增量因子图、位姿因子和噪声模型。
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

// PCL：关键帧点云、Scan Context 输入、ICP 验证和最终地图重建。
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ROS 2：消息同步、全局定位发布、TF 和保存地图服务。
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

// 项目自定义 GNSS 消息和 FAST-LIVO2 公共类型。
#include <gnss_comm/msg/gnss_pvt_soln_msg.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include "LIVMapper.h"
#include "global_esikf.h"

// 标准库：线程、同步、容器和文件路径。
#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// 保存一个待处理关键帧；回调只入队，耗时计算全部交给后端线程。
struct BackendKeyframeInput
{
  double stamp = 0.0;                         // 关键帧传感器时间，单位为秒。
  gtsam::Pose3 local_pose;                    // FAST-LIVO2 在 camera_init/odom 中的局部位姿。
  PointCloudXYZRGB::Ptr cloud;                // 与该位姿同步的机体系彩色点云。
};

// 后端与实时滤波器共享同一份清洗结果，避免两套 RTK 质量逻辑产生差异。
using BackendRtkMeasurement = GlobalRtkObservation;

// Scan Context 数据库条目；descriptor 用于精匹配，ring_key 用于快速候选检索。
struct ScanContextEntry
{
  Eigen::MatrixXf descriptor;                 // 行为半径环、列为方位扇区的最大高度描述子。
  Eigen::VectorXf ring_key;                   // 每个半径环的均值，作为旋转不变粗检索键。
};

// 在线全局后端。保留历史类名 optimization，避免改变 main.cpp 的调用接口。
class optimization
{
public:
  // 创建订阅、发布器、iSAM2 和独立后端线程。
  explicit optimization(const rclcpp::Node::SharedPtr &node);

  // 停止线程并在退出前重建最终轨迹和地图。
  ~optimization();

private:
  // 同步回调只筛选关键帧并将数据复制到有界队列，避免阻塞实时前端。
  void syncedCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg);

  // 高频局部里程计同时驱动实时全局 ESIKF，并发布图优化平滑位姿与 map→odom。
  void localOdometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg);

  // 接收 gnss_comm PVT，检查 FIX 质量并转换成局部 ENU 观测。
  void gpsHandler(const gnss_comm::msg::GnssPVTSolnMsg::ConstSharedPtr &gps_msg);

  // 接收标准 NavSatFix，检查有效性并转换成局部 ENU 观测。
  void navSatFixHandler(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &gps_msg);

  // 后端线程入口：处理关键帧、回环和异步地图保存请求。
  void backendWorker();

  // 将一个关键帧增量加入 iSAM2，并更新全局修正。
  void processKeyframe(const BackendKeyframeInput &input);

  // 根据距离、转角和时间判断是否应创建新的因子图节点。
  bool shouldCreateKeyframe(const gtsam::Pose3 &pose, double stamp) const;

  // 在给定关键帧时间附近查找质量合格的 RTK 观测。
  bool findNearestRtk(double stamp, BackendRtkMeasurement &measurement);

  // 生成 Scan Context 最大高度描述子及其 ring key。
  ScanContextEntry makeScanContext(const PointCloudXYZRGB::Ptr &cloud) const;

  // 使用 ring key 粗检索并通过扇区循环移位计算 Scan Context 相似度。
  bool detectLoopCandidate(const ScanContextEntry &query,
                           size_t current_index,
                           size_t &candidate_index,
                           double &context_distance) const;

  // 计算两个 Scan Context 在所有方位循环移位下的最小余弦距离。
  double scanContextDistance(const Eigen::MatrixXf &query,
                             const Eigen::MatrixXf &candidate) const;

  // 用 GICP 验证候选回环并返回 candidate→current 的相对位姿测量。
  bool verifyLoopWithGicp(size_t candidate_index,
                          const BackendKeyframeInput &current,
                          gtsam::Pose3 &relative_pose,
                          double &fitness) const;

  // 根据最新优化结果计算目标 map→camera_init 修正。
  void updateGlobalCorrection(size_t latest_index,
                              const gtsam::Pose3 &latest_local_pose,
                              const gtsam::Pose3 &latest_global_pose);

  // 在 SE(3) 上指数平滑全局修正，避免 RTK/回环使发布位姿瞬间跳变。
  gtsam::Pose3 smoothCorrectionStep();

  // 发布优化关键帧、全局路径以及用于 RViz 观察的回环连线。
  void publishBackendState(double stamp);

  // 发布通过质量门限的原始 RTK ENU 轨迹，便于和各融合结果直接比较。
  void publishRtkPath(const BackendRtkMeasurement &measurement);

  // 服务回调只提交异步保存请求，不在 ROS 回调线程中拼接大地图。
  void saveMapService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // 使用当前 iSAM2 最优关键帧位姿重建最终轨迹和高精点云地图。
  bool reconstructFinalMap();

  // ROS 位姿消息转换为 GTSAM Pose3。
  static gtsam::Pose3 poseMsgToGtsam(const geometry_msgs::msg::Pose &pose);

  // GTSAM Pose3 写入 ROS 位姿消息。
  static void gtsamToPoseMsg(const gtsam::Pose3 &pose, geometry_msgs::msg::Pose &message);

  // ROS 2 节点由前端和后端共享；耗时任务不在其回调线程运行。
  rclcpp::Node::SharedPtr node_;

  // 同步 FAST-LIVO2 局部位姿与对应机体系点云。
  message_filters::Subscriber<nav_msgs::msg::Odometry> keyframe_odom_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> keyframe_cloud_sub_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> keyframe_sync_;

  // 单独订阅局部里程计，以尽可能高的频率发布平滑全局定位。
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_odom_sub_;

  // 根据参数选择 PVT 或 NavSatFix RTK 输入。
  rclcpp::Subscription<gnss_comm::msg::GnssPVTSolnMsg>::SharedPtr pvt_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;

  // 实时 ESIKF、图优化平滑位姿、路径、关键帧、地图和回环可视化输出。
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr global_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr graph_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rtk_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr realtime_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr optimized_keyposes_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr loop_markers_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 手动触发最终地图重建的 ROS 2 服务。
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_service_;

  // iSAM2 只由 backend_thread_ 访问，无需阻塞实时 ROS 回调。
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::Values latest_estimate_;

  // 因子图中保存的局部位姿、点云、时间和 Scan Context 数据库。
  std::vector<gtsam::Pose3> local_keyposes_;
  std::vector<PointCloudXYZRGB::Ptr> keyframe_clouds_;
  std::vector<double> keyframe_stamps_;
  std::vector<std::optional<BackendRtkMeasurement>> keyframe_rtk_;  // 每个图节点实际匹配的 RTK。
  std::vector<ScanContextEntry> scan_context_database_;

  // 已接受的回环边，用于可视化和诊断。
  std::vector<std::pair<size_t, size_t>> loop_edges_;

  // RTK 队列独立加锁，允许 ROS 回调与后端线程并行读写。
  mutable std::mutex rtk_mutex_;
  std::deque<BackendRtkMeasurement> rtk_queue_;
  mutable std::mutex enu_mutex_;                          // 保护 ENU 投影器及原点元数据。
  GeographicLib::LocalCartesian local_cartesian_;
  bool enu_origin_initialized_ = false;
  Eigen::Vector3d enu_origin_lla_ = Eigen::Vector3d::Zero();  // 保存原点经纬高供地图地理配准。

  // 独立全局误差状态滤波器只消费纯 LIVO 增量，不回写 FAST-LIVO2 局部状态。
  RealtimeGlobalEsikf realtime_global_esikf_;

  // 后端输入使用有界队列，防止回环计算暂时变慢时无限占用内存。
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<BackendKeyframeInput> keyframe_queue_;
  std::mutex keyframe_selection_mutex_;                   // 保护回调侧轻量关键帧筛选状态。
  gtsam::Pose3 last_enqueued_local_pose_;                 // 最近一次入队的纯 LIVO 位姿。
  double last_enqueued_stamp_ = 0.0;                     // 最近一次入队的传感器时间。
  bool have_enqueued_keyframe_ = false;                  // 是否已有回调侧参考关键帧。
  std::thread backend_thread_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> save_requested_{false};

  // 平滑修正的当前值与目标值都表示 map→camera_init(局部 odom) 变换。
  std::mutex correction_mutex_;
  gtsam::Pose3 smoothed_map_to_odom_;
  gtsam::Pose3 target_map_to_odom_;
  bool correction_initialized_ = false;

  // 原始 RTK、实时滤波和图优化路径消息均使用同一个 ENU map 坐标系。
  std::string output_directory_;
  nav_msgs::msg::Path rtk_path_;
  nav_msgs::msg::Path realtime_path_;
  nav_msgs::msg::Path global_path_;
  double last_realtime_path_stamp_ = 0.0;                // Path 降采样，避免高频重复发布大消息。

  // GNSS 与天线杆臂配置。
  std::string gps_topic_;
  std::string gps_message_type_;
  bool gps_enabled_ = true;
  double gps_time_offset_ = 0.0;
  double rtk_sync_tolerance_ = 0.10;
  double rtk_max_horizontal_std_ = 0.50;
  double rtk_max_vertical_std_ = 1.00;
  double rtk_heading_offset_ = 0.0;
  gtsam::Point3 imu_to_antenna_{0.0, 0.0, 0.0};

  // 关键帧和局部里程计因子参数。
  double keyframe_distance_ = 0.50;
  double keyframe_angle_ = 0.10;
  double keyframe_time_ = 1.00;
  double keyframe_sync_tolerance_ = 0.03;
  double odom_rotation_variance_ = 1e-5;
  double odom_translation_variance_ = 1e-3;
  size_t max_pending_keyframes_ = 30;

  // Scan Context 与 GICP 回环参数。
  bool loop_enabled_ = true;
  int sc_rings_ = 20;
  int sc_sectors_ = 60;
  double sc_max_radius_ = 80.0;
  double sc_sensor_height_ = 2.0;
  size_t sc_exclude_recent_ = 30;
  double sc_distance_threshold_ = 0.18;
  double gicp_voxel_size_ = 0.30;
  double gicp_max_correspondence_ = 2.0;
  double gicp_fitness_threshold_ = 0.30;
  int gicp_max_iterations_ = 50;
  int gicp_correspondence_randomness_ = 20;
  double gicp_transformation_epsilon_ = 1e-6;
  double gicp_rotation_epsilon_ = 1e-6;
  double loop_rotation_variance_ = 1e-3;
  double loop_translation_variance_ = 5e-2;

  // 发布平滑与地图重建参数。
  double correction_smoothing_alpha_ = 0.05;
  double map_voxel_size_ = 0.10;
};
