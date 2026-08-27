/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "vio.h"
#include "preprocess.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vikit/camera_loader.h>
#include <gnss_comm/msg/gnss_pvt_soln_msg.hpp>
#include <GeographicLib/LocalCartesian.hpp>

class LIVMapper
{
public:
  LIVMapper(const rclcpp::Node::SharedPtr &node);
  ~LIVMapper();
  void initializeSubscribersAndPublishers(image_transport::ImageTransport &it);
  void initializeComponents();
  void initializeFiles();
  void run();
  void gravityAlignment();
  void handleFirstFrame();
  void stateEstimationAndMapping();
  void handleVIO();
  void handleLIO();
  void handleRTK();
  void savePCD();
  void processImu();

  bool sync_packages(LidarMeasureGroup &meas);
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);
  void imu_prop_callback();
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
  void pointBodyToWorld(const PointType &pi, PointType &po);

  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
  void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg_in);
  void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in);
  void img_cbk(const sensor_msgs::msg::Image::ConstSharedPtr &msg_in);
  void rtk_cbk(const gnss_comm::msg::GnssPVTSolnMsg::ConstSharedPtr& gpsMsg);
  void navsatfix_cbk(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gpsMsg);
  // ONLINE_RTK: Returns true only after a well-supported LIVO<->ENU alignment
  // has actually been computed.  Callers must not enable fusion on failure.
  bool InitializeRTK();
  void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);
  void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes, VIOManagerPtr vio_manager);
  void publish_visual_sub_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubSubVisualMap);
  void publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);
  void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped);
  void publish_mavros(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &mavros_pose_publisher);
  void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath);
  void readParameters();
  template <typename T> void set_posestamp(T &out);
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
  cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;

  SLAM_MODE slam_mode_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  
  string root_dir;
  string lid_topic, imu_topic, seq_name, img_topic, gps_topic, gps_message_type;
  string sensor_qos_mode = "reliable";
  V3D extT;
  M3D extR;

  int feats_down_size = 0, max_iterations = 0;

  double res_mean_last = 0.05;
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;
  double blind_rgb_points = 0.0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  double _first_lidar_time = 0.0;
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  bool lidar_map_inited = false, pcd_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
  int pcd_save_interval = -1, pcd_index = 0;
  int pub_scan_num = 1;

  StatesGroup imu_propagate, latest_ekf_state;

  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
  deque<sensor_msgs::msg::Imu> prop_imu_buffer;
  sensor_msgs::msg::Imu newest_imu;
  double latest_ekf_time;
  nav_msgs::msg::Odometry imu_prop_odom;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuPropOdom;
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;

  bool gravity_align_en = false, gravity_align_finished = false;

  bool sync_jump_flag = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
  bool dense_map_en = false;
  int img_en = 1, imu_int_frame = 3;
  bool normal_en = true;
  bool exposure_estimate_en = false;
  double exposure_time_init = 0.0;
  bool inverse_composition_en = false;
  bool raycast_en = false;
  int lidar_en = 1;
  bool is_first_frame = false;
  int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
  double outlier_threshold;
  double plot_time;
  int frame_cnt;
  double img_time_offset = 0.0;
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;
  deque<cv::Mat> img_buffer;
  deque<double> img_time_buffer;
  vector<pointWithVar> _pv_list;
  vector<double> extrinT;
  vector<double> extrinR;
  vector<double> cameraextrinT;
  vector<double> cameraextrinR;
  double IMG_POINT_COV;

  PointCloudXYZI::Ptr visual_sub_map;
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZRGB::Ptr pcl_wait_save;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;

  ofstream fout_pre, fout_out, fout_pcd_pos, fout_points;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  LidarMeasureGroup LidarMeasures;
  StatesGroup _state;
  StatesGroup  state_propagat;

  nav_msgs::msg::Path path;
  nav_msgs::msg::Odometry odomAftMapped;
  geometry_msgs::msg::Quaternion geoQuat;
  geometry_msgs::msg::PoseStamped msg_body_pose;

  PreprocessPtr p_pre;
  ImuProcessPtr p_imu;
  VoxelMapManagerPtr voxelmap_manager;
  VIOManagerPtr vio_manager;
  GeographicLib::LocalCartesian gps_trans_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr plane_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxel_pub;
  rclcpp::SubscriptionBase::SharedPtr sub_pcl;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
  rclcpp::Subscription<gnss_comm::msg::GnssPVTSolnMsg>::SharedPtr subGPS_pvt;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGPS_navsatfix;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubNormal;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSubVisualMap;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDyn;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDynRmed;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDynDbg;
  image_transport::Publisher pubImage;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavros_pose_publisher;
  rclcpp::TimerBase::SharedPtr imu_prop_timer;

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  bool colmap_output_en = false;

  bool rtk_en = false;
  bool rtk_ini = false;
  bool rtk_good = false;
  std::deque<RTK> rtk_buffer;
  std::deque<RTK> rtk_proc_buffer;
  std::deque<std::vector<double>> livo_state_buffer;
  Sophus::SE3 T_W_to_G;
  Sophus::SE3 T_G_to_W;
  vector<double> T_I_R;
  // ONLINE_RTK: Front-end timing, quality gating and alignment parameters.
  double gps_time_offset = 0.0;
  double online_rtk_sync_threshold = 0.05;
  double online_rtk_min_distance = 5.0;
  double online_rtk_max_h_acc = 0.5;
  double online_rtk_max_v_acc = 1.0;
  double online_rtk_min_h_std = 0.02;
  double online_rtk_min_v_std = 0.05;
  double online_rtk_innovation_gate = 16.27;
  int online_rtk_min_matches = 20;

  int pcd_file_index = 0;
  bool debug_mode = false;
  std::string save_directory;
  std::string pcd_save_file;
  std::string rtk_save_file;
  std::string imu_save_file;
  std::string odom_save_file; 
  std::string cov_save_file; 
  
  double keyframe_time = 0.0;

  rclcpp::SubscriptionBase::SharedPtr sub_gps;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidarRGB;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_rtk;
};
#endif
