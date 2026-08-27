/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"
#include "tf_compat.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <gnss_comm/msg/gnss_pvt_soln_msg.hpp>
#include <filesystem>
#include <GeographicLib/LocalCartesian.hpp>
LIVMapper::LIVMapper(const rclcpp::Node::SharedPtr &node)
    : node_(node),
      extT(0, 0, 0),
      extR(M3D::Identity())
{
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);

  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters();
  VoxelMapConfig voxel_config;
  loadVoxelConfig(node_, voxel_config);

  visual_sub_map.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  vio_manager.reset(new VIOManager());
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents();
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "camera_init";
}

LIVMapper::~LIVMapper() {}

void LIVMapper::readParameters()
{
  fastlivo_compat::get_param<string>(node_, "common/lid_topic", lid_topic, "/livox/lidar");
  fastlivo_compat::get_param<string>(node_, "common/imu_topic", imu_topic, "/livox/imu");
  fastlivo_compat::get_param<bool>(node_, "common/ros_driver_bug_fix", ros_driver_fix_en, false);
  fastlivo_compat::get_param<int>(node_, "common/img_en", img_en, 1);
  fastlivo_compat::get_param<int>(node_, "common/lidar_en", lidar_en, 1);
  fastlivo_compat::get_param<string>(node_, "common/img_topic", img_topic, "/left_camera/image");
  fastlivo_compat::get_param<string>(node_, "gps/gps_topic", gps_topic, "/ublox_driver/receiver_pvt");
  fastlivo_compat::get_param<string>(node_, "gps/message_type", gps_message_type, "pvt");
  // QoS for the sensor inputs: "reliable" (default, matches `ros2 bag play`) or
  // "sensor_data"/"best_effort" (matches typical live Livox/IMU/camera drivers).
  fastlivo_compat::get_param<string>(node_, "common/sensor_qos", sensor_qos_mode, std::string("reliable"));

  fastlivo_compat::get_param<bool>(node_, "vio/normal_en", normal_en, true);
  fastlivo_compat::get_param<bool>(node_, "vio/inverse_composition_en", inverse_composition_en, false);
  fastlivo_compat::get_param<int>(node_, "vio/max_iterations", max_iterations, 5);
  fastlivo_compat::get_param<double>(node_, "vio/img_point_cov", IMG_POINT_COV, 100);
  fastlivo_compat::get_param<bool>(node_, "vio/raycast_en", raycast_en, false);
  fastlivo_compat::get_param<bool>(node_, "vio/exposure_estimate_en", exposure_estimate_en, true);
  fastlivo_compat::get_param<double>(node_, "vio/inv_expo_cov", inv_expo_cov, 0.2);
  fastlivo_compat::get_param<int>(node_, "vio/grid_size", grid_size, 5);
  fastlivo_compat::get_param<int>(node_, "vio/grid_n_height", grid_n_height, 17);
  fastlivo_compat::get_param<int>(node_, "vio/patch_pyrimid_level", patch_pyrimid_level, 3);
  fastlivo_compat::get_param<int>(node_, "vio/patch_size", patch_size, 8);
  fastlivo_compat::get_param<double>(node_, "vio/outlier_threshold", outlier_threshold, 1000);

  fastlivo_compat::get_param<double>(node_, "time_offset/exposure_time_init", exposure_time_init, 0.0);
  fastlivo_compat::get_param<double>(node_, "time_offset/img_time_offset", img_time_offset, 0.0);
  fastlivo_compat::get_param<double>(node_, "time_offset/imu_time_offset", imu_time_offset, 0.0);
  fastlivo_compat::get_param<double>(node_, "time_offset/lidar_time_offset", lidar_time_offset, 0.0);
  fastlivo_compat::get_param<bool>(node_, "uav/imu_rate_odom", imu_prop_enable, false);
  fastlivo_compat::get_param<bool>(node_, "uav/gravity_align_en", gravity_align_en, false);

  fastlivo_compat::get_param<string>(node_, "evo/seq_name", seq_name, "01");
  fastlivo_compat::get_param<bool>(node_, "evo/pose_output_en", pose_output_en, false);
  fastlivo_compat::get_param<double>(node_, "imu/gyr_cov", gyr_cov, 1.0);
  fastlivo_compat::get_param<double>(node_, "imu/acc_cov", acc_cov, 1.0);
  fastlivo_compat::get_param<int>(node_, "imu/imu_int_frame", imu_int_frame, 3);
  fastlivo_compat::get_param<bool>(node_, "imu/imu_en", imu_en, false);
  fastlivo_compat::get_param<bool>(node_, "imu/gravity_est_en", gravity_est_en, true);
  fastlivo_compat::get_param<bool>(node_, "imu/ba_bg_est_en", ba_bg_est_en, false);

  fastlivo_compat::get_param<double>(node_, "preprocess/blind", p_pre->blind, 0.01);
  fastlivo_compat::get_param<double>(node_, "preprocess/filter_size_surf", filter_size_surf_min, 0.5);
  fastlivo_compat::get_param<bool>(node_, "preprocess/hilti_en", hilti_en, false);
  fastlivo_compat::get_param<int>(node_, "preprocess/lidar_type", p_pre->lidar_type, AVIA);
  fastlivo_compat::get_param<int>(node_, "preprocess/scan_line", p_pre->N_SCANS, 6);
  fastlivo_compat::get_param<int>(node_, "preprocess/point_filter_num", p_pre->point_filter_num, 3);
  fastlivo_compat::get_param<bool>(node_, "preprocess/feature_extract_enabled", p_pre->feature_enabled, false);

  fastlivo_compat::get_param<int>(node_, "pcd_save/interval", pcd_save_interval, -1);
  fastlivo_compat::get_param<bool>(node_, "pcd_save/pcd_save_en", pcd_save_en, false);
  fastlivo_compat::get_param<bool>(node_, "pcd_save/colmap_output_en", colmap_output_en, false);
  fastlivo_compat::get_param<double>(node_, "pcd_save/filter_size_pcd", filter_size_pcd, 0.5);
  fastlivo_compat::get_param<vector<double>>(node_, "extrin_calib/extrinsic_T", extrinT, vector<double>());
  fastlivo_compat::get_param<vector<double>>(node_, "extrin_calib/extrinsic_R", extrinR, vector<double>());
  fastlivo_compat::get_param<vector<double>>(node_, "extrin_calib/Pcl", cameraextrinT, vector<double>());
  fastlivo_compat::get_param<vector<double>>(node_, "extrin_calib/Rcl", cameraextrinR, vector<double>());
  fastlivo_compat::get_param<double>(node_, "debug/plot_time", plot_time, -10);
  fastlivo_compat::get_param<int>(node_, "debug/frame_cnt", frame_cnt, 6);

  fastlivo_compat::get_param<double>(node_, "publish/blind_rgb_points", blind_rgb_points, 0.01);
  fastlivo_compat::get_param<int>(node_, "publish/pub_scan_num", pub_scan_num, 1);
  fastlivo_compat::get_param<bool>(node_, "publish/pub_effect_point_en", pub_effect_point_en, false);
  fastlivo_compat::get_param<bool>(node_, "publish/dense_map_en", dense_map_en, false);

  fastlivo_compat::get_param<bool>(node_, "gps/debug_mode", debug_mode, false);
  p_pre->blind_sqr = p_pre->blind * p_pre->blind;

  // 旧 RTK 更新默认关闭；RTK 由独立全局 ESIKF 和 GTSAM 后端各自消费纯 LIVO。
  // 若需要诊断旧版前端 RTK 路径，可显式设置 gps/frontend_fusion_en=true。
  fastlivo_compat::get_param<bool>(node_, "gps/frontend_fusion_en", rtk_en, false);
  fastlivo_compat::get_param<double>(node_, "gps/gps_time_offset", gps_time_offset, 0.0);
  fastlivo_compat::get_param<double>(node_, "gps/online_sync_threshold", online_rtk_sync_threshold, 0.05);
  fastlivo_compat::get_param<double>(node_, "gps/online_min_distance", online_rtk_min_distance, 5.0);
  fastlivo_compat::get_param<int>(node_, "gps/online_min_matches", online_rtk_min_matches, 20);
  fastlivo_compat::get_param<double>(node_, "gps/online_max_h_acc", online_rtk_max_h_acc, 0.5);
  fastlivo_compat::get_param<double>(node_, "gps/online_max_v_acc", online_rtk_max_v_acc, 1.0);
  fastlivo_compat::get_param<double>(node_, "gps/online_min_h_std", online_rtk_min_h_std, 0.02);
  fastlivo_compat::get_param<double>(node_, "gps/online_min_v_std", online_rtk_min_v_std, 0.05);
  fastlivo_compat::get_param<double>(node_, "gps/online_innovation_gate", online_rtk_innovation_gate, 16.27);
  fastlivo_compat::get_param<vector<double>>(node_, "gps/extrinsic_T", T_I_R, vector<double>());
  fastlivo_compat::get_param<std::string>(node_, "laserMapping/outputfilepath", save_directory, "output");
  pcd_save_file = save_directory + "/debug/pcd/";
  rtk_save_file = save_directory + "/debug/rtk.txt";
  imu_save_file = save_directory + "/debug/imu.txt";
  odom_save_file = save_directory + "/debug/odom.txt"; 
  cov_save_file  = save_directory + "/debug/cov.txt"; 
}

void LIVMapper::initializeComponents() 
{
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  // ONLINE_RTK: Pass the surveyed IMU-to-antenna lever arm and innovation gate
  // to the joint LiDAR/RTK ESIKF measurement update.
  if (T_I_R.size() >= 3)
  {
    voxelmap_manager->rtk_lever_arm_ << T_I_R[0], T_I_R[1], T_I_R[2];
  }
  else if (rtk_en)
  {
    ROS_WARN("gps/extrinsic_T is missing; online RTK uses a zero lever arm.");
  }
  voxelmap_manager->rtk_innovation_gate_ = online_rtk_innovation_gate;

  if (!vk::camera_loader::loadFromRosNs(node_, "laserMapping", vio_manager->cam)) throw std::runtime_error("Camera model not correctly specified.");

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->initializeVIO();

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_imu_init_frame_num(imu_int_frame);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::initializeFiles()
{
  // Ensure the output / log directory trees exist before opening files (a plain
  // `ros2 launch`/installed run may point outputfilepath somewhere not yet created).
  std::error_code _ec;
  std::filesystem::create_directories(std::string(ROOT_DIR) + "Log/result", _ec);
  std::filesystem::create_directories(std::string(ROOT_DIR) + "Log/PCD", _ec);
  std::filesystem::create_directories(std::string(ROOT_DIR) + "Log/Colmap/sparse/0", _ec);
  if (!save_directory.empty())
  {
    std::filesystem::create_directories(save_directory + "/debug/pcd", _ec);
    for (const auto &sub : {"/TUM", "/vel", "/global_pcd", "/scan_pcd"})
      std::filesystem::create_directories(save_directory + sub, _ec);
  }
  if (pcd_save_en && colmap_output_en)
  {
      const std::string folderPath = std::string(ROOT_DIR) + "/scripts/colmap_output.sh";
      
      std::string chmodCommand = "chmod +x " + folderPath;
      
      int chmodRet = system(chmodCommand.c_str());  
      if (chmodRet != 0) {
          std::cerr << "Failed to set execute permissions for the script." << std::endl;
          return;
      }

      int executionRet = system(folderPath.c_str());
      if (executionRet != 0) {
          std::cerr << "Failed to execute the script." << std::endl;
          return;
      }
  }
  if(colmap_output_en) fout_points.open(std::string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt", std::ios::out);
  if(pcd_save_interval > 0) fout_pcd_pos.open(std::string(ROOT_DIR) + "Log/PCD/scans_pos.json", std::ios::out);
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
}

void LIVMapper::initializeSubscribersAndPublishers(image_transport::ImageTransport &it)
{
  using std::placeholders::_1;
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  // Reliable QoS matching `ros2 bag play` (volatile, keep-last). Depths are
  // generous enough to absorb bursts; the run() loop spins continuously so
  // callbacks drain quickly. Depths are large so a slow mapping frame cannot
  // overflow the reader history and silently drop sensor data (ROS 1 used very
  // large queues + a single global callback queue; this preserves that
  // "nothing dropped" behaviour — the realistic backlog is only a handful).
  //
  // Reliability is selectable: "reliable" (default) matches `ros2 bag play`;
  // "sensor_data"/"best_effort" matches typical live Livox/IMU/camera drivers.
  const bool best_effort = (sensor_qos_mode == "sensor_data" || sensor_qos_mode == "best_effort");
  auto qos_lidar = best_effort ? rclcpp::SensorDataQoS() : rclcpp::QoS(rclcpp::KeepLast(2000));
  auto qos_imu   = best_effort ? rclcpp::SensorDataQoS() : rclcpp::QoS(rclcpp::KeepLast(20000));
  auto qos_img   = best_effort ? rclcpp::SensorDataQoS() : rclcpp::QoS(rclcpp::KeepLast(2000));
  if (p_pre->lidar_type == AVIA)
    sub_pcl = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lid_topic, qos_lidar,
        std::bind(&LIVMapper::livox_pcl_cbk, this, _1));
  else
    sub_pcl = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        lid_topic, qos_lidar,
        std::bind(&LIVMapper::standard_pcl_cbk, this, _1));
  sub_imu = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, qos_imu,
      std::bind(&LIVMapper::imu_cbk, this, _1));
  sub_img = node_->create_subscription<sensor_msgs::msg::Image>(
      img_topic, qos_img,
      std::bind(&LIVMapper::img_cbk, this, _1));

  // 纯 LIVO 模式不再重复执行旧前端 RTK 回调；debug_mode 可保留原始 RTK 诊断输出。
  if (rtk_en || debug_mode) {
    if (gps_message_type == "navsatfix") {
      subGPS_navsatfix = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
          gps_topic, rclcpp::QoS(rclcpp::KeepLast(2000)),
          std::bind(&LIVMapper::navsatfix_cbk, this, _1));
    } else {
      subGPS_pvt = node_->create_subscription<gnss_comm::msg::GnssPVTSolnMsg>(
          gps_topic, rclcpp::QoS(rclcpp::KeepLast(2000)),
          std::bind(&LIVMapper::rtk_cbk, this, _1));
    }
  }

  pub_odom = node_->create_publisher<nav_msgs::msg::Odometry>("/odometry/fast_livo2", rclcpp::QoS(rclcpp::KeepLast(10000)));
  pub_lidarRGB = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/synced_cloud", rclcpp::QoS(rclcpp::KeepLast(10000)));
  pub_rtk = node_->create_publisher<nav_msgs::msg::Odometry>("/gps/odometry", rclcpp::QoS(rclcpp::KeepLast(2000)));

  pubLaserCloudFullRes = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", rclcpp::QoS(rclcpp::KeepLast(100)));
  pubNormal = node_->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", rclcpp::QoS(rclcpp::KeepLast(100)));
  pubSubVisualMap = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_visual_sub_map_before", rclcpp::QoS(rclcpp::KeepLast(100)));
  pubLaserCloudEffect = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", rclcpp::QoS(rclcpp::KeepLast(100)));
  pubLaserCloudMap = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", rclcpp::QoS(rclcpp::KeepLast(100)));
  pubOdomAftMapped = node_->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", rclcpp::QoS(rclcpp::KeepLast(10)));
  pubPath = node_->create_publisher<nav_msgs::msg::Path>("/path", rclcpp::QoS(rclcpp::KeepLast(10)));
  plane_pub = node_->create_publisher<visualization_msgs::msg::Marker>("/planner_normal", rclcpp::QoS(rclcpp::KeepLast(1)));
  voxel_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/voxels", rclcpp::QoS(rclcpp::KeepLast(1)));
  pubLaserCloudDyn = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj", rclcpp::QoS(rclcpp::KeepLast(100)));
  pubLaserCloudDynRmed = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_removed", rclcpp::QoS(rclcpp::KeepLast(100)));
  pubLaserCloudDynDbg = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_dbg_hist", rclcpp::QoS(rclcpp::KeepLast(100)));
  mavros_pose_publisher = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", rclcpp::QoS(rclcpp::KeepLast(10)));
  pubImage = it.advertise("/rgb_img", 1);
  pubImuPropOdom = node_->create_publisher<nav_msgs::msg::Odometry>("/LIVO2/imu_propagate", rclcpp::QoS(rclcpp::KeepLast(2000)));
  imu_prop_timer = node_->create_wall_timer(std::chrono::milliseconds(4), std::bind(&LIVMapper::imu_prop_callback, this));
  voxelmap_manager->voxel_map_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/planes", rclcpp::QoS(rclcpp::KeepLast(1000)));
}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu() 
{
  // double t0 = omp_get_wtime();

  p_imu->Process2(LidarMeasures, _state, feats_undistort, T_G_to_W);

  if (gravity_align_en) gravityAlignment();

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping() 
{
  switch (LidarMeasures.lio_vio_flg) 
  {
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();
      ROS_INFO("[HandleLIO] Position after: [%f, %f, %f]", _state.pos_end(0), _state.pos_end(1), _state.pos_end(2));
      ROS_INFO("[HandleLIO] p_cov after: %.12f, %.12f, %.12f", _state.cov(3,3), _state.cov(4,4), _state.cov(5,5));

      // if(rtk_en && rtk_ini)
      // {
      //   handleRTK();
      // }
      break;
  }
}

void LIVMapper::handleRTK()
{
  ROS_INFO("%s[ RTK Update ]%s", BLUE, RESET);
  if (LidarMeasures.measures.empty()) 
  {
      return; 
  }

  auto rtk_data = LidarMeasures.measures.back().rtk; 
  if(rtk_data.timestamp < 0.00001)
  {
    return;
  }

  std::deque temp_measures = LidarMeasures.measures;

  //ROS_INFO("[HandleRTK] rtk_data.p: [%.6f, %.6f, %.6f]", rtk_data.p[0], rtk_data.p[1], rtk_data.p[2]);
  Eigen::Vector3d z_k = rtk_data.p; 
  z_k[2] = _state.pos_end(2);
  Eigen::Vector3d T_I_to_R;
  T_I_to_R[0] = T_I_R[0];
  T_I_to_R[1] = T_I_R[1];
  T_I_to_R[2] = T_I_R[2];
  ROS_INFO("[HandleRTK] RTK_W: [%f, %f, %f]", z_k(0), z_k(1), z_k(2));
  ROS_INFO("[HandleRTK] Position before: [%f, %f, %f]", _state.pos_end(0), _state.pos_end(1), _state.pos_end(2));
  ROS_INFO("[HandleRTK] p_cov before: %.12f, %.12f, %.12f", _state.cov(3,3), _state.cov(4,4), _state.cov(5,5));
  ROS_INFO("[HandleRTK] Rotation R_before(R1): [%.6f, %.6f, %.6f]", _state.rot_end(0,0), _state.rot_end(0,1), _state.rot_end(0,2));
  ROS_INFO("[HandleRTK] Rotation R_before(R2): [%.6f, %.6f, %.6f]", _state.rot_end(1,0), _state.rot_end(1,1), _state.rot_end(1,2));
  ROS_INFO("[HandleRTK] Rotation R_before(R3): [%.6f, %.6f, %.6f]", _state.rot_end(2,0), _state.rot_end(2,1), _state.rot_end(2,2));

  Eigen::Matrix3d R_cov = Eigen::Matrix3d::Identity() * 1e-4 * 5;

  
  Eigen::Matrix<double, 3, DIM_STATE> H;
  H.setZero();
  H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, DIM_STATE, DIM_STATE> P = _state.cov;
  Eigen::Matrix<double, 3, 3> S = H * P * H.transpose() + R_cov;
  Eigen::Matrix<double, DIM_STATE, 3> K = P * H.transpose() * S.inverse();
  Eigen::Vector3d y = z_k - (_state.pos_end + _state.rot_end * T_I_to_R);
  ROS_INFO("[HandleRTK] y: [%f, %f, %f]", y(0), y(1), y(2));
  Eigen::Matrix<double, DIM_STATE, 1> dx = K * y;
  ROS_INFO("[HandleRTK] dx: [%f, %f, %f, %f, %f, %f]", dx(0), dx(1), dx(2), dx(3), dx(4), dx(5));
  _state += dx;
  
  if (_state.gravity.norm() > 0.1) { 
      _state.gravity += dx.segment<3>(16);
  }

  Eigen::Matrix<double, DIM_STATE, DIM_STATE> I = Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Identity();
  _state.cov = (I - K * H) * P;
  ROS_INFO("[HandleRTK] Position after: [%f, %f, %f]", _state.pos_end(0), _state.pos_end(1), _state.pos_end(2));
  ROS_INFO("[HandleRTK] p_cov after: %.12f, %.12f, %.12f", _state.cov(3,3), _state.cov(4,4), _state.cov(5,5));
  ROS_INFO("[HandleRTK] Rotation R_after(R1): [%.6f, %.6f, %.6f]", _state.rot_end(0,0), _state.rot_end(0,1), _state.rot_end(0,2));
  ROS_INFO("[HandleRTK] Rotation R_after(R2): [%.6f, %.6f, %.6f]", _state.rot_end(1,0), _state.rot_end(1,1), _state.rot_end(1,2));
  ROS_INFO("[HandleRTK] Rotation R_after(R3): [%.6f, %.6f, %.6f]", _state.rot_end(2,0), _state.rot_end(2,1), _state.rot_end(2,2));
  // std::cout << "[RTK Update] Pos Correction: " << dx.segment<3>(3).transpose() << std::endl;
}

void LIVMapper::handleVIO() 
{
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;
    
  if (pcl_w_wait_pub->empty() || (pcl_w_wait_pub == nullptr)) 
  {
    std::cout << "[ VIO ] No point!!!" << std::endl;
    return;
  }
    
  //std::cout << "[ VIO ] Raw feature num: " << pcl_w_wait_pub->points.size() << std::endl;

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) - plot_time) < (frame_cnt / 2 * 0.1)) 
  {
    vio_manager->plot_flag = true;
  } 
  else 
  {
    vio_manager->plot_flag = false;
  }

  vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_, LidarMeasures.last_lio_update_time - _first_lidar_time);

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  // int size_sub_map = vio_manager->visual_sub_map_cur.size();
  // visual_sub_map->reserve(size_sub_map);
  // for (int i = 0; i < size_sub_map; i++) 
  // {
  //   PointType temp_map;
  //   temp_map.x = vio_manager->visual_sub_map_cur[i]->pos_[0];
  //   temp_map.y = vio_manager->visual_sub_map_cur[i]->pos_[1];
  //   temp_map.z = vio_manager->visual_sub_map_cur[i]->pos_[2];
  //   temp_map.intensity = 0.;
  //   visual_sub_map->push_back(temp_map);
  // }

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  publish_img_rgb(pubImage, vio_manager);

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::handleLIO() 
{    
  ROS_INFO("%s[ LIO Update ]%s", BLUE, RESET);
  ROS_INFO("[HandleLIO] Position before: [%f, %f, %f]", _state.pos_end(0), _state.pos_end(1), _state.pos_end(2));
  ROS_INFO("[HandleLIO] P_cov before: %.12f, %.12f, %.12f", _state.cov(3,3), _state.cov(4,4), _state.cov(5,5));
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;
           
  if (feats_undistort->empty() || (feats_undistort == nullptr)) 
  {
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  double t0 = omp_get_wtime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);
  
  double t_down = omp_get_wtime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;
  
  if (!lidar_map_inited) 
  {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  double t1 = omp_get_wtime();

  const RTK &rtk_data = LidarMeasures.measures.back().rtk;
  if(rtk_ini && rtk_data.valid && rtk_data.timestamp > 0.00001)
  {
    rtk_good = true;
  }
  else
  {
    rtk_good = false;
  }
  
  ROS_INFO("[HandleLIO] RTK_W: [%f, %f, %f]", rtk_data.p(0), rtk_data.p(1), rtk_data.p(2));
  voxelmap_manager->StateEstimation(state_propagat, rtk_good, rtk_data);
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;

  double t2 = omp_get_wtime();

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (pose_output_en) 
  {
    static bool pos_opend = false;
    static int ocount = 0;
    std::ofstream outFile, evoFile;
    if (!pos_opend) 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open()) ROS_ERROR("open fail\n");
    } 
    else 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::app);
      if (!evoFile.is_open()) ROS_ERROR("open fail\n");
    }
    Eigen::Matrix4d outT;
    Eigen::Quaterniond q(_state.rot_end);
    evoFile << std::fixed;
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  
  euler_cur = RotMtoEuler(_state.rot_end);
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
  publish_odometry(pubOdomAftMapped);

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) 
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  //std::cout << "[ LIO ] Update Voxel Map" << std::endl;
  _pv_list = voxelmap_manager->pv_list_;
  
  double t4 = omp_get_wtime();

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }
  
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  if (!img_en) publish_frame_world(pubLaserCloudFullRes, vio_manager);
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();
  publish_path(pubPath);
  publish_mavros(mavros_pose_publisher);

  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  // printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  // printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
  // printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  // printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
  // printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  // printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  // printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  // printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  // printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  // printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  // printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
  // printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::savePCD() 
{
  if (pcd_save_en && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval < 0) 
  {
    std::string raw_points_dir = std::string(ROOT_DIR) + "Log/PCD/all_raw_points.pcd";
    std::string downsampled_points_dir = std::string(ROOT_DIR) + "Log/PCD/all_downsampled_points.pcd";
    pcl::PCDWriter pcd_writer;

    if (img_en)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);
  
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save); // Save the raw point cloud data
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;
      
      pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the downsampled point cloud data
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir 
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

      if(colmap_output_en)
      {
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) 
        {
            const auto& point = downsampled_cloud->points[i];
            fout_points << i << " "
                        << std::fixed << std::setprecision(6)
                        << point.x << " " << point.y << " " << point.z << " "
                        << static_cast<int>(point.r) << " "
                        << static_cast<int>(point.g) << " "
                        << static_cast<int>(point.b) << " "
                        << 0 << std::endl;
        }
      }
    }
    else
    {      
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
  }
}

void LIVMapper::run() 
{
  ros::Rate rate(5000);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node_);
    if (!sync_packages(LidarMeasures))
    {
      rate.sleep();
      continue;
    }
    handleFirstFrame();

    processImu();

    // if (!p_imu->imu_time_init) continue;
    stateEstimationAndMapping();
  }
  savePCD();
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback()
{
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) { return; }
  mtx_buffer_imu_prop.lock();
  new_imu = false; // 控制propagate频率和IMU频率一致
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg)
    {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() && toSec(prop_imu_buffer.front().header.stamp) < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (int i = 0; i < prop_imu_buffer.size(); i++)
      {
        double t_from_lidar_end_time = toSec(prop_imu_buffer[i].header.stamp) - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    }
    else
    {
      V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
      V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
      double t_from_lidar_end_time = toSec(newest_imu.header.stamp) - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    imu_prop_odom.header.frame_id = "world";
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    pubImuPropOdom->publish(imu_prop_odom);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  if (!lidar_en) return;
  mtx_buffer.lock();

  double cur_head_time = toSec(msg->header.stamp) + lidar_time_offset;
  // cout<<"got feature"<<endl;
  if (cur_head_time < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", toSec(msg->header.stamp));
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg_in)
{
  if (!lidar_en) return;
  mtx_buffer.lock();
  livox_ros_driver2::msg::CustomMsg::SharedPtr msg(new livox_ros_driver2::msg::CustomMsg(*msg_in));
  // if (toSec((abs(msg->header.stamp) - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("lidar jumps %.3f\n", toSec(msg->header.stamp) - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_lidar + 0.1);
  // }
  if (abs(last_timestamp_imu - toSec(msg->header.stamp)) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - toSec(msg->header.stamp);
    printf("\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = toSec(msg->header.stamp);
  //ROS_INFO("Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", toSec(msg->header.stamp));
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->empty()) {
    ROS_ERROR("Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}


void LIVMapper::rtk_cbk(const gnss_comm::msg::GnssPVTSolnMsg::ConstSharedPtr& gpsMsg)
{
    ROS_INFO("RTK received");
    Eigen::Vector3d trans_local_;
    static bool first_gps = false;
    if (!first_gps) {
        first_gps = true;
        gps_trans_.Reset(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);
        std::ofstream gps_file_header(rtk_save_file, std::ios::out); 
        if (gps_file_header.is_open()) {
            gps_file_header << "# timestamp x y z vx vy vz h_acc v_acc" << std::endl;
            gps_file_header.close();
        }
    }

    const double GPS_EPOCH_UNIX_TIME = 315964800.0;
    const double SECONDS_PER_WEEK = 604800.0; 
    const double LEAP_SECONDS = 18.0;
    int week = gpsMsg->time.week;
    double tow = gpsMsg->time.tow; 
    double total_gps_seconds = (double)week * SECONDS_PER_WEEK + tow;
    double timestamp_sec = total_gps_seconds + GPS_EPOCH_UNIX_TIME - LEAP_SECONDS;
    ros::Time stamp;
    stamp.fromSec(timestamp_sec);

    gps_trans_.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, trans_local_[0], trans_local_[1], trans_local_[2]);

    RTK rtk_data;
    // ONLINE_RTK: Use the configured offset instead of the former hard-coded
    // -1 ms correction, and retain receiver-reported measurement quality.
    rtk_data.timestamp = stamp.toSec() - gps_time_offset;
    rtk_data.p[0] = trans_local_[0];
    rtk_data.p[1] = trans_local_[1];
    rtk_data.p[2] = trans_local_[2];
    rtk_data.v[0] = gpsMsg->vel_e;
    rtk_data.v[1] = gpsMsg->vel_n;
    rtk_data.v[2] = -gpsMsg->vel_d;
    const double h_std = std::max(gpsMsg->h_acc, online_rtk_min_h_std);
    const double v_std = std::max(gpsMsg->v_acc, online_rtk_min_v_std);
    rtk_data.position_cov = (V3D(h_std * h_std, h_std * h_std, v_std * v_std)).asDiagonal();
    rtk_data.valid = gpsMsg->valid_fix && gpsMsg->diff_soln && gpsMsg->carr_soln == 2 &&
                     std::isfinite(gpsMsg->h_acc) && std::isfinite(gpsMsg->v_acc) &&
                     gpsMsg->h_acc <= online_rtk_max_h_acc && gpsMsg->v_acc <= online_rtk_max_v_acc;

    if (rtk_en && rtk_data.valid)
    {
      rtk_buffer.push_back(rtk_data);
      // ONLINE_RTK: Bound pre-initialization storage during long stationary
      // periods or poor-motion data collection.
      while (rtk_buffer.size() > 5000) rtk_buffer.pop_front();
      if (!rtk_ini && InitializeRTK())
      {
        rtk_ini = true;
        ROS_INFO("ONLINE_RTK: RTK-to-LIVO transformation initialized; front-end fusion enabled.");
      }
    }

    // if(rtk_ini)
    // {
    //trans_local_ = T_G_to_W * Eigen::Vector3d(trans_local_[0], trans_local_[1], trans_local_[2]);
    nav_msgs::msg::Odometry gps_odom;
    gps_odom.header.stamp = stamp; 
    gps_odom.header.frame_id = "camera_init";
    gps_odom.pose.pose.position.x = trans_local_[0];
    gps_odom.pose.pose.position.y = trans_local_[1];
    gps_odom.pose.pose.position.z = trans_local_[2];
    gps_odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    gps_odom.twist.twist.linear.x = gpsMsg->vel_e;
    gps_odom.twist.twist.linear.y = gpsMsg->vel_n;
    gps_odom.twist.twist.linear.z = -gpsMsg->vel_d;
    gps_odom.pose.covariance[0] = gpsMsg->h_acc; 
    gps_odom.pose.covariance[7] = gpsMsg->h_acc; 
    gps_odom.pose.covariance[14] = gpsMsg->v_acc; 

    pub_rtk->publish(gps_odom);
    
    if(debug_mode)
    {
      std::ofstream gps_file(rtk_save_file, std::ios::app); 
      if (gps_file.is_open()) {
          gps_file << std::fixed << std::setprecision(6) 
                  << toSec(gps_odom.header.stamp) << " "  // 1. Timestamp
                  << trans_local_[0] << " "                // 2. x
                  << trans_local_[1] << " "                // 3. y
                  << trans_local_[2] << " "                // 4. z
                  << gpsMsg->vel_e << " "                  // 5. vx (East)
                  << gpsMsg->vel_n << " "                  // 6. vy (North)
                  << -gpsMsg->vel_d << " "                 // 7. vz (Up, 注意取反)
                  << gpsMsg->h_acc << " "                  // 8. h_acc
                  << gpsMsg->v_acc                         // 9. v_acc
                  << std::endl;
          gps_file.close();
      }
    // }
  }
}

void LIVMapper::navsatfix_cbk(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gpsMsg)
{
  if (gpsMsg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX ||
      !std::isfinite(gpsMsg->latitude) || !std::isfinite(gpsMsg->longitude) ||
      !std::isfinite(gpsMsg->altitude)) {
    ROS_WARN("Ignoring invalid NavSatFix message");
    return;
  }

  Eigen::Vector3d trans_local;
  static bool first_gps = true;
  if (first_gps) {
    first_gps = false;
    gps_trans_.Reset(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);
  }
  gps_trans_.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude,
                     trans_local.x(), trans_local.y(), trans_local.z());

  const double stamp = rclcpp::Time(gpsMsg->header.stamp).seconds();
  RTK rtk_data;
  // ONLINE_RTK: NavSatFix already carries Unix/ROS time; apply the same
  // configurable sensor offset as the PVT input path.
  rtk_data.timestamp = stamp - gps_time_offset;
  rtk_data.p = trans_local;
  rtk_data.v.setZero();
  const double x_std = std::sqrt(std::max(gpsMsg->position_covariance[0], online_rtk_min_h_std * online_rtk_min_h_std));
  const double y_std = std::sqrt(std::max(gpsMsg->position_covariance[4], online_rtk_min_h_std * online_rtk_min_h_std));
  const double z_std = std::sqrt(std::max(gpsMsg->position_covariance[8], online_rtk_min_v_std * online_rtk_min_v_std));
  rtk_data.position_cov = (V3D(x_std * x_std, y_std * y_std, z_std * z_std)).asDiagonal();
  rtk_data.valid = x_std <= online_rtk_max_h_acc && y_std <= online_rtk_max_h_acc &&
                   z_std <= online_rtk_max_v_acc;

  if (rtk_en && rtk_data.valid) {
    rtk_buffer.push_back(rtk_data);
    while (rtk_buffer.size() > 5000) rtk_buffer.pop_front();
    if (!rtk_ini && InitializeRTK()) {
      rtk_ini = true;
      ROS_INFO("ONLINE_RTK: RTK-to-LIVO transformation initialized; front-end fusion enabled.");
    }
  }

  nav_msgs::msg::Odometry gps_odom;
  gps_odom.header = gpsMsg->header;
  gps_odom.header.frame_id = "camera_init";
  gps_odom.pose.pose.position.x = trans_local.x();
  gps_odom.pose.pose.position.y = trans_local.y();
  gps_odom.pose.pose.position.z = trans_local.z();
  gps_odom.pose.pose.orientation.w = 1.0;
  gps_odom.pose.covariance[0] = gpsMsg->position_covariance[0];
  gps_odom.pose.covariance[7] = gpsMsg->position_covariance[4];
  gps_odom.pose.covariance[14] = gpsMsg->position_covariance[8];
  pub_rtk->publish(gps_odom);
}

Sophus::SE3 computeSVD(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& target, 
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& source)
{
    if (target.empty() || target.size() != source.size()) {
        return Sophus::SE3(); 
    }

    Eigen::Vector3d target_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d source_center = Eigen::Vector3d::Zero();
    for (const auto& p : target) target_center += p;
    for (const auto& p : source) source_center += p;
    target_center /= target.size();
    source_center /= source.size();

    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < source.size(); ++i) {
        W += (target[i] - target_center) * (source[i] - source_center).transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
    if (R.determinant() < 0) { 
        R = svd.matrixU() * Eigen::DiagonalMatrix<double, 3>(1, 1, -1) * svd.matrixV().transpose();
    }

    Eigen::Vector3d t = target_center - R * source_center;
    
    return Sophus::SE3(Sophus::SO3(R), t);
}

bool LIVMapper::InitializeRTK()
{
    // ONLINE_RTK: Initialization is attempted incrementally, but fusion is not
    // enabled until enough time-matched samples and translational excitation
    // are present.  This prevents the old "warn then initialize anyway" path.
    if (rtk_buffer.size() < static_cast<size_t>(online_rtk_min_matches) ||
        livo_state_buffer.size() < static_cast<size_t>(online_rtk_min_matches))
    {
        return false;
    }

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts_source_livo; // Source
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts_target_rtk;  // Target
    auto livo_it = livo_state_buffer.begin();

    for (const auto& rtk_data : rtk_buffer)
    {
        double t_rtk = rtk_data.timestamp;
        while (livo_it != livo_state_buffer.end())
        {
            double t_livo = (*livo_it)[0]; 
            if (t_livo < t_rtk - online_rtk_sync_threshold) {
                livo_it++;
            } else {
                break;
            }
        }

        if (livo_it == livo_state_buffer.end()) break;

        auto best_it = livo_it;
        double min_dt = std::abs((*best_it)[0] - t_rtk);

        auto next_it = std::next(livo_it);
        if (next_it != livo_state_buffer.end())
        {
            double dt_next = std::abs((*next_it)[0] - t_rtk);
            if (dt_next < min_dt)
            {
                min_dt = dt_next;
                best_it = next_it;
            }
        }

        if (min_dt < online_rtk_sync_threshold)
        {
          if (best_it->size() < 4) continue;
            pts_target_rtk.push_back(rtk_data.p);
            Eigen::Vector3d p_livo((*best_it)[1], (*best_it)[2], (*best_it)[3]);
            pts_source_livo.push_back(p_livo);
        }
    }

    if (pts_target_rtk.size() < static_cast<size_t>(online_rtk_min_matches)) {
        return false;
    }

    double max_baseline = 0.0;
    for (const auto &p : pts_source_livo)
    {
      max_baseline = std::max(max_baseline, (p - pts_source_livo.front()).norm());
    }
    if (max_baseline < online_rtk_min_distance)
    {
      return false;
    }

    T_W_to_G = computeSVD(pts_target_rtk, pts_source_livo);
    T_G_to_W = T_W_to_G.inverse();
    const Eigen::Matrix4d transform = T_W_to_G.matrix();
    if (!transform.allFinite())
    {
      ROS_ERROR("ONLINE_RTK: non-finite RTK-to-LIVO initialization result rejected.");
      return false;
    }

    ROS_INFO("ONLINE_RTK: initialized from %zu matches, LIVO baseline %.2f m.",
             pts_target_rtk.size(), max_baseline);
    return true;
}

void LIVMapper::imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in)
{
  if (!imu_en) return;

  if (last_timestamp_lidar < 0.0) return;
  // ROS_INFO("get imu at time: %.6f", toSec(msg_in->header.stamp));
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
  msg->header.stamp = ros::Time().fromSec(toSec(msg->header.stamp) - imu_time_offset);
  double timestamp = toSec(msg->header.stamp);

  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    ROS_WARN("IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en) timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = ros::Time().fromSec(timestamp);

  static bool first_imu_save = true;

  if (first_imu_save) {
      std::ofstream imu_file(imu_save_file, std::ios::out);
      if (imu_file.is_open()) {
          imu_file << "# timestamp ax ay az gx gy gz" << std::endl;
          imu_file.close();
      }
      first_imu_save = false;
  }

  if(debug_mode)
  {
    std::ofstream imu_file(imu_save_file, std::ios::app);
    if (imu_file.is_open()) {
        imu_file << std::fixed << std::setprecision(6)
                << timestamp << " "                     // 时间戳
                << msg->linear_acceleration.x << " "    // ax
                << msg->linear_acceleration.y << " "    // ay
                << msg->linear_acceleration.z << " "    // az
                << msg->angular_velocity.x << " "       // gx
                << msg->angular_velocity.y << " "       // gy
                << msg->angular_velocity.z << std::endl;// gz
        imu_file.close();
    }
  }

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    ROS_ERROR("imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  // if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
  // {

  //   ROS_WARN("imu time stamp Jumps %0.4lf seconds \n", timestamp - last_timestamp_imu);
  //   mtx_buffer.unlock();
  //   sig_buffer.notify_all();
  //   return;
  // }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
  cv::Mat img;
  img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  return img;
}

void LIVMapper::img_cbk(const sensor_msgs::msg::Image::ConstSharedPtr &msg_in)
{
  if (!img_en) return;
  sensor_msgs::msg::Image::SharedPtr msg(new sensor_msgs::msg::Image(*msg_in));
  // if (toSec((abs(msg->header.stamp) - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("img jumps %.3f\n", toSec(msg->header.stamp) - last_timestamp_img);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  if (hilti_en)
  {
    static int frame_counter = 0;
    if (++frame_counter % 4 != 0) return;
  }
  // double msg_header_time =  toSec(msg->header.stamp);
  double msg_header_time = toSec(msg->header.stamp) + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
  //ROS_INFO("Get image, its header time: %.6f", msg_header_time);
  if (last_timestamp_lidar < 0) return;

  if (msg_header_time < last_timestamp_img)
  {
    ROS_ERROR("image loop back. \n");
    return;
  }

  mtx_buffer.lock();

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  if (img_time_correct - last_timestamp_img < 0.02)
  {
    ROS_WARN("Image need Jumps: %.6f", img_time_correct);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  cv::Mat img_cur = getImageFromMsg(msg);
  img_buffer.push_back(img_cur);
  img_time_buffer.push_back(img_time_correct);

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  last_timestamp_img = img_time_correct;
  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  if (img_buffer.empty() && img_en) return false;
  if (imu_buffer.empty() && imu_en) return false;

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;                                                                                       // flag
    }

    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (toSec(imu_buffer.front()->header.stamp) > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed = false; // sync one whole lidar scan.
    if(!img_en)keyframe_time = meas.lidar_frame_end_time;
    return true;

    break;
  }

  case LIVO:
  {
    /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
     * first than VIO imediatly ***/
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // double t0 = omp_get_wtime();
    switch (last_lio_vio_flg)
    {
    // double img_capture_time = meas.lidar_frame_beg_time + exposure_time_init;
    case WAIT:
    case VIO:
    {
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      if(img_en)keyframe_time = img_capture_time;
      /*** has img topic, but img topic timestamp larger than lidar end time,
       * process lidar topic. After LIO update, the meas.lidar_frame_end_time
       * will be refresh. ***/
      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
      // printf("[ Data Cut ] wait \n");
      // printf("[ Data Cut ] last_lio_update_time: %lf \n",
      // meas.last_lio_update_time);

      double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.back()->points.back().curvature / double(1000);
      double imu_newest_time = toSec(imu_buffer.back()->header.stamp);

      if (img_capture_time < meas.last_lio_update_time + 0.00001)
      {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        ROS_ERROR("[ Data Cut ] Throw one image frame! \n");
        return false;
      }

      if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
      {
        // ROS_ERROR("lost first camera frame");
        // printf("img_capture_time, lid_newest_time, imu_newest_time: %lf , %lf
        // , %lf \n", img_capture_time, lid_newest_time, imu_newest_time);
        return false;
      }

      struct MeasureGroup m;

      // printf("[ Data Cut ] LIO \n");
      // printf("[ Data Cut ] img_capture_time: %lf \n", img_capture_time);
      m.imu.clear();
      m.rtk = RTK();
      m.lio_time = img_capture_time;
      mtx_buffer.lock();

      if (rtk_en && rtk_ini)
      {
          // ONLINE_RTK: Consume the entire interval and retain its newest
          // valid sample.  CGI-430 may publish faster than the 10 Hz LIO cycle;
          // the old early break kept the oldest sample and increased latency.
          while (!rtk_buffer.empty())
          {
            ROS_INFO("[SyncRTK] RTK_buffer front time: %.6f, last_lio_update_time: %.6f, lio_time: %.6f", rtk_buffer.front().timestamp, meas.last_lio_update_time, m.lio_time);
            if (rtk_buffer.front().timestamp > m.lio_time) break;
            if (rtk_buffer.front().timestamp > meas.last_lio_update_time && rtk_buffer.front().valid)
            {
                m.rtk = rtk_buffer.front();
            }
            rtk_buffer.pop_front();
          }
          ROS_INFO("[SyncRTK] RTK.pushed to MeasureGroup: %.6f, %.3f, %.3f, %.3f", m.rtk.timestamp, m.rtk.p[0], m.rtk.p[1], m.rtk.p[2]);
      }
      
      while (!imu_buffer.empty())
      {
        if (toSec(imu_buffer.front()->header.stamp) > m.lio_time) break;

        if (toSec(imu_buffer.front()->header.stamp) > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

        imu_buffer.pop_front();
        // printf("[ Data Cut ] imu time: %lf \n",
        // toSec(imu_buffer.front()->header.stamp));
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      int lid_frame_num = lid_raw_data_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);
      // deque<PointCloudXYZI::Ptr> lidar_buffer_tmp;

      while (!lid_raw_data_buffer.empty())
      {
        if (lid_header_time_buffer.front() > img_capture_time) break;
        auto pcl(lid_raw_data_buffer.front()->points);
        double frame_header_time(lid_header_time_buffer.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (int i = 0; i < pcl.size(); i++)
        {
          auto pt = pcl[i];
          if (pcl[i].curvature < max_offs_time_ms)
          {
            pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            meas.pcl_proc_cur->points.push_back(pt);
          }
          else
          {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // meas.last_lio_update_time = m.lio_time;
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      // printf("[ Data Cut ] pcl_proc_cur number: %d \n", meas.pcl_proc_cur
      // ->points.size()); printf("[ Data Cut ] LIO process time: %lf \n",
      // omp_get_wtime() - t0);
      return true;
    }

    case LIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      meas.lio_vio_flg = VIO;
      // printf("[ Data Cut ] VIO \n");
      meas.measures.clear();
      double imu_time = toSec(imu_buffer.front()->header.stamp);

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();
      mtx_buffer.lock();
      
      // while ((!imu_buffer.empty() && (imu_time < img_capture_time)))
      // {
      //   imu_time = toSec(imu_buffer.front()->header.stamp);
      //   if (imu_time > img_capture_time) break;
      //   m.imu.push_back(imu_buffer.front());
      //   imu_buffer.pop_front();
      //   printf("[ Data Cut ] imu time: %lf \n",
      //   toSec(imu_buffer.front()->header.stamp));
      // }

      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed = false; // after VIO update, the _lidar_frame_end_time will be refresh.
      // printf("[ Data Cut ] VIO process time: %lf \n", omp_get_wtime() - t0);
      return true;
    }

    default:
    {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed) 
    { 
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;             
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  ROS_ERROR("out sync");
}

void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager)
{
  cv::Mat img_rgb = vio_manager->img_cp;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = ros::Time::now();
  // out_msg.header.frame_id = "camera_init";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  pubImage.publish(out_msg.toImageMsg());
}

void LIVMapper::publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes, VIOManagerPtr vio_manager)
{
  ROS_INFO("Publish frame world pointcloud.");
  if (pcl_w_wait_pub->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  if (img_en)
  {
    static int pub_num = 1;
    *pcl_wait_pub += *pcl_w_wait_pub;
    if(pub_num == pub_scan_num)
    {
      pub_num = 1;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      // double inv_expo = _state.inv_expo_time;
      cv::Mat img_rgb = vio_manager->img_rgb;
      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;

        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        V3D pf(vio_manager->new_frame_->w2f(p_w)); if (pf[2] < 0) continue;
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];
          // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
          // if (pointRGB.r > 255) pointRGB.r = 255;
          // else if (pointRGB.r < 0) pointRGB.r = 0;
          // if (pointRGB.g > 255) pointRGB.g = 255;
          // else if (pointRGB.g < 0) pointRGB.g = 0;
          // if (pointRGB.b > 255) pointRGB.b = 255;
          // else if (pointRGB.b < 0) pointRGB.b = 0;
          if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
    else
    {
      pub_num++;
    }
  }

  /*** Publish Frame ***/
  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  if (img_en)
  {
    // cout << "RGB pointcloud size: " << laserCloudWorldRGB->size() << endl;
    pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
  }
  else 
  { 
    pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg); 
  }
  laserCloudmsg.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes->publish(laserCloudmsg);

  if(img_en)
  {
    PointCloudXYZRGB::Ptr laserCloudBodyRGB(new PointCloudXYZRGB());
    laserCloudBodyRGB->reserve(laserCloudWorldRGB->size());
    for (const auto& pt_world_rgb : laserCloudWorldRGB->points)
    {
          V3D p_global(pt_world_rgb.x, pt_world_rgb.y, pt_world_rgb.z);
          V3D p_body = _state.rot_end.transpose() * (p_global - _state.pos_end);
          PointTypeRGB pointBodyRGB;
          pointBodyRGB.x = p_body(0);
          pointBodyRGB.y = p_body(1);
          pointBodyRGB.z = p_body(2);
          pointBodyRGB.r = pt_world_rgb.r;
          pointBodyRGB.g = pt_world_rgb.g;
          pointBodyRGB.b = pt_world_rgb.b;
          laserCloudBodyRGB->push_back(pointBodyRGB);
    }
    sensor_msgs::msg::PointCloud2 laserCloudbodymsg;
    pcl::toROSMsg(*laserCloudBodyRGB, laserCloudbodymsg);
    laserCloudbodymsg.header.stamp = ros::Time(keyframe_time);
    laserCloudbodymsg.header.frame_id = "camera_init";
    pub_lidarRGB->publish(laserCloudbodymsg);

    if(debug_mode)
    {
      if (laserCloudBodyRGB->size() > 0) 
      {
          std::stringstream ss;
          ss << std::setfill('0') << std::setw(6) << pcd_file_index;
          std::string pcd_filename = pcd_save_file + ss.str() + ".pcd";    
          pcl::io::savePCDFileBinary(pcd_filename, *laserCloudBodyRGB);
          pcd_file_index++;
      }
    }
  }
  
  if(!img_en)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudBody(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudBody->reserve(pcl_w_wait_pub->size());
    for (const auto& pt_world : pcl_w_wait_pub->points)
    {
          V3D p_global(pt_world.x, pt_world.y, pt_world.z);
          V3D p_body = _state.rot_end.transpose() * (p_global - _state.pos_end);
          pcl::PointXYZI pointBody;
          pointBody.x = p_body(0);
          pointBody.y = p_body(1);
          pointBody.z = p_body(2);
          laserCloudBody->push_back(pointBody);
    }
      sensor_msgs::msg::PointCloud2 laserCloudbodymsg;
      pcl::toROSMsg(*laserCloudBody, laserCloudbodymsg);
      laserCloudbodymsg.header.stamp = ros::Time(keyframe_time);
      laserCloudbodymsg.header.frame_id = "camera_init";
      pub_lidarRGB->publish(laserCloudbodymsg);

      if(debug_mode)
      {
        if (laserCloudBody->size() > 0) 
        {
          std::stringstream ss;
          ss << std::setfill('0') << std::setw(6) << pcd_file_index;
          std::string pcd_filename = pcd_save_file + ss.str() + ".pcd";
          pcl::io::savePCDFileBinary(pcd_filename, *laserCloudBody);
          pcd_file_index++;
        }
      }
  }

  
  if (rtk_en && !rtk_ini)
  {
    // ONLINE_RTK: Align like with like: RTK reports the antenna position, so
    // the LIVO initialization trajectory must also be the predicted antenna
    // trajectory.  Aligning IMU positions here and then applying the lever arm
    // again in StateEstimation creates a constant bias (and usually causes the
    // NIS gate to reject every online RTK update). Keep only initialization
    // samples; after alignment this buffer is no longer needed.
    const V3D livo_antenna =
        _state.pos_end + _state.rot_end * voxelmap_manager->rtk_lever_arm_;
    std::vector<double> livo_xyz(4);
    livo_xyz[0] = keyframe_time;
    livo_xyz[1] = livo_antenna(0);
    livo_xyz[2] = livo_antenna(1);
    livo_xyz[3] = livo_antenna(2);
    livo_state_buffer.push_back(livo_xyz);
    while (livo_state_buffer.size() > 5000) livo_state_buffer.pop_front();
  }


  nav_msgs::msg::Odometry odom_msg_ekf;
  odom_msg_ekf.header.stamp = ros::Time(keyframe_time);
  odom_msg_ekf.header.frame_id = "camera_init";
  odom_msg_ekf.pose.pose.position.x = _state.pos_end(0);
  odom_msg_ekf.pose.pose.position.y = _state.pos_end(1);
  odom_msg_ekf.pose.pose.position.z = _state.pos_end(2);
  Eigen::Quaterniond q(_state.rot_end);
  odom_msg_ekf.pose.pose.orientation.w = q.w();
  odom_msg_ekf.pose.pose.orientation.x = q.x();
  odom_msg_ekf.pose.pose.orientation.y = q.y();
  odom_msg_ekf.pose.pose.orientation.z = q.z();
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> pose_cov_map(odom_msg_ekf.pose.covariance.data());

  pose_cov_map.block<3, 3>(0, 0) = _state.cov.block<3, 3>(3, 3);
  pose_cov_map.block<3, 3>(3, 3) = _state.cov.block<3, 3>(0, 0);
  pose_cov_map.block<3, 3>(0, 3) = _state.cov.block<3, 3>(3, 0);
  pose_cov_map.block<3, 3>(3, 0) = _state.cov.block<3, 3>(0, 3);
  pub_odom->publish(odom_msg_ekf); 

  if(debug_mode)
  {
    std::ofstream odom_file(odom_save_file, std::ios::app); 
    if (odom_file.is_open()) {
        odom_file << std::fixed << std::setprecision(6) 
                  << keyframe_time << " "
                  << _state.pos_end(0) << " "
                  << _state.pos_end(1) << " "
                  << _state.pos_end(2) << " "
                  << q.x() << " "
                  << q.y() << " "
                  << q.z() << " "
                  << q.w() << std::endl;
        odom_file.close();
    }

    std::ofstream cov_file(cov_save_file, std::ios::app);
    if (cov_file.is_open()) {
        cov_file << std::fixed << std::setprecision(6) << keyframe_time;
        
        for (int i = 0; i < 36; ++i) {
            cov_file << " " << odom_msg_ekf.pose.covariance[i];
        }
        cov_file << std::endl;
        cov_file.close();
    }
  }
  
  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  if (pcd_save_en)
  {
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
    static int scan_wait_num = 0;

    if (img_en)
    {
      *pcl_wait_save += *laserCloudWorldRGB;
    }
    else
    {
      *pcl_wait_save_intensity += *pcl_w_wait_pub;
    }
    scan_wait_num++;

    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      pcd_index++;
      string all_points_dir(string(string(ROOT_DIR) + "Log/PCD/") + to_string(pcd_index) + string(".pcd"));
      pcl::PCDWriter pcd_writer;
      if (pcd_save_en)
      {
        cout << "current scan saved to /PCD/" << all_points_dir << endl;
        if (img_en)
        {
          pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
          PointCloudXYZRGB().swap(*pcl_wait_save);
        }
        else
        {
          pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
          PointCloudXYZI().swap(*pcl_wait_save_intensity);
        }        
        Eigen::Quaterniond q(_state.rot_end);
        fout_pcd_pos << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.w() << " " << q.x() << " " << q.y()
                     << " " << q.z() << " " << endl;
        scan_wait_num = 0;
      }
    }
  }
  if(laserCloudWorldRGB->size() > 0)  PointCloudXYZI().swap(*pcl_wait_pub); 
  PointCloudXYZI().swap(*pcl_w_wait_pub);
}

void LIVMapper::publish_visual_sub_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubSubVisualMap)
{
  PointCloudXYZI::Ptr laserCloudFullRes(visual_sub_map);
  int size = laserCloudFullRes->points.size(); if (size == 0) return;
  PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
  *sub_pcl_visual_map_pub = *laserCloudFullRes;
  if (1)
  {
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time::now();
    laserCloudmsg.header.frame_id = "camera_init";
    pubSubVisualMap->publish(laserCloudmsg);
  }
}

void LIVMapper::publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = ros::Time::now();
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect->publish(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped)
{
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "aft_mapped";
  odomAftMapped.header.stamp = ros::Time::now(); //.ros::Time()fromSec(last_timestamp_lidar);
  set_posestamp(odomAftMapped.pose.pose);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = odomAftMapped.header.stamp;
  transform.header.frame_id = "camera_init";
  transform.child_frame_id = "aft_mapped";
  transform.transform.translation.x = _state.pos_end(0);
  transform.transform.translation.y = _state.pos_end(1);
  transform.transform.translation.z = _state.pos_end(2);
  transform.transform.rotation.w = geoQuat.w;
  transform.transform.rotation.x = geoQuat.x;
  transform.transform.rotation.y = geoQuat.y;
  transform.transform.rotation.z = geoQuat.z;
  tf_broadcaster_->sendTransform(transform);
  pubOdomAftMapped->publish(odomAftMapped);
}

void LIVMapper::publish_mavros(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &mavros_pose_publisher)
{
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "camera_init";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher->publish(msg_body_pose);
}

void LIVMapper::publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "camera_init";
  path.poses.push_back(msg_body_pose);
  pubPath->publish(path);
}
