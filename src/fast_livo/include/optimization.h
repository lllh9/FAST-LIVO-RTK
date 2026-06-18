#pragma once 
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "tf_compat.hpp"

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <gnss_comm/msg/gnss_pvt_soln_msg.hpp>

#include <GeographicLib/LocalCartesian.hpp>
#include <memory>
#include "LIVMapper.h"
// NB: FastDTW/example.hpp defines non-inline free functions (DTW), so it must be
// included only in the single TU that uses them (optimization.cpp) — including
// it here would multiply-define DTW in every TU that includes optimization.h.

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D;                  
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
} EIGEN_ALIGN16;      

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT PointTypePose;

inline gtsam::Pose3 trans2gtsamPose(float transformIn[])
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                        gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

inline Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
{ 
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

inline gtsam::Pose3 computeSVD(const std::vector<Eigen::Vector3d>& target, 
                               const std::vector<Eigen::Vector3d>& source)
{
    if (target.empty() || target.size() != source.size()) {
        return gtsam::Pose3::Identity(); 
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
    
    return gtsam::Pose3{gtsam::Rot3(R), gtsam::Point3(t)};
}

class optimization
{
public:
    optimization(const rclcpp::Node::SharedPtr &node);
    ~optimization();

    void loadData(const std::string& data_dir);
    void offlineOptimizationTask();
    void initialAlign();
    double calculateDtwTimeOffset(const std::vector<std::vector<double>>& gpsdata, const std::vector<std::vector<double>>& slamdata);
    void buildBatchGraph();
    void saveKeyFramesAndFactor();
    void syncedCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odomMsg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloudMsg);
    void gpsHandler(const gnss_comm::msg::GnssPVTSolnMsg::ConstSharedPtr& pvtMsg);
    // void gpsHandler(const sensor_msgs::msg::NavSatFixConstPtr& gpsMsg);

    void saveOptimizedGlobalMap();
    void savekeyframescan();
    void writeOptimizedTumTrajectory();
    void writeRtkTumTrajectory();
    
    template<typename T>
    void publishCloud(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub, const T& cloud, const ros::Time& stamp, const std::string& frame_id)
    {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id;
        if (pub) pub->publish(msg);
    }

public:
    rclcpp::Node::SharedPtr node_;
    rclcpp::SubscriptionBase::SharedPtr subGPS;
    rclcpp::Subscription<gnss_comm::msg::GnssPVTSolnMsg>::SharedPtr subGPS_pvt;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubGpsOdom;

    message_filters::Subscriber<nav_msgs::msg::Odometry> subOdom_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subCloud_;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2> SyncPolicy;


    gtsam::NonlinearFactorGraph gtSAMgraph; 
    gtsam::Values initialEstimate;        
    gtsam::Values isamCurrentEstimate;
    gtsam::Values optimizedEstimate;
    
    gtsam::Pose3 T_imu_rtk;


    PointCloudXYZRGB::Ptr laserCloudSurfLastDS;
    vector<PointCloudXYZRGB::Ptr> surfCloudKeyFrames;
    
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    std::thread optimization_thread_;

    std::deque<nav_msgs::msg::Odometry> gpsQueue;
    std::deque<nav_msgs::msg::Odometry> gpsQueue_B;
    GeographicLib::LocalCartesian gps_trans_;

    float gpstimestamp = 0.0;
    double timeLaserInfoCur;
    ros::Time timeLaserInoStamp;

    float transformTobeMapped[6];
    vector<double> gps_extrinT;

    double gps_offset;

    double rtk_cov;
    double livo2_RPY_cov;
    double livo2_XYZ_cov;
       

    bool debug_mode = false;
    bool addrtkfactor_ = false;
    bool save_pcd_enable_ = false;
    bool is_optimized = false;
    bool gps_en;

    string gps_topic;
    string outputfilepath;
    string debug_optdata_path_;
    string opt_tum_output_path_;
    string gps_tum_output_path_;
    string opt_vel_output_path_;
    string gps_vel_output_path_;
    string global_map_pcd_path_;
    string keyframe_scan_pcd_path_;
    std::string pcd_save_directory_;

private:
    std::mutex mutex;
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};