/**
 * @file utils.hpp
 * @brief Stateless ROS, buffering, publication, and output helpers.
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "imu_processing.hpp"

class Preprocess;

namespace pv_lio_plus
{
enum class MapType;
const char* MapTypeName(MapType type);
}  // namespace pv_lio_plus

/** @brief Forwards a process signal to the ROS shutdown flag. */
void SigHandle(int sig);
/** @brief Converts a LiDAR point into the world frame. */
void RGBpointBodyToWorld(const state_ikfom& state, PointType const* input, PointType* output);
/** @brief Converts a LiDAR point into the IMU body frame. */
void pointLidarToIMU(const state_ikfom& state, PointType const* input, PointType* output);

/** @brief Buffers a standard PointCloud2 scan after preprocessing. */
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr& msg, double lidar_time_offset, std::mutex& buffer_mutex,
                      std::condition_variable& buffer_condition, std::deque<PointCloudXYZI::Ptr>& lidar_buffer,
                      std::deque<double>& time_buffer, double& last_timestamp_lidar, int& scan_count,
                      Preprocess& preprocess);
/** @brief Buffers a Livox scan after preprocessing and optional time sync. */
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr& msg, bool time_sync_en, std::mutex& buffer_mutex,
                   std::condition_variable& buffer_condition, std::deque<PointCloudXYZI::Ptr>& lidar_buffer,
                   std::deque<double>& time_buffer, std::deque<sensor_msgs::Imu::ConstPtr>& imu_buffer,
                   double& last_timestamp_lidar, double last_timestamp_imu, bool& timediff_set,
                   double& timediff_lidar_wrt_imu, int& scan_count, Preprocess& preprocess);
/** @brief Buffers a validated IMU sample and applies optional time sync. */
void imu_cbk(const sensor_msgs::Imu::ConstPtr& msg_in, bool time_sync_en, double timediff_lidar_wrt_imu,
             std::mutex& buffer_mutex, std::condition_variable& buffer_condition,
             std::deque<sensor_msgs::Imu::ConstPtr>& imu_buffer, double& last_timestamp_imu);
/** @brief Extracts one synchronized LiDAR/IMU measurement group. */
bool sync_packages(MeasureGroup& measures, std::deque<PointCloudXYZI::Ptr>& lidar_buffer,
                   std::deque<double>& time_buffer, std::deque<sensor_msgs::Imu::ConstPtr>& imu_buffer,
                   bool& lidar_pushed, double& lidar_end_time, double last_timestamp_imu, double& lidar_mean_scantime,
                   int& scan_num);

/** @brief Publishes a world-frame scan and optionally accumulates PCD output. */
void publish_frame_world(const ros::Publisher& publisher, bool scan_pub_en, bool scan_dense_pub_en, bool pcd_save_en,
                         const PointCloudXYZI::Ptr& feats_undistort, const PointCloudXYZI::Ptr& feats_undistort_down,
                         const state_ikfom& state, double lidar_end_time, const std::string& root_dir,
                         int pcd_save_interval, int& pcd_index, int& scan_wait_num, PointCloudXYZI& pcl_wait_save);
/** @brief Publishes the current scan in the IMU body frame. */
void publish_frame_body(const ros::Publisher& publisher, bool scan_dense_pub_en,
                        const PointCloudXYZI::Ptr& feats_undistort, const PointCloudXYZI::Ptr& feats_undistort_down,
                        const state_ikfom& state, double lidar_end_time);
/** @brief Publishes the current scan in the native LiDAR frame. */
void publish_frame_lidar(const ros::Publisher& publisher, bool scan_dense_pub_en,
                         const PointCloudXYZI::Ptr& feats_undistort, const PointCloudXYZI::Ptr& feats_undistort_down,
                         double lidar_end_time);
/** @brief Publishes a local-map snapshot. */
void publish_map_snapshot(const ros::Publisher& publisher, const PointCloudXYZI& map_cloud, double stamp);
/** @brief Publishes odometry and the world/body transforms. */
void publish_odometry(const ros::Publisher& publisher, nav_msgs::Odometry& odometry, const state_ikfom& state,
                      const esekfom::esekf<state_ikfom, 12, input_ikfom>& kf, const ImuProcess& imu_process,
                      double lidar_end_time, const geometry_msgs::Quaternion& quaternion);
/** @brief Appends the current pose to a trajectory vector. */
void record_pose(std::vector<std::vector<double>>& poses, double lidar_end_time, const state_ikfom& state,
                 const geometry_msgs::Quaternion& quaternion);
/** @brief Publishes the accumulated path and records the current pose. */
void publish_path(const ros::Publisher& publisher, nav_msgs::Path& path, geometry_msgs::PoseStamped& body_pose,
                  std::vector<std::vector<double>>& poses, double lidar_end_time, const state_ikfom& state,
                  const geometry_msgs::Quaternion& quaternion);
/** @brief Returns the output filename prefix for a selected map backend. */
std::string result_file_stem(const std::string& backend_name);

/** @brief Reports runtime statistics and persists trajectory and cloud results. */
void save_results(std::size_t converged_count, std::size_t not_converged_count,
                  const std::vector<std::vector<double>>& poses, const std::string& root_dir,
                  pv_lio_plus::MapType map_type, bool pcd_save_en, const PointCloudXYZI& pcl_wait_save);
