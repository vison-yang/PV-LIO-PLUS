#pragma once

#include "imu_processing.hpp"
#include "preprocess.h"

#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver/CustomMsg.h>
#include <memory>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <vector>

extern std::mutex mtx_buffer;
extern std::condition_variable sig_buffer;
extern std::string root_dir;
extern std::string lid_topic;
extern std::string imu_topic;
extern double lidar_time_offset;
extern double last_timestamp_lidar;
extern double last_timestamp_imu;
extern double lidar_end_time;
extern int scan_count;
extern int publish_count;
extern int pcd_save_interval;
extern int pcd_index;
extern int scan_wait_num;
extern bool time_sync_en;
extern bool pcd_save_en;
extern bool lidar_pushed;
extern bool flg_exit;
extern bool scan_pub_en;
extern bool scan_dense_pub_en;
extern bool scan_body_pub_en;
extern bool scan_lidar_pub_en;
extern double s_plot11[];
extern std::deque<double> time_buffer;
extern std::deque<PointCloudXYZI::Ptr> lidar_buffer;
extern std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
extern PointCloudXYZI::Ptr feats_undistort;
extern PointCloudXYZI::Ptr feats_undistort_down;
extern state_ikfom state_point;
extern esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
extern nav_msgs::Path path;
extern nav_msgs::Odometry odomAftMapped;
extern geometry_msgs::Quaternion geoQuat;
extern geometry_msgs::PoseStamped msg_body_pose;
extern std::shared_ptr<Preprocess> p_pre;
extern std::shared_ptr<ImuProcess> p_imu;
extern std::vector<std::vector<double>> vec_poses;
extern PointCloudXYZI::Ptr pcl_wait_save;

void SigHandle(int sig);
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr& msg);
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr& msg);
void imu_cbk(const sensor_msgs::Imu::ConstPtr& msg_in);
bool sync_packages(MeasureGroup& meas);
void publish_frame_world(const ros::Publisher& pubLaserCloudFull);
void publish_frame_body(const ros::Publisher& _pub);
void publish_frame_lidar(const ros::Publisher& _pub);
/** @brief 发布核心提供的局部地图快照。 */
void publish_map_snapshot(const ros::Publisher& pubLaserCloudMap, const PointCloudXYZI& map_cloud, double stamp);
void publish_odometry(const ros::Publisher& pubOdomAftMapped);
void record_pose();
void publish_path(const ros::Publisher pubPath);
/** @brief 根据后端名称返回结果文件前缀。 */
std::string result_file_stem(const std::string& backend_name);
