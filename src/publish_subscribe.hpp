/**
 * @file publish_subscribe.hpp
 * @brief ROS transport, sensor buffering, and mapping output ownership.
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

#include "config.hpp"
#include "imu_processing.hpp"

class Preprocess;

namespace pv_lio_plus
{
enum class MapType;

/**
 * @brief Owns ROS transport state and the data exchanged with Mapping.
 *
 * Mapping keeps algorithm state and current-frame computation here.  This
 * class owns subscribers, publishers, sensor queues, and output accumulators.
 */
class PublishSubscribe
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** @brief Initializes subscribers, publishers, and the sensor buffers. */
    void Initialize(ros::NodeHandle& nh, const Config& config, Preprocess& preprocess);

    /** @brief Pulls one synchronized LiDAR/IMU group from the owned buffers. */
    bool SyncPackages();

    /** @brief Returns the synchronized measurement group for the current scan. */
    const MeasureGroup& measures() const;

    /** @brief Returns the end timestamp of the synchronized LiDAR scan. */
    double lidar_end_time() const;

    /** @brief Returns the number of converged EKF updates. */
    std::size_t converged_count() const;

    /** @brief Returns the number of non-converged EKF updates. */
    std::size_t not_converged_count() const;

    /** @brief Returns the publisher used for optional voxel-plane visualization. */
    ros::Publisher& voxel_map_publisher();

    /** @brief Publishes one corrected frame and records its trajectory state. */
    void PublishFrame(const PointCloudXYZI::Ptr& undistorted, const PointCloudXYZI::Ptr& downsampled,
                      const state_ikfom& state, const esekfom::esekf<state_ikfom, 12, input_ikfom>& kf,
                      const ImuProcess& imu_process, const geometry_msgs::Quaternion& quaternion, bool converged);

    /** @brief Publishes a local-map snapshot after optional plane visualization. */
    void PublishMapSnapshot(const PointCloudXYZI& map_cloud, double stamp);

    /** @brief Saves trajectory and pending point-cloud results after shutdown. */
    void SaveResults(const Config& config, MapType map_type);

   private:
    /** @brief Registers the Livox subscriber and binds it to the owned queues. */
    void subscribe_livox(ros::NodeHandle& nh, const Config& config, Preprocess& preprocess);

    /** @brief Registers a standard PointCloud2 subscriber. */
    void subscribe_standard_cloud(ros::NodeHandle& nh, const Config& config, Preprocess& preprocess);

    /** @brief Registers the IMU subscriber. */
    void subscribe_imu(ros::NodeHandle& nh, const Config& config);

    /** @brief Advertises all mapping output topics. */
    void advertise_publishers(ros::NodeHandle& nh);

    const Config* config_ = nullptr;

    ros::Subscriber lidar_subscriber_;
    ros::Subscriber imu_subscriber_;
    ros::Publisher cloud_world_publisher_;
    ros::Publisher cloud_body_publisher_;
    ros::Publisher cloud_lidar_publisher_;
    ros::Publisher map_publisher_;
    ros::Publisher odometry_publisher_;
    ros::Publisher path_publisher_;
    ros::Publisher voxel_map_publisher_;

    std::mutex buffer_mutex_;
    std::condition_variable buffer_condition_;
    std::deque<double> time_buffer_;
    std::deque<PointCloudXYZI::Ptr> lidar_buffer_;
    std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer_;
    bool lidar_pushed_             = false;
    double last_timestamp_lidar_   = 0.0;
    double last_timestamp_imu_     = -1.0;
    double lidar_end_time_         = 0.0;
    double timediff_lidar_wrt_imu_ = 0.0;
    bool timediff_set_             = false;
    double lidar_mean_scantime_    = 0.0;
    int scan_num_                  = 0;
    int scan_count_                = 0;
    MeasureGroup measures_;

    nav_msgs::Path path_;
    nav_msgs::Odometry odometry_;
    geometry_msgs::PoseStamped body_pose_;
    std::vector<std::vector<double>> poses_;
    PointCloudXYZI::Ptr pcl_wait_save_{new PointCloudXYZI()};
    int pcd_index_                   = 0;
    int scan_wait_num_               = 0;
    std::size_t converged_count_     = 0;
    std::size_t not_converged_count_ = 0;
};

}  // namespace pv_lio_plus
