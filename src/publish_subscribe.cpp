/**
 * @file publish_subscribe.cpp
 * @brief ROS transport, sensor buffering, and mapping output implementation.
 */

#include "publish_subscribe.hpp"

#include "preprocess.h"
#include "utils.hpp"

namespace pv_lio_plus
{
void PublishSubscribe::Initialize(ros::NodeHandle& nh, const Config& config, Preprocess& preprocess)
{
    config_ = &config;

    if (preprocess.lidar_type == AVIA)
    {
        subscribe_livox(nh, config, preprocess);
    }
    else
    {
        subscribe_standard_cloud(nh, config, preprocess);
    }

    subscribe_imu(nh, config);
    advertise_publishers(nh);

    path_.header.stamp    = ros::Time::now();
    path_.header.frame_id = "camera_init";
}

bool PublishSubscribe::SyncPackages()
{
    return sync_packages(measures_, lidar_buffer_, time_buffer_, imu_buffer_, lidar_pushed_, lidar_end_time_,
                         last_timestamp_imu_, lidar_mean_scantime_, scan_num_);
}

const MeasureGroup& PublishSubscribe::measures() const
{
    return measures_;
}

double PublishSubscribe::lidar_end_time() const
{
    return lidar_end_time_;
}

std::size_t PublishSubscribe::converged_count() const
{
    return converged_count_;
}

std::size_t PublishSubscribe::not_converged_count() const
{
    return not_converged_count_;
}

ros::Publisher& PublishSubscribe::voxel_map_publisher()
{
    return voxel_map_publisher_;
}

void PublishSubscribe::subscribe_livox(ros::NodeHandle& nh, const Config& config, Preprocess& preprocess)
{
    lidar_subscriber_ = nh.subscribe<livox_ros_driver::CustomMsg>(
        config.lidar_topic_, 200000, [this, &config, &preprocess](const livox_ros_driver::CustomMsg::ConstPtr& msg) {
            livox_pcl_cbk(msg, config.time_sync_en_, buffer_mutex_, buffer_condition_, lidar_buffer_, time_buffer_,
                          imu_buffer_, last_timestamp_lidar_, last_timestamp_imu_, timediff_set_,
                          timediff_lidar_wrt_imu_, scan_count_, preprocess);
        });
}

void PublishSubscribe::subscribe_standard_cloud(ros::NodeHandle& nh, const Config& config, Preprocess& preprocess)
{
    lidar_subscriber_ = nh.subscribe<sensor_msgs::PointCloud2>(
        config.lidar_topic_, 200000, [this, &config, &preprocess](const sensor_msgs::PointCloud2::ConstPtr& msg) {
            standard_pcl_cbk(msg, config.lidar_time_offset_, buffer_mutex_, buffer_condition_, lidar_buffer_,
                             time_buffer_, last_timestamp_lidar_, scan_count_, preprocess);
        });
}

void PublishSubscribe::subscribe_imu(ros::NodeHandle& nh, const Config& config)
{
    imu_subscriber_ = nh.subscribe<sensor_msgs::Imu>(
        config.imu_topic_, 200000, [this, &config](const sensor_msgs::Imu::ConstPtr& msg) {
            imu_cbk(msg, config.time_sync_en_, timediff_lidar_wrt_imu_, buffer_mutex_, buffer_condition_, imu_buffer_,
                    last_timestamp_imu_);
        });
}

void PublishSubscribe::advertise_publishers(ros::NodeHandle& nh)
{
    cloud_world_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    cloud_body_publisher_  = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
    cloud_lidar_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_lidar", 100000);
    map_publisher_         = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
    odometry_publisher_    = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    path_publisher_        = nh.advertise<nav_msgs::Path>("/path", 100000);
    voxel_map_publisher_   = nh.advertise<visualization_msgs::MarkerArray>("/planes", 10000);
}

void PublishSubscribe::PublishFrame(const PointCloudXYZI::Ptr& undistorted, const PointCloudXYZI::Ptr& downsampled,
                                    const state_ikfom& state, const esekfom::esekf<state_ikfom, 12, input_ikfom>& kf,
                                    const ImuProcess& imu_process, const geometry_msgs::Quaternion& quaternion,
                                    bool converged)
{
    const double stamp = lidar_end_time_;
    const OdometryPublishContext odometry_context{odometry_, state, kf, imu_process, stamp, quaternion};
    publish_odometry(odometry_publisher_, odometry_context);

    const PathPublishContext path_context{path_, body_pose_, poses_, stamp, state, quaternion};
    if (config_->path_pub_en_)
    {
        publish_path(path_publisher_, path_context);
    }
    else
    {
        record_pose(poses_, stamp, state, quaternion);
    }

    const FramePublishContext frame_context{undistorted, downsampled, state, stamp, config_->scan_dense_pub_en_};
    if (config_->scan_pub_en_)
    {
        publish_frame_world(cloud_world_publisher_, frame_context);
    }
    if (config_->pcd_save_en_)
    {
        PcdOutputContext pcd_context{config_->root_dir_, config_->pcd_save_interval_, pcd_index_, scan_wait_num_,
                                     *pcl_wait_save_};
        accumulate_frame_pcd(frame_context, pcd_context);
    }
    if (config_->scan_pub_en_ && config_->scan_body_pub_en_)
    {
        publish_frame_body(cloud_body_publisher_, frame_context);
    }
    if (config_->scan_pub_en_ && config_->scan_lidar_pub_en_)
    {
        publish_frame_lidar(cloud_lidar_publisher_, frame_context);
    }
    if (converged)
    {
        ++converged_count_;
    }
    else
    {
        ++not_converged_count_;
    }
}

void PublishSubscribe::PublishMapSnapshot(const PointCloudXYZI& map_cloud, double stamp)
{
    publish_map_snapshot(map_publisher_, map_cloud, stamp);
}

void PublishSubscribe::SaveResults(const Config& config, MapType map_type)
{
    save_results(converged_count_, not_converged_count_, poses_, config.root_dir_, map_type, config.pcd_save_en_,
                 *pcl_wait_save_);
}

}  // namespace pv_lio_plus
