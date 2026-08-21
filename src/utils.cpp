/**
 * @file utils.cpp
 * @brief Stateless ROS callbacks, buffering, publication, and output helpers.
 */

#include "utils.hpp"

#include "preprocess.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

/** @brief Copies the current state and orientation into a ROS pose message. */
template <typename PoseMessage>
static void set_posestamp(PoseMessage& message, const state_ikfom& state, const geometry_msgs::Quaternion& quaternion) {
    message.pose.position.x = state.pos(0);
    message.pose.position.y = state.pos(1);
    message.pose.position.z = state.pos(2);
    message.pose.orientation.x = quaternion.x;
    message.pose.orientation.y = quaternion.y;
    message.pose.orientation.z = quaternion.z;
    message.pose.orientation.w = quaternion.w;
}

/** @brief Requests shutdown after receiving a POSIX signal. */
void SigHandle(int sig) {
    ROS_WARN("catch sig %d", sig);
    ros::requestShutdown();
}

/** @brief Converts a LiDAR point into the world frame. */
void RGBpointBodyToWorld(const state_ikfom& state, PointType const* input, PointType* output) {
    const V3D point_body(input->x, input->y, input->z);
    const V3D point_world(state.rot * (state.offset_R_L_I * point_body + state.offset_T_L_I) + state.pos);
    output->x = point_world(0);
    output->y = point_world(1);
    output->z = point_world(2);
    output->intensity = input->intensity;
}

/** @brief Converts a LiDAR point into the IMU body frame. */
void pointLidarToIMU(const state_ikfom& state, PointType const* input, PointType* output) {
    const V3D point_lidar(input->x, input->y, input->z);
    const V3D point_imu(state.offset_R_L_I * point_lidar + state.offset_T_L_I);
    output->x = point_imu(0);
    output->y = point_imu(1);
    output->z = point_imu(2);
    output->intensity = input->intensity;
    output->curvature = input->curvature;
    output->normal_x = input->normal_x;
}

/** @brief Buffers a standard PointCloud2 scan after preprocessing. */
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr& msg, double lidar_time_offset, std::mutex& buffer_mutex,
                      std::condition_variable& buffer_condition, std::deque<PointCloudXYZI::Ptr>& lidar_buffer,
                      std::deque<double>& time_buffer, double& last_timestamp_lidar, int& scan_count,
                      Preprocess& preprocess) {
    const double timestamp = msg->header.stamp.toSec() + lidar_time_offset;
    std::lock_guard<std::mutex> lock(buffer_mutex);
    ++scan_count;
    if (timestamp < last_timestamp_lidar) {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    preprocess.process(msg, cloud);
    lidar_buffer.push_back(cloud);
    time_buffer.push_back(timestamp);
    last_timestamp_lidar = timestamp;
    buffer_condition.notify_all();
}

/** @brief Buffers a Livox scan after preprocessing and optional time sync. */
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr& msg, bool time_sync_en, std::mutex& buffer_mutex,
                   std::condition_variable& buffer_condition, std::deque<PointCloudXYZI::Ptr>& lidar_buffer,
                   std::deque<double>& time_buffer, std::deque<sensor_msgs::Imu::ConstPtr>& imu_buffer,
                   double& last_timestamp_lidar, double last_timestamp_imu, bool& timediff_set,
                   double& timediff_lidar_wrt_imu, int& scan_count, Preprocess& preprocess) {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    ++scan_count;
    const double timestamp = msg->header.stamp.toSec();
    if (timestamp < last_timestamp_lidar) {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = timestamp;
    if (!time_sync_en && std::abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() &&
        !lidar_buffer.empty()) {
        ROS_WARN("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf", last_timestamp_imu,
                 last_timestamp_lidar);
    }
    if (time_sync_en && !timediff_set && std::abs(last_timestamp_lidar - last_timestamp_imu) > 1.0 &&
        !imu_buffer.empty()) {
        timediff_set = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        ROS_INFO("Self sync IMU and LiDAR, time diff is %.10lf", timediff_lidar_wrt_imu);
    }
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    preprocess.process(msg, cloud);
    lidar_buffer.push_back(cloud);
    time_buffer.push_back(last_timestamp_lidar);
    buffer_condition.notify_all();
}

/** @brief Buffers a validated IMU sample and applies optional time sync. */
void imu_cbk(const sensor_msgs::Imu::ConstPtr& msg_in, bool time_sync_en, double timediff_lidar_wrt_imu,
             std::mutex& buffer_mutex, std::condition_variable& buffer_condition,
             std::deque<sensor_msgs::Imu::ConstPtr>& imu_buffer, double& last_timestamp_imu) {
    sensor_msgs::Imu::Ptr message(new sensor_msgs::Imu(*msg_in));
    if (std::abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en) {
        message->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }
    const double timestamp = message->header.stamp.toSec();
    if (timestamp < last_timestamp_imu) {
        ROS_WARN("imu loop back, ignoring!!!");
        return;
    }
    if (std::abs(message->angular_velocity.x) > 10 || std::abs(message->angular_velocity.y) > 10 ||
        std::abs(message->angular_velocity.z) > 10) {
        ROS_WARN("Large IMU measurement!!! Drop Data!!!");
        return;
    }
    last_timestamp_imu = timestamp;
    std::lock_guard<std::mutex> lock(buffer_mutex);
    imu_buffer.push_back(message);
    buffer_condition.notify_all();
}

/** @brief Extracts one synchronized LiDAR/IMU measurement group. */
bool sync_packages(MeasureGroup& measures, std::deque<PointCloudXYZI::Ptr>& lidar_buffer,
                   std::deque<double>& time_buffer, std::deque<sensor_msgs::Imu::ConstPtr>& imu_buffer,
                   bool& lidar_pushed, double& lidar_end_time, double last_timestamp_imu, double& lidar_mean_scantime,
                   int& scan_num) {
    if (lidar_buffer.empty() || imu_buffer.empty())
        return false;
    if (!lidar_pushed) {
        measures.lidar = lidar_buffer.front();
        measures.lidar_beg_time = time_buffer.front();
        if (measures.lidar->points.size() <= 1 ||
            measures.lidar->points.back().curvature / 1000.0 < 0.5 * lidar_mean_scantime) {
            lidar_end_time = measures.lidar_beg_time + lidar_mean_scantime;
        } else {
            ++scan_num;
            const double scan_time = measures.lidar->points.back().curvature / 1000.0;
            lidar_end_time = measures.lidar_beg_time + scan_time;
            lidar_mean_scantime += (scan_time - lidar_mean_scantime) / scan_num;
        }
        measures.lidar_end_time = lidar_end_time;
        lidar_pushed = true;
    }
    if (last_timestamp_imu < lidar_end_time)
        return false;
    measures.imu.clear();
    while (!imu_buffer.empty() && imu_buffer.front()->header.stamp.toSec() <= lidar_end_time) {
        measures.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

/** @brief Publishes a world-frame scan and optionally accumulates PCD output. */
void publish_frame_world(const ros::Publisher& publisher, bool scan_pub_en, bool scan_dense_pub_en, bool pcd_save_en,
                         const PointCloudXYZI::Ptr& feats_undistort, const PointCloudXYZI::Ptr& feats_undistort_down,
                         const state_ikfom& state, double lidar_end_time, const std::string& root_dir,
                         int pcd_save_interval, int& pcd_index, int& scan_wait_num, PointCloudXYZI& pcl_wait_save) {
    const PointCloudXYZI::Ptr cloud(scan_dense_pub_en ? feats_undistort : feats_undistort_down);
    if (scan_pub_en) {
        PointCloudXYZI world_cloud;
        for (const auto& point : cloud->points) {
            if (point.intensity < 5)
                continue;
            PointType world_point;
            RGBpointBodyToWorld(state, &point, &world_point);
            world_cloud.push_back(world_point);
        }
        sensor_msgs::PointCloud2 message;
        pcl::toROSMsg(world_cloud, message);
        message.header.stamp = ros::Time().fromSec(lidar_end_time);
        message.header.frame_id = "camera_init";
        publisher.publish(message);
    }
    if (!pcd_save_en)
        return;
    PointCloudXYZI world_cloud;
    world_cloud.reserve(feats_undistort->size());
    for (const auto& point : feats_undistort->points) {
        PointType world_point;
        RGBpointBodyToWorld(state, &point, &world_point);
        world_cloud.push_back(world_point);
    }
    pcl_wait_save += world_cloud;
    ++scan_wait_num;
    if (pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval && !pcl_wait_save.empty()) {
        const std::string pcd_dir = root_dir + "PCD";
        std::filesystem::create_directories(pcd_dir);
        const std::string path = pcd_dir + "/scans_" + std::to_string(++pcd_index) + ".pcd";
        pcl::PCDWriter writer;
        writer.writeBinary(path, pcl_wait_save);
        pcl_wait_save.clear();
        scan_wait_num = 0;
    }
}

/** @brief Publishes the current scan in the IMU body frame. */
void publish_frame_body(const ros::Publisher& publisher, bool scan_dense_pub_en,
                        const PointCloudXYZI::Ptr& feats_undistort, const PointCloudXYZI::Ptr& feats_undistort_down,
                        const state_ikfom& state, double lidar_end_time) {
    const PointCloudXYZI::Ptr cloud(scan_dense_pub_en ? feats_undistort : feats_undistort_down);
    PointCloudXYZI body_cloud(cloud->size(), 1);
    for (std::size_t i = 0; i < cloud->size(); ++i)
        pointLidarToIMU(state, &cloud->points[i], &body_cloud.points[i]);
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(body_cloud, message);
    message.header.stamp = ros::Time().fromSec(lidar_end_time);
    message.header.frame_id = "body";
    publisher.publish(message);
}

/** @brief Publishes the current scan in the native LiDAR frame. */
void publish_frame_lidar(const ros::Publisher& publisher, bool scan_dense_pub_en,
                         const PointCloudXYZI::Ptr& feats_undistort, const PointCloudXYZI::Ptr& feats_undistort_down,
                         double lidar_end_time) {
    const PointCloudXYZI::Ptr cloud(scan_dense_pub_en ? feats_undistort : feats_undistort_down);
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(*cloud, message);
    message.header.stamp = ros::Time().fromSec(lidar_end_time);
    message.header.frame_id = "lidar";
    publisher.publish(message);
}

/** @brief Publishes a local-map snapshot. */
void publish_map_snapshot(const ros::Publisher& publisher, const PointCloudXYZI& map_cloud, double stamp) {
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(map_cloud, message);
    message.header.stamp = ros::Time().fromSec(stamp);
    message.header.frame_id = "camera_init";
    publisher.publish(message);
}

/** @brief Publishes odometry and the world/body transforms. */
void publish_odometry(const ros::Publisher& publisher, nav_msgs::Odometry& odometry, const state_ikfom& state,
                      const esekfom::esekf<state_ikfom, 12, input_ikfom>& kf, const ImuProcess& imu_process,
                      double lidar_end_time, const geometry_msgs::Quaternion& quaternion) {
    odometry.header.frame_id = "camera_init";
    odometry.child_frame_id = "body";
    odometry.header.stamp = ros::Time().fromSec(lidar_end_time);
    set_posestamp(odometry.pose, state, quaternion);
    const auto covariance = kf.get_P();
    for (int i = 0; i < 6; ++i) {
        const int k = i < 3 ? i + 3 : i - 3;
        for (int j = 0; j < 6; ++j)
            odometry.pose.covariance[i * 6 + j] = covariance(k, j < 3 ? j + 3 : j - 3);
    }
    publisher.publish(odometry);
    static tf::TransformBroadcaster body_broadcaster;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(state.pos(0), state.pos(1), state.pos(2)));
    q.setX(quaternion.x);
    q.setY(quaternion.y);
    q.setZ(quaternion.z);
    q.setW(quaternion.w);
    transform.setRotation(q);
    body_broadcaster.sendTransform(tf::StampedTransform(transform, odometry.header.stamp, "camera_init", "body"));
    static tf::TransformBroadcaster world_broadcaster;
    transform.setOrigin(tf::Vector3(0, 0, 0));
    q.setValue(imu_process.Initial_R_wrt_G.x(), imu_process.Initial_R_wrt_G.y(), imu_process.Initial_R_wrt_G.z(),
               imu_process.Initial_R_wrt_G.w());
    transform.setRotation(q);
    world_broadcaster.sendTransform(tf::StampedTransform(transform, odometry.header.stamp, "world", "camera_init"));
}

/** @brief Appends the current pose to a trajectory vector. */
void record_pose(std::vector<std::vector<double>>& poses, double lidar_end_time, const state_ikfom& state,
                 const geometry_msgs::Quaternion& quaternion) {
    poses.push_back({lidar_end_time, state.pos(0), state.pos(1), state.pos(2), quaternion.w, quaternion.x, quaternion.y,
                     quaternion.z});
}

/** @brief Publishes the accumulated path and records the current pose. */
void publish_path(const ros::Publisher& publisher, nav_msgs::Path& path, geometry_msgs::PoseStamped& body_pose,
                  std::vector<std::vector<double>>& poses, double lidar_end_time, const state_ikfom& state,
                  const geometry_msgs::Quaternion& quaternion) {
    set_posestamp(body_pose, state, quaternion);
    body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    body_pose.header.frame_id = "camera_init";
    path.header.stamp = body_pose.header.stamp;
    path.poses.push_back(body_pose);
    publisher.publish(path);
    record_pose(poses, lidar_end_time, state, quaternion);
}

/** @brief Returns the output filename prefix for a selected map backend. */
std::string result_file_stem(const std::string& backend_name) {
    if (backend_name == "voxelmap_plus")
        return "pv_lio_plus";
    if (backend_name == "ikdtree")
        return "pv_lio_ikdtree";
    if (backend_name == "ivox")
        return "pv_lio_ivox";
    if (backend_name == "c3p_voxelmap")
        return "pv_lio_c3p_voxelmap";
    return "pv_lio";
}

/** @brief Reports runtime statistics and persists trajectory and point-cloud results. */
void save_results(std::size_t converged_count, std::size_t not_converged_count,
                  const std::vector<std::vector<double>>& poses, const std::string& root_dir,
                  pv_lio_plus::MapType map_type, bool pcd_save_en, const PointCloudXYZI& pcl_wait_save) {
    std::printf("[INFO}: converged %zu, not converged %zu \n", converged_count, not_converged_count);

    if (!poses.empty()) {
        const std::string stem = result_file_stem(pv_lio_plus::MapTypeName(map_type));
        const std::string path = root_dir + stem + "_pos.txt";
        std::ofstream output(path);
        output << std::fixed;
        for (const auto& pose : poses) {
            for (const double value : pose) {
                output << value << "\t";
            }
            output << "\n";
        }
        std::cout << "save trajectory to " << path << "\n";
    }

    if (pcd_save_en && !pcl_wait_save.empty()) {
        const std::string stem = result_file_stem(pv_lio_plus::MapTypeName(map_type));
        const std::string path = root_dir + stem + ".pcd";
        pcl::io::savePCDFileBinary(path, pcl_wait_save);
        std::cout << "save point cloud to " << path << "\n";
    }
}
