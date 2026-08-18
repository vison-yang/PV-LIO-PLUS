/** @file utils.cpp @brief PV-LIO-PLUS ROS and output helpers. */

#include "utils.hpp"

#include <omp.h>

namespace {
constexpr int kPubFramePeriod = 20;
}

void SigHandle(int sig) {
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}
void RGBpointBodyToWorld(PointType const* const pi, PointType* const po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void pointLidarToIMU(PointType const* const pi, PointType* const po) {
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;

    po->curvature = pi->curvature;
    po->normal_x = pi->normal_x;
}

/* -------------------------------------------------------------------------- */
/*                            msg callback function                           */
/* -------------------------------------------------------------------------- */

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    auto time_offset = lidar_time_offset;
    //    std::printf("lidar offset:%f\n", lidar_time_offset);
    mtx_buffer.lock();
    scan_count++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() + time_offset < last_timestamp_lidar) {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec() + time_offset);
    last_timestamp_lidar = msg->header.stamp.toSec() + time_offset;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar) {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() &&
        !lidar_buffer.empty()) {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu,
               last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 &&
        !imu_buffer.empty()) {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr& msg_in) {
    publish_count++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en) {
        msg->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    if (timestamp < last_timestamp_imu) {
        ROS_WARN("imu loop back, ignoring!!!");
        ROS_WARN("current T: %f, last T: %f", timestamp, last_timestamp_imu);
        return;
    }

    //! 剔除异常数据，这里与 fast-lio2 不一样
    if (std::abs(msg->angular_velocity.x) > 10 || std::abs(msg->angular_velocity.y) > 10 ||
        std::abs(msg->angular_velocity.z) > 10) {
        ROS_WARN("Large IMU measurement!!! Drop Data!!! %.3f  %.3f  %.3f", msg->angular_velocity.x,
                 msg->angular_velocity.y, msg->angular_velocity.z);
        return;
    }

    //    // 如果是第一帧 拿过来做重力对齐
    //    // TODO 用多帧平均的重力
    //    if (is_first_imu) {
    //        double acc_vec[3] = {msg_in->linear_acceleration.x, msg_in->linear_acceleration.y,
    //        msg_in->linear_acceleration.z};
    //
    //        R__world__o__initial = SO3(g2R(Eigen::Vector3d(acc_vec)));
    //
    //        is_first_imu = false;
    //    }

    last_timestamp_imu = timestamp;

    mtx_buffer.lock();

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int scan_num = 0;
bool sync_packages(MeasureGroup& meas) {
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed) {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        } else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime) {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        } else {
            scan_num++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime +=
                (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }
        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time) {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time)
            break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

/* -------------------------------------------------------------------------- */
/*                         publish and record results                         */
/* -------------------------------------------------------------------------- */

/** @brief Reusable cloud buffer for registered-cloud publication. */
PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
/** @brief Accumulated world-frame scan cloud pending PCD output. */
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

/**
 * @brief Publishes the current world-frame scan and optionally saves it.
 * @param pubLaserCloudFull Publisher for `/cloud_registered`.
 *
 * Saved points are accumulated independently from the `/Laser_map` snapshot.
 */
void publish_frame_world(const ros::Publisher& pubLaserCloudFull) {
    if (scan_pub_en) {
        PointCloudXYZI::Ptr laserCloudFullRes(scan_dense_pub_en ? feats_undistort : feats_undistort_down);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI laserCloudWorld;
        for (int i = 0; i < size; i++) {
            PointType const* const p = &laserCloudFullRes->points[i];
            if (p->intensity < 5) {
                continue;
            }
            PointType p_world;

            RGBpointBodyToWorld(p, &p_world);
            laserCloudWorld.push_back(p_world);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= kPubFramePeriod;
    }

    if (pcd_save_en) {
        PointCloudXYZI laserCloudWorld;
        laserCloudWorld.reserve(feats_undistort->size());
        for (const auto& point : feats_undistort->points) {
            PointType point_world;
            RGBpointBodyToWorld(&point, &point_world);
            laserCloudWorld.push_back(point_world);
        }
        *pcl_wait_save += laserCloudWorld;
        ++scan_wait_num;

        if (pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval && !pcl_wait_save->empty()) {
            const std::string pcd_dir = std::string(root_dir) + "PCD";
            std::filesystem::create_directories(pcd_dir);
            const std::string pcd_path = pcd_dir + "/scans_" + std::to_string(++pcd_index) + ".pcd";
            pcl::PCDWriter writer;
            writer.writeBinary(pcd_path, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
            ROS_INFO("Saved point-cloud chunk: %s", pcd_path.c_str());
        }
    }
}

void publish_frame_body(const ros::Publisher& _pub) {
    //    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudFullRes(scan_dense_pub_en ? feats_undistort : feats_undistort_down);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++) {
        pointLidarToIMU(&laserCloudFullRes->points[i], &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    _pub.publish(laserCloudmsg);
    // publish_count -= kPubFramePeriod;
}

void publish_frame_lidar(const ros::Publisher& _pub) {
    //    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudFullRes(scan_dense_pub_en ? feats_undistort : feats_undistort_down);
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudFullRes, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "lidar";
    _pub.publish(laserCloudmsg);
    // publish_count -= kPubFramePeriod;
}

void publish_map_snapshot(const ros::Publisher& pubLaserCloudMap, const PointCloudXYZI& map_cloud, double stamp) {
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(map_cloud, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(stamp);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template <typename T>
void set_posestamp(T& out) {
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
}

void publish_odometry(const ros::Publisher& pubOdomAftMapped) {
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time); // ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i++) {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }
    pubOdomAftMapped.publish(odomAftMapped);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, odomAftMapped.pose.pose.position.y,
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "body"));

    static tf::TransformBroadcaster br_world;
    transform.setOrigin(tf::Vector3(0, 0, 0));
    q.setValue(p_imu->Initial_R_wrt_G.x(), p_imu->Initial_R_wrt_G.y(), p_imu->Initial_R_wrt_G.z(),
               p_imu->Initial_R_wrt_G.w());
    transform.setRotation(q);
    br_world.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "world", "camera_init"));
}
std::vector<std::vector<double>> vec_poses;
/** @brief Appends the current timestamp, position, and orientation to output. */
void record_pose() {
    vec_poses.push_back({lidar_end_time, state_point.pos(0), state_point.pos(1), state_point.pos(2), geoQuat.w,
                         geoQuat.x, geoQuat.y, geoQuat.z});
}

void publish_path(const ros::Publisher pubPath) {
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 1 == 0) {
        path.header.stamp = msg_body_pose.header.stamp;
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }

    record_pose();
}

std::string result_file_stem(const std::string& backend_name) {
    if (backend_name == "voxelmap_plus") {
        return "pv_lio_plus";
    }
    if (backend_name == "ikdtree") {
        return "pv_lio_ikdtree";
    }
    if (backend_name == "ivox") {
        return "pv_lio_ivox";
    }
    if (backend_name == "c3p_voxelmap") {
        return "pv_lio_c3p_voxelmap";
    }
    return "pv_lio";
}
