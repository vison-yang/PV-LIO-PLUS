// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <Python.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver/CustomMsg.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <so3_math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <unistd.h>

#include <Eigen/Core>
#include <csignal>
#include <fstream>
#include <mutex>
#include <thread>

#include "IMU_Processing.hpp"
#include "preprocess.h"
#include "voxel_map_util.hpp"
#include "voxelmapplus_util.hpp"

#define INIT_TIME       (0.1)
#define LASER_POINT_COV (0.001)
#define MAXN            (720000)
#define PUBFRAME_PERIOD (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool time_sync_en = false, extrinsic_est_en = true, path_pub_en = true;
double lidar_time_offset = 0.0;
/**************************/

/* -------------------------------------------------------------------------- */
/*                              global variables                              */
/* -------------------------------------------------------------------------- */
float res_last[100000]    = {0.0};
float DET_RANGE           = 300.0f;
const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_surf_min = 0;
double total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_index = 0;
bool point_selected_surf[100000] = {0};
bool lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool scan_pub_en = false, scan_dense_pub_en = false, scan_body_pub_en = false, scan_lidar_pub_en = false;

vector<vector<int>> pointSearchInd_surf;
vector<PointVector> Nearest_Points;
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
deque<double> time_buffer;
deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

PointCloudXYZI::Ptr feats_map(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort_down(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;
std::vector<M3D> var_down_lidar;

pcl::VoxelGrid<PointType> downSizeFilterSurf;

std::vector<float> nn_dist_in_feats;
std::vector<float> nn_plane_std;
PointCloudXYZI::Ptr feats_with_correspondence(new PointCloudXYZI());

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

// params for voxel mapping algorithm
double plannar_threshold = 0.003;
int max_layer            = 0;  // 4

int max_cov_points_size = 50;
int max_points_size     = 50;
double sigma_num        = 2.0;
double voxel_size       = 1.0;
std::vector<int> layer_size;

double ranging_cov = 0.0;
double angle_cov   = 0.0;
std::vector<double> layer_point_size;

bool publish_voxel_map      = false;
int publish_max_voxel_layer = 0;

std::unordered_map<voxel_map_ns::VOXEL_LOC, voxel_map_ns::OctoTree *> voxel_map;
std::unordered_map<voxel_map_plus_ns::VOXEL_LOC, voxel_map_plus_ns::UnionFindNode *> voxel_map_plus;
bool b_use_voxelmap_plus = false;

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

// verbose: time check
size_t nScanCount = 0;

double search_time = 0.;
double eseikf_time = 0.;
double update_time = 0.;
double total_time  = 0.;

double avg_search_time = 0.;
double avg_eseikf_time = 0.;
double avg_update_time = 0.;
double avg_total_time  = 0.;

/* -------------------------------------------------------------------------- */

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

const bool var_contrast(voxel_map_ns::pointWithCov &x, voxel_map_ns::pointWithCov &y)
{
    return (x.cov_world.diagonal().norm() < y.cov_world.diagonal().norm());
};
const bool var_contrast_plus(voxel_map_plus_ns::pointWithCov &x, voxel_map_plus_ns::pointWithCov &y)
{
    return (x.cov_world.diagonal().norm() < y.cov_world.diagonal().norm());
};

/* -------------------------------------------------------------------------- */
/*                           coordinates conversion                           */
/* -------------------------------------------------------------------------- */

// no use，实际上就是需要参数指定 state_point 对象
void pointBodyToWorld_ikfom(PointType const *const pi, PointType *const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

    po->x         = p_global(0);
    po->y         = p_global(1);
    po->z         = p_global(2);
    po->intensity = pi->intensity;
}

void pointLidarToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x         = p_global(0);
    po->y         = p_global(1);
    po->z         = p_global(2);
    po->intensity = pi->intensity;
}

template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x         = p_global(0);
    po->y         = p_global(1);
    po->z         = p_global(2);
    po->intensity = pi->intensity;
}

void pointLidarToIMU(PointType const *const pi, PointType *const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

    po->x         = p_body_imu(0);
    po->y         = p_body_imu(1);
    po->z         = p_body_imu(2);
    po->intensity = pi->intensity;

    po->curvature = pi->curvature;
    po->normal_x  = pi->normal_x;
}

/* -------------------------------------------------------------------------- */
/*                            msg callback function                           */
/* -------------------------------------------------------------------------- */

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    auto time_offset = lidar_time_offset;
    //    std::printf("lidar offset:%f\n", lidar_time_offset);
    mtx_buffer.lock();
    scan_count++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() + time_offset < last_timestamp_lidar)
    {
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
bool timediff_set_flg         = false;
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg       = true;
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

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    publish_count++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp =
            ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, ignoring!!!");
        ROS_WARN("current T: %f, last T: %f", timestamp, last_timestamp_imu);
        return;
    }

    //! 剔除异常数据，这里与 fast-lio2 不一样
    if (std::abs(msg->angular_velocity.x) > 10
        || std::abs(msg->angular_velocity.y) > 10
        || std::abs(msg->angular_velocity.z) > 10)
    {
        ROS_WARN("Large IMU measurement!!! Drop Data!!! %.3f  %.3f  %.3f",
                 msg->angular_velocity.x,
                 msg->angular_velocity.y,
                 msg->angular_velocity.z);
        return;
    }

    //    // 如果是第一帧 拿过来做重力对齐
    //    // TODO 用多帧平均的重力
    //    if (is_first_imu) {
    //        double acc_vec[3] = {msg_in->linear_acceleration.x, msg_in->linear_acceleration.y, msg_in->linear_acceleration.z};
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
int scan_num               = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty())
    {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed)
    {
        meas.lidar          = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1)  // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }
        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
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

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

void publish_frame_world(const ros::Publisher &pubLaserCloudFull)
{
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(scan_dense_pub_en ? feats_undistort : feats_undistort_down);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI laserCloudWorld;
        for (int i = 0; i < size; i++)
        {
            PointType const *const p = &laserCloudFullRes->points[i];
            if (p->intensity < 5)
            {
                continue;
            }
            PointType p_world;

            RGBpointBodyToWorld(p, &p_world);
            laserCloudWorld.push_back(p_world);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp    = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;

        // note: add to output
        {
            *pcl_wait_save += laserCloudWorld;
        }
        // note: add to output
    }
}

void publish_frame_body(const ros::Publisher &_pub)
{
    //    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudFullRes(scan_dense_pub_en ? feats_undistort : feats_undistort_down);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++)
    {
        pointLidarToIMU(&laserCloudFullRes->points[i],
                        &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp    = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    _pub.publish(laserCloudmsg);
    // publish_count -= PUBFRAME_PERIOD;
}

void publish_frame_lidar(const ros::Publisher &_pub)
{
    //    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudFullRes(scan_dense_pub_en ? feats_undistort : feats_undistort_down);
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudFullRes, laserCloudmsg);
    laserCloudmsg.header.stamp    = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "lidar";
    _pub.publish(laserCloudmsg);
    // publish_count -= PUBFRAME_PERIOD;
}

void publish_map(const ros::Publisher &pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*feats_map, laserCloudMap);
    laserCloudMap.header.stamp    = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template <typename T>
void set_posestamp(T &out)
{
    out.pose.position.x    = state_point.pos(0);
    out.pose.position.y    = state_point.pos(1);
    out.pose.position.z    = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
}

void publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id  = "body";
    odomAftMapped.header.stamp    = ros::Time().fromSec(lidar_end_time);  // ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i++)
    {
        int k                                    = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                    odomAftMapped.pose.pose.position.y,
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "body"));

    static tf::TransformBroadcaster br_world;
    transform.setOrigin(tf::Vector3(0, 0, 0));
    q.setValue(p_imu->Initial_R_wrt_G.x(), p_imu->Initial_R_wrt_G.y(), p_imu->Initial_R_wrt_G.z(), p_imu->Initial_R_wrt_G.w());
    transform.setRotation(q);
    br_world.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "world", "camera_init"));
}

std::vector<std::vector<double>> vec_poses;
void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp    = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 1 == 0)
    {
        path.header.stamp = msg_body_pose.header.stamp;
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }

    // note: add to output
    vec_poses.push_back({lidar_end_time, state_point.pos(0), state_point.pos(1), state_point.pos(2),
                         geoQuat.w, geoQuat.x, geoQuat.y, geoQuat.z});
    // note: add to output
}

/* -------------------------------------------------------------------------- */
/*                          coordinates conversion 2                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief lidar 系转到 world 系，实际上等效于循环调用 pointBodyToWorld()
 * @param state_point
 * @param input_cloud lidar
 * @param trans_cloud world
 */
void transformLidar2World(const state_ikfom &state_point, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
    trans_cloud->clear();
    trans_cloud->reserve(input_cloud->size());
    for (size_t i = 0; i < input_cloud->size(); i++)
    {
        pcl::PointXYZINormal p_c = input_cloud->points[i];
        Eigen::Vector3d p_lidar(p_c.x, p_c.y, p_c.z);
        Eigen::Vector3d p_world = state_point.rot * (state_point.offset_R_L_I * p_lidar + state_point.offset_T_L_I) + state_point.pos;

        PointType pi;
        pi.x         = p_world(0);
        pi.y         = p_world(1);
        pi.z         = p_world(2);
        pi.intensity = p_c.intensity;
        trans_cloud->points.push_back(pi);
    }
}

M3D transformLidarCovToWorld(Eigen::Vector3d &p_lidar, const esekfom::esekf<state_ikfom, 12, input_ikfom> &kf,
                             const Eigen::Matrix3d &COV_lidar)
{
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(p_lidar);
    auto state = kf.get_x();

    // lidar到body的方差传播
    // 注意外参的var是先rot 后pos
    M3D il_rot_var = kf.get_P().block<3, 3>(6, 6);
    M3D il_t_var   = kf.get_P().block<3, 3>(9, 9);

    M3D COV_body = state.offset_R_L_I * COV_lidar * state.offset_R_L_I.conjugate()
                   + state.offset_R_L_I * (-point_crossmat) * il_rot_var * (-point_crossmat).transpose() * state.offset_R_L_I.conjugate()
                   + il_t_var;

    // body的坐标
    V3D p_body = state.offset_R_L_I * p_lidar + state.offset_T_L_I;

    // body到world的方差传播
    // 注意pose的var是先pos 后rot
    point_crossmat << SKEW_SYM_MATRX(p_body);
    M3D rot_var = kf.get_P().block<3, 3>(3, 3);
    M3D t_var   = kf.get_P().block<3, 3>(0, 0);

    // Eq. (3)
    M3D COV_world = state.rot * COV_body * state.rot.conjugate()
                    + state.rot * (-point_crossmat) * rot_var * (-point_crossmat).transpose() * state.rot.conjugate()
                    + t_var;

    return COV_world;
    // Voxel map 真实实现
    //    M3D cov_world = R_body * COV_lidar * R_body.conjugate() +
    //          (-point_crossmat) * rot_var * (-point_crossmat).transpose() + t_var;
}

/* -------------------------------------------------------------------------- */
/*                                   model                                    */
/* -------------------------------------------------------------------------- */

void observation_model_share(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    feats_with_correspondence->clear();
    total_residual = 0.0;

    //* 1.用最新的位姿将点云转换到world地图系
    PointCloudXYZI::Ptr feats_world(new PointCloudXYZI);
    transformLidar2World(s, feats_undistort_down, feats_world);

    //* 2.body系转world系
    vector<voxel_map_ns::pointWithCov> pv_list(feats_undistort_down->size());
    for (size_t i = 0; i < feats_undistort_down->size(); i++)
    {
        // note: 每次迭代时，body系下的点云 pos 和 cov 不变
        voxel_map_ns::pointWithCov &pv = pv_list[i];
        pv.point_lidar << feats_undistort_down->points[i].x, feats_undistort_down->points[i].y, feats_undistort_down->points[i].z;
        pv.point_world << feats_world->points[i].x, feats_world->points[i].y, feats_world->points[i].z;
        pv.cov_lidar = var_down_lidar[i];
        pv.cov_world = transformLidarCovToWorld(pv.point_lidar, kf, pv.cov_lidar);
        // pv_list[i] = pv;
    }

    ros::WallTime t0 = ros::WallTime::now();
    //* 3.查找最近点，计算残差
    std::vector<voxel_map_ns::ptpl> ptpl_list;
    std::vector<V3D> non_match_list;
    voxel_map_ns::BuildResidualListOMP(voxel_map, voxel_size, 3.0, max_layer,
                                       pv_list, ptpl_list, non_match_list);
    search_time += (ros::WallTime::now() - t0).toSec() * 1000;

    //* 4.计算用于构建法方程的各矩阵向量，Computation of Measuremnt Jacobian matrix H and measurents vector
    // TODO 为什么不加上状态量对状态量误差的导数？？？？像quaternion那本书？
    effct_feat_num = ptpl_list.size();
    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);  //< 观测值对状态量的导数，点面距离只和位姿、外参有关，对其他状态量的导数都是0
    ekfom_data.h.resize(effct_feat_num);
    ekfom_data.R.resize(effct_feat_num, 1);  // note: 把R作为向量 用的时候转换成diag

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < effct_feat_num; i++)
    {
        voxel_map_ns::ptpl &ptpl_i = ptpl_list[i];  // note: modified

        //* 点坐标及其反对称矩阵、法向量
        V3D point_this_be(ptpl_i.point);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);

        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        V3D norm_vec(ptpl_i.normal);

        //! 计算 Measuremnt Jacobian matrix H，即 h_x
        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec.x(), norm_vec.y(), norm_vec.z(),
                VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            // 不估计外参，不需要求后六位的导数
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec.x(), norm_vec.y(), norm_vec.z(),
                VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        //! 计算 estimate measurement，其几何意义是 distance to the closest surface/corner
        ekfom_data.h(i) = -(norm_vec.dot(ptpl_i.point_world) + ptpl_i.d);

        // 这四行在 build_single_residual() 中有重复，可在 ptpl 中作记录以避免重复计算
        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq.block<1, 3>(0, 0) = ptpl_i.point_world - ptpl_i.center;
        J_nq.block<1, 3>(0, 3) = -ptpl_i.normal;
        double sigma_l         = J_nq * ptpl_i.plane_cov * J_nq.transpose();

        // HACK 1. 因为是标量 所以求逆直接用1除
        // HACK 2. 不同分量的方差用加法来合成 因为公式(12)中的Sigma是对角阵，逐元素运算之后就是对角线上的项目相加
        // M3D cov         = s.rot * s.offset_R_L_I * ptpl_i.cov_lidar * s.offset_R_L_I.conjugate() * s.rot.conjugate();
        M3D cov         = ptpl_i.cov_world;
        ekfom_data.R(i) = 1.0 / (sigma_l + norm_vec.transpose() * cov * norm_vec);
    }

    res_mean_last = total_residual / effct_feat_num;  //? 未使用，total_residual 未计算
}

void observation_model_share_plus(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    total_residual = 0.0;

    //* 1.用最新的位姿将点云转换到world地图系
    PointCloudXYZI::Ptr feats_world(new PointCloudXYZI);
    transformLidar2World(s, feats_undistort_down, feats_world);

    //* 2.body系转world系
    std::vector<voxel_map_plus_ns::pointWithCov> pv_list(feats_undistort_down->size());
    for (size_t i = 0; i < feats_undistort_down->size(); i++)
    {
        // note: 每次迭代时，body系下的点云 pos 和 cov 不变
        voxel_map_plus_ns::pointWithCov &pv = pv_list[i];
        pv.point_lidar << feats_undistort_down->points[i].x, feats_undistort_down->points[i].y, feats_undistort_down->points[i].z;
        pv.point_world << feats_world->points[i].x, feats_world->points[i].y, feats_world->points[i].z;
        Eigen::Matrix3d &cov_lidar = var_down_lidar[i];
        pv.cov_world               = transformLidarCovToWorld(pv.point_lidar, kf, cov_lidar);
    }

    ros::WallTime t0 = ros::WallTime::now();
    //* 3.查找最近点，计算残差
    std::vector<voxel_map_plus_ns::ptpl> ptpl_list;
    std::vector<V3D> non_match_list;
    voxel_map_plus_ns::BuildResidualListOMP(voxel_map_plus, pv_list, ptpl_list, non_match_list);
    search_time += (ros::WallTime::now() - t0).toSec() * 1000;

    //* 4.计算用于构建法方程的各矩阵向量，Computation of Measuremnt Jacobian matrix H and measurents vector
    // TODO 为什么不加上状态量对状态量误差的导数？？？？像quaternion那本书？
    effct_feat_num = ptpl_list.size();
    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points!");
        return;
    }
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);  //< 观测值对状态量的导数，点面距离只和位姿、外参有关，对其他状态量的导数都是0
    ekfom_data.h.resize(effct_feat_num);
    ekfom_data.R.resize(effct_feat_num, 1);  // note: 把R作为向量 用的时候转换成diag

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < effct_feat_num; i++)
    {
        voxel_map_plus_ns::ptpl &ptpl_i = ptpl_list[i];  // note: modified

        //* 点坐标及其反对称矩阵、法向量
        V3D point_this_be(ptpl_i.point);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        V3D norm_vec(ptpl_i.omega / ptpl_i.omega_norm);

        //! 计算 Measuremnt Jacobian matrix H，即 h_x
        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D A(point_crossmat * s.rot.conjugate() * norm_vec);  // 与 voxelmap_plus 一致
        {
            // 不估计外参，不需要求后六位的导数
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec.x(), norm_vec.y(), norm_vec.z(),
                VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        //! 计算 estimate measurement，其几何意义是 distance to the closest surface/corner
        //! NOTE 赋值给 ekfom_data.h(i) 前需先强制转换为 float，否则误差会随着每次转角逐渐累积，导致最终漂移
        ekfom_data.h(i) = -float(ptpl_i.dist);

        Eigen::Matrix<double, 1, 3> J_abd;
        Eigen::Matrix<double, 1, 3> J_pw;
        V3D pw     = ptpl_i.point_world;
        double tmp = ptpl_i.dist / (ptpl_i.omega_norm * ptpl_i.omega_norm);
        if (ptpl_list[i].main_direction == 0)
        {  // Plane equation: ax+by+z+d = 0
            J_abd << pw(0) - ptpl_i.omega[0] * tmp, pw(1) - ptpl_i.omega[1] * tmp, 1;
        }
        else if (ptpl_list[i].main_direction == 1)
        {  // Plane equation: ax+y+bz+d = 0
            J_abd << pw(0) - ptpl_i.omega[0] * tmp, pw(2) - ptpl_i.omega[2] * tmp, 1;
        }
        else
        {  // Plane equation: x+ay+bz+d = 0
            J_abd << pw(1) - ptpl_i.omega[1] * tmp, pw(2) - ptpl_i.omega[2] * tmp, 1;
        }
        J_abd /= ptpl_i.omega_norm;
        double sigma_l = J_abd * ptpl_i.plane_cov * J_abd.transpose();
        J_pw           = ptpl_i.omega.transpose() / ptpl_i.omega_norm;

        // note: 原始 voxelmap++ 中，ptpl_i.point_cov 处使用 lidar 系的 cov，这是导致算法不稳定的一个原因
        ekfom_data.R(i) = 1.0 / (sigma_l + J_pw * ptpl_i.point_cov * J_pw.transpose());
    }

    res_mean_last = total_residual / effct_feat_num;  //? 未使用，total_residual 未计算
}

/* -------------------------------------------------------------------------- */
/*                                    main                                    */
/* -------------------------------------------------------------------------- */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    //* ---------------------------------- Init ---------------------------------- */
    {
        nh.param<double>("time_offset", lidar_time_offset, 0.0);

        nh.param<bool>("publish/path_en", path_pub_en, true);
        nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
        nh.param<bool>("publish/dense_publish_en", scan_dense_pub_en, true);
        nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, true);
        nh.param<bool>("publish/scan_lidarframe_pub_en", scan_lidar_pub_en, true);
        nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
        nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
        nh.param<bool>("common/time_sync_en", time_sync_en, false);

        // mapping algorithm params
        nh.param<float>("mapping/det_range", DET_RANGE, 300.f);
        nh.param<int>("mapping/max_iteration", NUM_MAX_ITERATIONS, 4);
        nh.param<int>("mapping/max_points_size", max_points_size, 100);
        nh.param<int>("mapping/max_cov_points_size", max_cov_points_size, 100);
        nh.param<vector<double>>("mapping/layer_point_size", layer_point_size, vector<double>());
        nh.param<int>("mapping/max_layer", max_layer, 2);
        nh.param<double>("mapping/voxel_size", voxel_size, 1.0);
        nh.param<double>("mapping/down_sample_size", filter_size_surf_min, 0.5);
        nh.param<double>("mapping/plannar_threshold", plannar_threshold, 0.01);
        nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
        nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
        nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
        nh.param<int>("mapping/update_size_threshold", voxel_map_plus_ns::update_size_threshold, 5);
        nh.param<double>("mapping/sigma_num", sigma_num, 3);
        nh.param<bool>("mapping/b_use_voxelmap_plus", b_use_voxelmap_plus, false);
        std::cout << "filter_size_surf_min:" << filter_size_surf_min << std::endl;
        voxel_map_plus_ns::sigma_num        = static_cast<int>(sigma_num);
        voxel_map_plus_ns::max_points_size  = max_points_size;
        voxel_map_plus_ns::voxel_size       = voxel_size;
        voxel_map_plus_ns::planer_threshold = plannar_threshold;
        voxel_map_plus_ns::quater_length    = voxel_size / 4;

        // noise model params
        nh.param<double>("noise_model/ranging_cov", ranging_cov, 0.02);
        nh.param<double>("noise_model/angle_cov", angle_cov, 0.05);
        nh.param<double>("noise_model/gyr_cov", gyr_cov, 0.1);
        nh.param<double>("noise_model/acc_cov", acc_cov, 0.1);
        nh.param<double>("noise_model/b_gyr_cov", b_gyr_cov, 0.0001);
        nh.param<double>("noise_model/b_acc_cov", b_acc_cov, 0.0001);

        // visualization params
        nh.param<bool>("publish/pub_voxel_map", publish_voxel_map, false);
        nh.param<int>("publish/publish_max_voxel_layer", publish_max_voxel_layer, 0);

        nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
        nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
        nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
        nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
        nh.param<int>("preprocess/point_filter_num", p_pre->point_filter_num, 1);
        nh.param<bool>("preprocess/feature_extract_enable", p_pre->feature_enabled, false);
        cout << "p_pre->lidar_type " << p_pre->lidar_type << endl;

        for (size_t i = 0; i < layer_point_size.size(); i++)
        {
            layer_size.push_back(layer_point_size[i]);
        }
    }

    path.header.stamp    = ros::Time::now();
    path.header.frame_id = "camera_init";

    _featsArray.reset(new PointCloudXYZI());
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    // imu 参数设置
    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    double epsi[23] = {0.001};
    fill(epsi, epsi + 23, 0.001);
    if (b_use_voxelmap_plus)
        kf.init_dyn_share(get_f, df_dx, df_dw, observation_model_share_plus, NUM_MAX_ITERATIONS, epsi);
    else
        kf.init_dyn_share(get_f, df_dx, df_dw, observation_model_share, NUM_MAX_ITERATIONS, epsi);

    // 消息订阅器、发布器
    ros::Subscriber sub_pcl                = p_pre->lidar_type == AVIA ? nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu                = nh.subscribe(imu_topic, 200000, imu_cbk);
    ros::Publisher pubLaserCloudFull       = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_body  = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudFull_lidar = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_lidar", 100000);
    ros::Publisher pubLaserCloudEffect     = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap        = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped        = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    ros::Publisher pubExtrinsic            = nh.advertise<nav_msgs::Odometry>("/Extrinsic", 100000);
    ros::Publisher pubPath                 = nh.advertise<nav_msgs::Path>("/path", 100000);
    ros::Publisher voxel_map_pub           = nh.advertise<visualization_msgs::MarkerArray>("/planes", 10000);

    //! -------------------------------- main loop ------------------------------- */

    bool init_map          = false;  // for Plane Map
    size_t n_converged     = 0;
    size_t n_not_converged = 0;

    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit)
            break;
        ros::spinOnce();

        ros::WallTime t0 = ros::WallTime::now();

        //! get data
        if (sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                first_lidar_time        = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan          = false;
                continue;
            }

            //* -------------------------------------------------------------------------- */
            //*                                start process                               */
            //* -------------------------------------------------------------------------- */

            //* step1. IMU 传播解算，Lidar 点云运动畸变校正（同 Fast-lio2）
            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

            //! step2. ekf初始化成功，尝试初始化 voxelmap
            if (flg_EKF_inited && !init_map)
            {
                ros::WallTime t1 = ros::WallTime::now();

                // body 转 world
                PointCloudXYZI::Ptr feats_world(new PointCloudXYZI);
                transformLidar2World(state_point, feats_undistort, feats_world);
                if (b_use_voxelmap_plus)
                {
                    std::vector<voxel_map_plus_ns::pointWithCov> pv_list(feats_undistort->size());
                    for (size_t i = 0; i < feats_world->size(); i++)
                    {
                        //? modified：统一了 pointWithCov 中成员的使用
                        voxel_map_plus_ns::pointWithCov &pv = pv_list[i];
                        pv.point_lidar << feats_undistort->points[i].x, feats_undistort->points[i].y, feats_undistort->points[i].z;
                        pv.point_world << feats_world->points[i].x, feats_world->points[i].y, feats_world->points[i].z;

                        pv.cov_lidar = voxel_map_plus_ns::calcLidarCov(pv.point_lidar, ranging_cov, angle_cov);
                        pv.cov_world = transformLidarCovToWorld(pv.point_lidar, kf, pv.cov_lidar);  // var 从 lidar 系转换到 world 系
                    }
                    voxel_map_plus_ns::BuildVoxelMap(pv_list, voxel_map_plus);
                }
                else
                {  // 计算首帧所有点的 covariance 并用于构建初始地图
                    std::vector<voxel_map_ns::pointWithCov> pv_list(feats_undistort->size());
                    for (size_t i = 0; i < feats_world->size(); i++)
                    {
                        //? modified：统一了 pointWithCov 中成员的使用
                        voxel_map_ns::pointWithCov &pv = pv_list[i];
                        pv.point_lidar << feats_undistort->points[i].x, feats_undistort->points[i].y, feats_undistort->points[i].z;
                        pv.point_world << feats_world->points[i].x, feats_world->points[i].y, feats_world->points[i].z;

                        // if z=0, error will occur in calcBodyCov. To be solved
                        if (pv.point_lidar[2] == 0)
                        {
                            pv.point_lidar[2] = 0.001;
                        }
                        pv.cov_lidar = voxel_map_ns::calcLidarCov(pv.point_lidar, ranging_cov, angle_cov);
                        pv.cov_world = transformLidarCovToWorld(pv.point_lidar, kf, pv.cov_lidar);  // var 从 lidar 系转换到 world 系
                    }
                    voxel_map_ns::buildVoxelMap(pv_list, voxel_size, max_layer, layer_size,
                                                max_points_size, max_cov_points_size, plannar_threshold, voxel_map);
                }

                double map_build_time = (ros::WallTime::now() - t1).toSec();
                if (b_use_voxelmap_plus)
                    std::cout << std::fixed << "[Init Map] Build voxel map plus: " << map_build_time * 1000 << "s\n";
                else
                    std::cout << std::fixed << "[Init Map] Build voxel map: " << map_build_time * 1000 << "s \n";

                init_map = true;
                continue;
            }

            //* step3. 单帧降采样并按时间排序
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_undistort_down);
            sort(feats_undistort_down->points.begin(), feats_undistort_down->points.end(), time_list);
            feats_down_size = feats_undistort_down->points.size();

            //! step4. 体坐标系下各点的协方差阵，迭代时不变，可复用
            var_down_lidar.clear();
            if (b_use_voxelmap_plus)
            {
                for (auto &pt : feats_undistort_down->points)
                {
                    //? 不需要和上边一样处理 pt.z？
                    V3D point_this(pt.x, pt.y, pt.z);
                    var_down_lidar.push_back(voxel_map_plus_ns::calcLidarCov(point_this, ranging_cov, angle_cov));
                }
            }
            else
            {
                for (auto &pt : feats_undistort_down->points)
                {
                    //? 不需要和上边一样处理 pt.z？
                    V3D point_this(pt.x, pt.y, pt.z);
                    var_down_lidar.push_back(voxel_map_ns::calcLidarCov(point_this, ranging_cov, angle_cov));
                }
            }

            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            //! --------------------------------- kernel --------------------------------- */
            //! step 5.迭代误差扩展卡尔曼滤波器解算
            /*** ICP and iterated state Kalman filter update ***/
            search_time      = 0.;
            ros::WallTime t2 = ros::WallTime::now();
            bool bConverged  = kf.update_iterated_dyn_share_diagonal();
            eseikf_time      = (ros::WallTime::now() - t2).toSec() * 1000;
            // kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            //! --------------------------------- kernel --------------------------------- */

            //* ------------------------------ post process ------------------------------ */

            //* step 6.从 kf 中提取结果
            state_point = kf.get_x();
            geoQuat.x   = state_point.rot.coeffs()[0];
            geoQuat.y   = state_point.rot.coeffs()[1];
            geoQuat.z   = state_point.rot.coeffs()[2];
            geoQuat.w   = state_point.rot.coeffs()[3];

            ros::WallTime t3 = ros::WallTime::now();
            //! step 7.更新地图 voxelmap，add the points to the voxel map
            PointCloudXYZI::Ptr feats_world(new PointCloudXYZI);
            transformLidar2World(state_point, feats_undistort_down, feats_world);
            // 用最新的状态估计将点及点的covariance转换到world系
            if (b_use_voxelmap_plus)
            {
                std::vector<voxel_map_plus_ns::pointWithCov> pv_list(feats_undistort_down->size());
                for (size_t i = 0; i < feats_undistort_down->size(); i++)
                {
                    voxel_map_plus_ns::pointWithCov &pv = pv_list[i];
                    pv.point_lidar << feats_undistort_down->points[i].x, feats_undistort_down->points[i].y, feats_undistort_down->points[i].z;
                    pv.point_world << feats_world->points[i].x, feats_world->points[i].y, feats_world->points[i].z;
                    pv.cov_lidar = var_down_lidar[i];
                    pv.cov_world = transformLidarCovToWorld(pv.point_lidar, kf, pv.cov_lidar);
                }
                std::sort(pv_list.begin(), pv_list.end(), var_contrast_plus);
                voxel_map_plus_ns::UpdateVoxelMap(pv_list, voxel_map_plus);
            }
            else
            {
                std::vector<voxel_map_ns::pointWithCov> pv_list(feats_undistort_down->size());
                for (size_t i = 0; i < feats_undistort_down->size(); i++)
                {
                    //* 保存body系和world系坐标及其协方差阵，最终updateVoxelMap需要用的是world系的point
                    // note: 由于 voxelmap 本身的变量使用混乱，原代码这里给 point 和 cov 赋值，这里统一改回来
                    // note: 注意这个在每次迭代时是存在重复计算的 因为lidar系的点云covariance是不变的
                    //? 这里错误的使用世界系的点来calcBodyCov时，反倒在某些seq（比如hilti2022的03 15）上效果更好
                    //?     需要考虑是不是init_plane时使用更大的cov更好
                    voxel_map_ns::pointWithCov &pv = pv_list[i];
                    pv.point_lidar << feats_undistort_down->points[i].x, feats_undistort_down->points[i].y, feats_undistort_down->points[i].z;
                    pv.point_world << feats_world->points[i].x, feats_world->points[i].y, feats_world->points[i].z;
                    pv.cov_lidar = var_down_lidar[i];
                    pv.cov_world = transformLidarCovToWorld(pv.point_lidar, kf, pv.cov_lidar);
                }
                std::sort(pv_list.begin(), pv_list.end(), var_contrast);
                voxel_map_ns::updateVoxelMapOMP(pv_list, voxel_size, max_layer, layer_size,
                                                max_points_size, max_cov_points_size, plannar_threshold, voxel_map);
            }
            update_time = (ros::WallTime::now() - t3).toSec() * 1000;

            // std::printf("BA: %.4f %.4f %.4f   \nBG: %.4f %.4f %.4f   \ng: %.4f %.4f %.4f\n",
            //             state_point.ba.x(), state_point.ba.y(), state_point.ba.z(),
            //             state_point.bg.x(), state_point.bg.y(), state_point.bg.z(),
            //             state_point.grav.get_vect().x(), state_point.grav.get_vect().y(), state_point.grav.get_vect().z());
            avg_eseikf_time = avg_eseikf_time * (nScanCount / double(nScanCount + 1)) + eseikf_time / (nScanCount + 1);
            avg_search_time = avg_search_time * (nScanCount / double(nScanCount + 1)) + search_time / (nScanCount + 1);
            avg_update_time = avg_update_time * (nScanCount / double(nScanCount + 1)) + update_time / (nScanCount + 1);
            avg_total_time  = avg_total_time * (nScanCount / double(nScanCount + 1)) + total_time / (nScanCount + 1);
            nScanCount++;

            //* step 8.可视化，发布odometry, path, cloud
            publish_odometry(pubOdomAftMapped);
            if (path_pub_en)
                publish_path(pubPath);
            if (scan_pub_en)
                publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en)
                publish_frame_body(pubLaserCloudFull_body);
            if (scan_pub_en && scan_lidar_pub_en)
                publish_frame_lidar(pubLaserCloudFull_lidar);
            if (publish_voxel_map)
            {
                if (b_use_voxelmap_plus)
                    voxel_map_plus_ns::pubVoxelMap(voxel_map_plus, voxel_map_pub);
                else
                    voxel_map_ns::pubVoxelMap(voxel_map, publish_max_voxel_layer, voxel_map_pub);
            }
            // publish_effect_world(pubLaserCloudEffect);
            // publish_map(pubLaserCloudMap);

            total_time = (ros::WallTime::now() - t0).toSec() * 1000;

            // verbose
            printf("[%4.4f] Pos: %3.3f %3.3f %3.3f %u\n",
                   lidar_end_time - first_lidar_time,
                   state_point.pos(0), state_point.pos(1), state_point.pos(2), bConverged);
            if (bConverged)
                ++n_converged;
            else
                ++n_not_converged;
            // printf("Time(ms): eseikf %3.3lf, search %3.3lf, update %3.3lf, total %3.3lf \n",
            //        eseikf_time, search_time, update_time, total_time);
        }

        status = ros::ok();
        rate.sleep();
    }

    printf("[INFO}: converged %ld, not converged %ld \n", n_converged, n_not_converged);
    if (nScanCount)
    {
        printf("[FINAL] Time(ms): eseikf %3.3lf, search %3.3lf, update %3.3lf, total %3.3lf \n",
               avg_eseikf_time, avg_search_time, avg_update_time, avg_total_time);
    }

    //* 输出结果
    if (pcl_wait_save->size() > 0 && vec_poses.size() > 0)
    {
        std::string fpath_poses = std::string(root_dir) + (b_use_voxelmap_plus ? "pv_lio_plus_pos.txt" : "pv_lio_pos.txt");
        std::ofstream ofs(fpath_poses);
        ofs << std::fixed;
        for (auto pose : vec_poses)
        {
            for (int j = 0; j < 8; ++j)
            {
                ofs << pose[j] << "\t";
            }
            ofs << "\n";
        }
        ofs.close();

        std::string fpath_cloud = std::string(root_dir) + (b_use_voxelmap_plus ? "pv_lio_plus.pcd" : "pv_lio.pcd");
        pcl::io::savePCDFileBinary(fpath_cloud, *pcl_wait_save);

        std::cout << "save results to " << fpath_poses << " and " << fpath_cloud << "\n";
    }

    return 0;
}
