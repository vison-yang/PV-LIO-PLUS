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

/*
 * PV-LIO-PLUS modifications retain the LOAM/Livox BSD-3-Clause notice above.
 * Local changes include MapManager dispatch, selectable local-map backends,
 * backend-specific residual construction, and result-file naming.
 * Modified: 2026-08-09; license notice updated: 2026-08-16.
 * See LICENSE and THIRD_PARTY_NOTICES.md.
 */

/**
 * @file mapping.cpp
 * @brief PV-LIO-PLUS node, observation models, and map-manager integration.
 */

#include "mapping.hpp"

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
#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <system_error>
#include <thread>

#include "imu_processing.hpp"
#include "map_manager/hvox_observation.h"
#include "map_manager/map_manager.h"
#include "map_manager/point_plane_jacobian.h"
#include "map_manager/r_voxelmap_observation.h"
#include "preprocess.h"
#include "utils.hpp"
// The native PV map headers define non-inline functions.  Keep the manager
// implementation in this translation unit to avoid changing those headers.
#include "map_manager/map_manager.cpp"
#include "map_manager/native/voxelmap/voxel_map_util.hpp"
#include "map_manager/native/voxelmap_plus/voxelmapplus_util.hpp"

// Keep backend implementation and configuration in the same translation unit.
// The native map headers contain non-inline definitions by design.
#include "config.cpp"

constexpr double kInitTime = 0.1;

namespace pv_lio_plus
{
namespace
{
using TimingClock = std::chrono::steady_clock;
static_assert(TimingClock::is_steady, "Elapsed-time measurements require a monotonic clock");

double ElapsedMilliseconds(const TimingClock::time_point start)
{
    return std::chrono::duration<double, std::milli>(TimingClock::now() - start).count();
}

// The shared PCL cloud is float. i-Octree's query contract keeps the current
// world transform in double and quantizes map insertions once at its boundary.
void RestoreIOctreeWorldPoints(const state_ikfom& state, MapPointList& points)
{
    for (auto& point : points)
        point.point_world = state.rot * (state.offset_R_L_I * point.point_lidar + state.offset_T_L_I) + state.pos;
}
}  // namespace

Mapping* Mapping::active_instance_ = nullptr;

/** @brief Creates the node-owned mapping state. */
Mapping::Mapping() : map_manager_(new pv_lio_plus::MapManager)
{
    active_instance_ = this;
}

/** @brief Releases the active mapping instance. */
Mapping::~Mapping()
{
    CloseTimingLog();
    if (active_instance_ == this)
    {
        active_instance_ = nullptr;
    }
}

/** @brief Bridges IKFoM's raw callback to the active Mapping instance. */
void Mapping::ObservationModelTrampoline(state_ikfom& state, esekfom::dyn_share_datastruct<double>& ekfom_data)
{
    if (active_instance_ != nullptr)
    {
        active_instance_->ObservationModel(state, ekfom_data);
    }
    else
    {
        ekfom_data.valid = false;
    }
}

/** @brief Transforms a LiDAR-frame cloud into the world frame. */
void Mapping::transformLidar2World(const state_ikfom& state_point_, const PointCloudXYZI::Ptr& input_cloud,
                                   PointCloudXYZI::Ptr& trans_cloud) const
{
    trans_cloud->clear();
    trans_cloud->reserve(input_cloud->size());
    for (std::size_t i = 0; i < input_cloud->size(); ++i)
    {
        pcl::PointXYZINormal p_c = input_cloud->points[i];
        Eigen::Vector3d p_lidar(p_c.x, p_c.y, p_c.z);
        Eigen::Vector3d p_world =
            state_point_.rot * (state_point_.offset_R_L_I * p_lidar + state_point_.offset_T_L_I) + state_point_.pos;

        PointType pi;
        pi.x         = p_world(0);
        pi.y         = p_world(1);
        pi.z         = p_world(2);
        pi.intensity = p_c.intensity;
        trans_cloud->points.push_back(pi);
    }
}

/** @brief Propagates LiDAR point covariance through extrinsics and pose. */
M3D Mapping::transformLidarCovToWorld(Eigen::Vector3d& p_lidar, const esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_,
                                      const Eigen::Matrix3d& COV_lidar) const
{
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(p_lidar);
    auto state = kf_.get_x();

    // lidar到body的方差传播
    // 注意外参的var是先rot 后pos
    M3D il_rot_var = kf_.get_P().block<3, 3>(6, 6);
    M3D il_t_var   = kf_.get_P().block<3, 3>(9, 9);

    M3D COV_body = state.offset_R_L_I * COV_lidar * state.offset_R_L_I.conjugate() +
                   state.offset_R_L_I * (-point_crossmat) * il_rot_var * (-point_crossmat).transpose() *
                       state.offset_R_L_I.conjugate() +
                   il_t_var;

    // body的坐标
    V3D p_body = state.offset_R_L_I * p_lidar + state.offset_T_L_I;

    // body到world的方差传播
    // 注意pose的var是先pos 后rot
    point_crossmat << SKEW_SYM_MATRX(p_body);
    M3D rot_var = kf_.get_P().block<3, 3>(3, 3);
    M3D t_var   = kf_.get_P().block<3, 3>(0, 0);

    // Eq. (3)
    M3D COV_world = state.rot * COV_body * state.rot.conjugate() +
                    state.rot * (-point_crossmat) * rot_var * (-point_crossmat).transpose() * state.rot.conjugate() +
                    t_var;

    return COV_world;
    // Voxel map 真实实现
    //    M3D cov_world = R_body * COV_lidar * R_body.conjugate() +
    //          (-point_crossmat) * rot_var * (-point_crossmat).transpose() + t_var;
}

/**
 * @brief Converts synchronized LiDAR/world clouds to manager points.
 * @param points_lidar Points in the current LiDAR frame.
 * @param points_world The same points transformed to the world frame.
 * @param cov_lidar LiDAR-frame covariance for each point, when available.
 * @return Eigen-aligned points accepted by @ref pv_lio_plus::MapManager.
 */
std::vector<MapPoint, Eigen::aligned_allocator<MapPoint>> Mapping::make_manager_points(
    const PointCloudXYZI::Ptr& points_lidar, const PointCloudXYZI::Ptr& points_world,
    const std::vector<M3D>& cov_lidar) const
{
    const std::size_t point_count = std::min(points_lidar->size(), points_world->size());
    pv_lio_plus::MapPointList result;
    result.reserve(point_count);
    for (std::size_t i = 0; i < point_count; ++i)
    {
        pv_lio_plus::MapPoint point;
        point.point_lidar << points_lidar->points[i].x, points_lidar->points[i].y, points_lidar->points[i].z;
        point.point_world << points_world->points[i].x, points_world->points[i].y, points_world->points[i].z;
        if (i < cov_lidar.size())
        {
            point.cov_lidar = cov_lidar[i];
            V3D lidar_point = point.point_lidar;
            point.cov_world = transformLidarCovToWorld(lidar_point, kf_, point.cov_lidar);
        }
        result.emplace_back(point);
    }
    return result;
}

/**
 * @brief Computes LiDAR-frame covariance for the selected PV backend.
 * @param point Point coordinates in the LiDAR frame.
 * @return Covariance using the configured ranging and angular noise.
 */
/** @brief Computes sensor covariance using the selected backend convention. */
M3D Mapping::calc_lidar_cov_for_backend(const V3D& point) const
{
    V3D safe_point = point;
    if (safe_point.z() == 0.0)
    {
        safe_point.z() = 0.001;
    }
    if (config_.map_manager_config().type == pv_lio_plus::MapType::VoxelMapPlus)
    {
        return voxel_map_plus_ns::calcLidarCov(safe_point, config_.ranging_cov_, config_.angle_cov_);
    }
    return voxel_map_ns::calcLidarCov(safe_point, config_.ranging_cov_, config_.angle_cov_);
}

/* -------------------------------------------------------------------------- */
/*                                   model                                    */
/* -------------------------------------------------------------------------- */

/** @brief Builds the native VoxelMap residual model. */
void Mapping::ObservationModelVoxelMap(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data)
{
    feats_with_correspondence_->clear();
    total_residual_ = 0.0;

    //* 1.用最新的位姿将点云转换到world地图系
    PointCloudXYZI::Ptr feats_world(new PointCloudXYZI);
    transformLidar2World(s, feats_undistort_down_, feats_world);

    //* 2.body系转world系
    std::vector<voxel_map_ns::pointWithCov> pv_list(feats_undistort_down_->size());
    for (std::size_t i = 0; i < feats_undistort_down_->size(); ++i)
    {
        // note: 每次迭代时，body系下的点云 pos 和 cov 不变
        voxel_map_ns::pointWithCov& pv = pv_list[i];
        pv.point_lidar << feats_undistort_down_->points[i].x, feats_undistort_down_->points[i].y,
            feats_undistort_down_->points[i].z;
        pv.point_world << feats_world->points[i].x, feats_world->points[i].y, feats_world->points[i].z;
        pv.cov_lidar = var_down_lidar_[i];
        pv.cov_world = transformLidarCovToWorld(pv.point_lidar, kf_, pv.cov_lidar);
        // pv_list[i] = pv;
    }

    const auto t0 = TimingClock::now();
    //* 3.查找最近点，计算残差
    std::vector<voxel_map_ns::ptpl> ptpl_list;
    std::vector<V3D> non_match_list;
    map_manager_->search(pv_list, ptpl_list, non_match_list);
    search_time_ += ElapsedMilliseconds(t0);

    //* 4.计算用于构建法方程的各矩阵向量，Computation of Measuremnt Jacobian matrix H and measurents vector
    // TODO 为什么不加上状态量对状态量误差的导数？？？？像quaternion那本书？
    effective_feature_num_ = ptpl_list.size();
    if (effective_feature_num_ < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }
    ekfom_data.h_x = MatrixXd::Zero(effective_feature_num_,
                                    12);  //< 观测值对状态量的导数，点面距离只和位姿、外参有关，对其他状态量的导数都是0
    ekfom_data.h.resize(effective_feature_num_);
    ekfom_data.R.resize(effective_feature_num_, 1);  // note: 把R作为向量 用的时候转换成diag

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < effective_feature_num_; i++)
    {
        voxel_map_ns::ptpl& ptpl_i = ptpl_list[i];  // note: modified

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
        if (config_.extrinsic_est_en_)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec.x(), norm_vec.y(), norm_vec.z(), VEC_FROM_ARRAY(A),
                VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            // 不估计外参，不需要求后六位的导数
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec.x(), norm_vec.y(), norm_vec.z(), VEC_FROM_ARRAY(A), 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0;
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

    res_mean_last_ = total_residual_ / effective_feature_num_;  //? 未使用，total_residual_ 未计算
}

/** @brief Builds the native VoxelMapPlus residual model. */
void Mapping::ObservationModelVoxelMapPlus(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data)
{
    total_residual_ = 0.0;

    //* 1.用最新的位姿将点云转换到world地图系
    PointCloudXYZI::Ptr feats_world(new PointCloudXYZI);
    transformLidar2World(s, feats_undistort_down_, feats_world);

    //* 2.body系转world系
    std::vector<voxel_map_plus_ns::pointWithCov> pv_list(feats_undistort_down_->size());
    for (std::size_t i = 0; i < feats_undistort_down_->size(); ++i)
    {
        // note: 每次迭代时，body系下的点云 pos 和 cov 不变
        voxel_map_plus_ns::pointWithCov& pv = pv_list[i];
        pv.point_lidar << feats_undistort_down_->points[i].x, feats_undistort_down_->points[i].y,
            feats_undistort_down_->points[i].z;
        pv.point_world << feats_world->points[i].x, feats_world->points[i].y, feats_world->points[i].z;
        Eigen::Matrix3d& cov_lidar = var_down_lidar_[i];
        pv.cov_world               = transformLidarCovToWorld(pv.point_lidar, kf_, cov_lidar);
    }

    const auto t0 = TimingClock::now();
    //* 3.查找最近点，计算残差
    std::vector<voxel_map_plus_ns::ptpl> ptpl_list;
    std::vector<V3D> non_match_list;
    map_manager_->search(pv_list, ptpl_list, non_match_list);
    search_time_ += ElapsedMilliseconds(t0);

    //* 4.计算用于构建法方程的各矩阵向量，Computation of Measuremnt Jacobian matrix H and measurents vector
    // TODO 为什么不加上状态量对状态量误差的导数？？？？像quaternion那本书？
    effective_feature_num_ = ptpl_list.size();
    if (effective_feature_num_ < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points!");
        return;
    }
    ekfom_data.h_x = MatrixXd::Zero(effective_feature_num_,
                                    12);  //< 观测值对状态量的导数，点面距离只和位姿、外参有关，对其他状态量的导数都是0
    ekfom_data.h.resize(effective_feature_num_);
    ekfom_data.R.resize(effective_feature_num_, 1);  // note: 把R作为向量 用的时候转换成diag

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < effective_feature_num_; i++)
    {
        voxel_map_plus_ns::ptpl& ptpl_i = ptpl_list[i];  // note: modified

        //* 点坐标及其反对称矩阵、法向量
        const V3D norm_vec(ptpl_i.omega / ptpl_i.omega_norm);
        // ptpl_i.point is in the LiDAR frame. IKFoM applies right rotation
        // perturbations to both s.rot and s.offset_R_L_I.
        ekfom_data.h_x.block<1, 12>(i, 0) = VoxelMapPlusPointPlaneJacobian(
            ptpl_i.point, norm_vec, s.rot, s.offset_R_L_I, s.offset_T_L_I, config_.extrinsic_est_en_);

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

    res_mean_last_ = total_residual_ / effective_feature_num_;  //? 未使用，total_residual_ 未计算
}

/**
 * @brief Builds EKF residuals from ikd-tree or iVox matches.
 * @param s Current iterated filter state.
 * @param ekfom_data EKF measurement Jacobian, residual, and weight outputs.
 */
void Mapping::ObservationModelPointBackend(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data)
{
    total_residual_ = 0.0;

    PointCloudXYZI::Ptr feats_world(new PointCloudXYZI);
    if (config_.map_manager_config().type == MapType::IOctree)
        feats_world = feats_undistort_down_;  // Temporary values; world coordinates are restored below.
    else
        transformLidar2World(s, feats_undistort_down_, feats_world);
    pv_lio_plus::MapPointList map_points = make_manager_points(feats_undistort_down_, feats_world, var_down_lidar_);
    if (config_.map_manager_config().type == MapType::IOctree) RestoreIOctreeWorldPoints(s, map_points);

    const auto t0 = TimingClock::now();
    pv_lio_plus::PlaneMatchList matches;
    std::vector<V3D> non_match_list;
    try
    {
        map_manager_->search(map_points, matches, non_match_list);
    }
    catch (const std::exception& exception)
    {
        if (config_.map_manager_config().type != MapType::IOctree) throw;
        search_time_ += ElapsedMilliseconds(t0);
        effective_feature_num_ = 0;
        ekfom_data.valid = false;
        ROS_WARN_THROTTLE(1.0, "i-Octree query rejected: %s", exception.what());
        return;
    }
    search_time_ += ElapsedMilliseconds(t0);

    effective_feature_num_ = static_cast<int>(matches.size());
    if (effective_feature_num_ < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points!");
        return;
    }

    ekfom_data.h_x = MatrixXd::Zero(effective_feature_num_, 12);
    ekfom_data.h.resize(effective_feature_num_);
    ekfom_data.R.resize(effective_feature_num_, 1);

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < effective_feature_num_; ++i)
    {
        const pv_lio_plus::PlaneMatch& match = matches[static_cast<std::size_t>(i)];
        const V3D point_this_be              = match.point;
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);

        const V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        V3D norm_vec      = match.normal;
        const double norm = norm_vec.norm();
        if (norm > std::numeric_limits<double>::epsilon())
        {
            norm_vec /= norm;
        }

        const V3D C = s.rot.conjugate() * norm_vec;
        const V3D A = point_crossmat * C;
        if (config_.extrinsic_est_en_)
        {
            const V3D B = point_be_crossmat * s.offset_R_L_I.conjugate() * C;
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec.x(), norm_vec.y(), norm_vec.z(), VEC_FROM_ARRAY(A),
                VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec.x(), norm_vec.y(), norm_vec.z(), VEC_FROM_ARRAY(A), 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0;
        }

        ekfom_data.h(i) = -match.distance;

        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq.block<1, 3>(0, 0)   = match.point_world - match.center;
        J_nq.block<1, 3>(0, 3)   = -match.normal;
        const double sigma_l     = (J_nq * match.plane_cov * J_nq.transpose())(0, 0);
        const double sigma_point = (norm_vec.transpose() * match.point_cov * norm_vec)(0, 0);
        double total_sigma       = sigma_l + sigma_point;
        if (!std::isfinite(total_sigma) || total_sigma <= 1e-9)
        {
            total_sigma = 1e-9;
        }
        ekfom_data.R(i) = 1.0 / total_sigma;
    }

    if (config_.map_manager_config().type == MapType::IOctree &&
        (!ekfom_data.h_x.allFinite() || !ekfom_data.h.allFinite() || !ekfom_data.R.allFinite()))
    {
        effective_feature_num_ = 0;
        ekfom_data.valid = false;
        return;
    }
    // Accumulate diagnostics in row order after the parallel observation loop.
    for (int i = 0; i < effective_feature_num_; ++i)
        total_residual_ += std::abs(ekfom_data.h(i));
    res_mean_last_ = total_residual_ / effective_feature_num_;
}

/**
 * @brief Builds Super-LIO OctVox residuals with its fixed information weight.
 *
 * MapManager owns the native OctVox TopK lookup and its local five-neighbor
 * plane/range gates.  Unlike generic point-map observations, this model keeps
 * Super-LIO's constant measurement information of 1000.
 */
void Mapping::ObservationModelOctVox(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data)
{
    total_residual_ = 0.0;

    PointCloudXYZI::Ptr feats_world(new PointCloudXYZI);
    transformLidar2World(s, feats_undistort_down_, feats_world);

    const std::size_t point_count = std::min(feats_undistort_down_->size(), feats_world->size());
    pv_lio_plus::MapPointList map_points;
    map_points.reserve(point_count);
    for (std::size_t index = 0; index < point_count; ++index)
    {
        pv_lio_plus::MapPoint point;
        point.point_lidar << feats_undistort_down_->points[index].x, feats_undistort_down_->points[index].y,
            feats_undistort_down_->points[index].z;
        point.point_world << feats_world->points[index].x, feats_world->points[index].y, feats_world->points[index].z;
        map_points.emplace_back(point);
    }

    const auto t0 = TimingClock::now();
    pv_lio_plus::PlaneMatchList matches;
    std::vector<V3D> non_match_list;
    map_manager_->search(map_points, matches, non_match_list);
    search_time_ += ElapsedMilliseconds(t0);

    effective_feature_num_ = static_cast<int>(matches.size());
    if (effective_feature_num_ < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points!");
        return;
    }

    ekfom_data.h_x = MatrixXd::Zero(effective_feature_num_, 12);
    ekfom_data.h.resize(effective_feature_num_);
    ekfom_data.R.resize(effective_feature_num_, 1);

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int index = 0; index < effective_feature_num_; ++index)
    {
        const pv_lio_plus::PlaneMatch& match = matches[static_cast<std::size_t>(index)];
        const V3D point_this_be              = match.point;
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);

        const V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        // OctVox's native plane builder normalizes this vector before the
        // residual gate; do not replace it with a covariance-weighted model.
        const V3D& norm_vec = match.normal;
        const V3D C         = s.rot.conjugate() * norm_vec;
        const V3D A         = point_crossmat * C;
        if (config_.extrinsic_est_en_)
        {
            const V3D B = point_be_crossmat * s.offset_R_L_I.conjugate() * C;
            ekfom_data.h_x.block<1, 12>(index, 0) << norm_vec.x(), norm_vec.y(), norm_vec.z(), VEC_FROM_ARRAY(A),
                VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(index, 0)
                << norm_vec.x(), norm_vec.y(), norm_vec.z(), VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        ekfom_data.h(index) = -match.distance;
        ekfom_data.R(index) = 1000.0;
    }

    for (int i = 0; i < effective_feature_num_; ++i)
        total_residual_ += std::abs(ekfom_data.h(i));
    res_mean_last_ = total_residual_ / effective_feature_num_;
}

/**
 * @brief Builds EKF residuals from native C3P-VoxelMap matches.
 * @param s Current iterated filter state.
 * @param ekfom_data EKF measurement Jacobian, residual, and weight outputs.
 */
void Mapping::ObservationModelC3P(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data)
{
    total_residual_ = 0.0;

    PointCloudXYZI::Ptr feats_world(new PointCloudXYZI);
    transformLidar2World(s, feats_undistort_down_, feats_world);

    const std::size_t point_count = std::min(feats_undistort_down_->size(), feats_world->size());
    pv_lio_plus::MapPointList map_points;
    map_points.reserve(point_count);
    for (std::size_t i = 0; i < point_count; ++i)
    {
        pv_lio_plus::MapPoint point;
        point.point_lidar << feats_undistort_down_->points[i].x, feats_undistort_down_->points[i].y,
            feats_undistort_down_->points[i].z;
        point.point_world << feats_world->points[i].x, feats_world->points[i].y, feats_world->points[i].z;
        if (i < var_down_lidar_.size())
        {
            point.cov_lidar = var_down_lidar_[i];
            point.cov_world = transformLidarCovToWorld(point.point_lidar, kf_, point.cov_lidar);
        }
        map_points.emplace_back(point);
    }

    const auto t0 = TimingClock::now();
    pv_lio_plus::PlaneMatchList matches;
    std::vector<V3D> non_match_list;
    map_manager_->search(map_points, matches, non_match_list);
    search_time_ += ElapsedMilliseconds(t0);

    effective_feature_num_ = static_cast<int>(matches.size());
    if (effective_feature_num_ < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points!");
        return;
    }

    ekfom_data.h_x = MatrixXd::Zero(effective_feature_num_, 12);
    ekfom_data.h.resize(effective_feature_num_);
    ekfom_data.R.resize(effective_feature_num_, 1);

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < effective_feature_num_; ++i)
    {
        const pv_lio_plus::PlaneMatch& match = matches[static_cast<std::size_t>(i)];
        const V3D point_this_be              = match.point;
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);

        const V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        const V3D& norm_vec = match.normal;
        const V3D C         = s.rot.conjugate() * norm_vec;
        const V3D A         = point_crossmat * C;
        if (config_.extrinsic_est_en_)
        {
            const V3D B = point_be_crossmat * s.offset_R_L_I.conjugate() * C;
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec.x(), norm_vec.y(), norm_vec.z(), VEC_FROM_ARRAY(A),
                VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec.x(), norm_vec.y(), norm_vec.z(), VEC_FROM_ARRAY(A), 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0;
        }

        const double signed_point_plane_residual = norm_vec.dot(match.point_world) + match.d;
        ekfom_data.h(i)                          = -signed_point_plane_residual;

        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq.block<1, 3>(0, 0)   = match.point_world - match.center;
        J_nq.block<1, 3>(0, 3)   = -norm_vec;
        const double sigma_plane = (J_nq * match.plane_cov * J_nq.transpose())(0, 0);
        const double sigma_point = (norm_vec.transpose() * match.point_cov * norm_vec)(0, 0);
        ekfom_data.R(i)          = 1.0 / (sigma_plane + sigma_point);
    }

    for (int i = 0; i < effective_feature_num_; ++i)
        total_residual_ += std::abs(ekfom_data.h(i));
    res_mean_last_ = total_residual_ / effective_feature_num_;
}

/** Builds R-VoxelMap observations with its native measurement-noise model. */
void Mapping::ObservationModelRVoxelMap(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data)
{
    total_residual_ = 0.0;
    PointCloudXYZI::Ptr feats_world(new PointCloudXYZI);
    transformLidar2World(s, feats_undistort_down_, feats_world);
    const MapPointList points = make_manager_points(feats_undistort_down_, feats_world, var_down_lidar_);

    const auto start = TimingClock::now();
    PlaneMatchList matches;
    std::vector<V3D> non_match;
    map_manager_->search(points, matches, non_match);
    search_time_ += ElapsedMilliseconds(start);

    std::vector<RVoxelMapMeasurement, Eigen::aligned_allocator<RVoxelMapMeasurement>> measurements;
    measurements.reserve(matches.size());
    for (const PlaneMatch &match : matches)
    {
        RVoxelMapMeasurement measurement;
        if (MakeRVoxelMapMeasurement(match.point, match.point_world, match.normal, match.center, match.d,
                                     match.plane_cov, match.lidar_cov, s.rot, s.offset_R_L_I, s.offset_T_L_I,
                                     config_.extrinsic_est_en_, measurement))
        {
            total_residual_ += std::abs(measurement.residual);
            measurements.push_back(measurement);
        }
    }
    effective_feature_num_ = static_cast<int>(measurements.size());
    if (effective_feature_num_ == 0)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points!");
        return;
    }
    ekfom_data.h_x.resize(effective_feature_num_, 12);
    ekfom_data.h.resize(effective_feature_num_);
    ekfom_data.R.resize(effective_feature_num_, 1);
    for (int i = 0; i < effective_feature_num_; ++i)
    {
        const RVoxelMapMeasurement &measurement = measurements[static_cast<std::size_t>(i)];
        ekfom_data.h_x.row(i) = measurement.jacobian;
        ekfom_data.h(i) = measurement.residual;
        ekfom_data.R(i) = measurement.information;
    }
    res_mean_last_ = total_residual_ / effective_feature_num_;
}

/** hvox_pko_ikfom: fixed surfels within a group, current-state signed residuals. */
void Mapping::ObservationModelHVox(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data)
{
    effective_feature_num_ = 0;
    total_residual_ = 0.0;
    if (!hvox_observations_)
    {
        ekfom_data.valid = false;
        return;
    }

    if (hvox_observations_->NeedsRefresh(ekfom_data.converge, map_manager_->revision()))
    {
        MapPointList points;
        points.reserve(feats_undistort_down_->size());
        for (const auto &source : feats_undistort_down_->points)
        {
            MapPoint point;
            point.point_lidar = V3D(source.x, source.y, source.z);
            point.point_world = s.rot * (s.offset_R_L_I * point.point_lidar + s.offset_T_L_I) + s.pos;
            points.push_back(point);
        }
        PlaneMatchList matches;
        std::vector<V3D> non_match;
        const auto start = TimingClock::now();
        try
        {
            map_manager_->search(points, matches, non_match);
        }
        catch (const std::exception &exception)
        {
            search_time_ += ElapsedMilliseconds(start);
            hvox_observations_->BeginFrame();
            ekfom_data.valid = false;
            ROS_WARN_THROTTLE(1.0, "hVox match rejected: %s", exception.what());
            return;
        }
        search_time_ += ElapsedMilliseconds(start);
        HVoxCorrespondences group;
        group.reserve(matches.size());
        for (const auto &match : matches)
        {
            HVoxCorrespondence correspondence;
            correspondence.point_lidar = match.point;
            correspondence.normal_world = match.normal;
            correspondence.center_world = match.center;
            group.push_back(correspondence);
        }
        if (!hvox_observations_->Refresh(group, s.rot, s.offset_R_L_I, s.offset_T_L_I, s.pos,
                                         map_manager_->revision()))
        {
            ekfom_data.valid = false;
            return;
        }
    }

    HVoxObservationRows rows;
    if (!hvox_observations_->Evaluate(s.rot, s.offset_R_L_I, s.offset_T_L_I, s.pos, config_.extrinsic_est_en_,
                                      config_.map_manager_config().hvox_lidar_noise_std, rows))
    {
        ekfom_data.valid = false;
        return;
    }
    effective_feature_num_ = static_cast<int>(rows.size());
    ekfom_data.h_x.resize(effective_feature_num_, 12);
    ekfom_data.h.resize(effective_feature_num_);
    ekfom_data.R.resize(effective_feature_num_, 1);
    for (int i = 0; i < effective_feature_num_; ++i)
    {
        const auto &row = rows[static_cast<std::size_t>(i)];
        ekfom_data.h_x.row(i) = row.jacobian;
        ekfom_data.h(i) = row.residual;
        ekfom_data.R(i) = row.information;
        total_residual_ += std::abs(row.residual);
    }
    res_mean_last_ = total_residual_ / effective_feature_num_;
}

/**
 * @brief Dispatches the EKF observation model to the selected map backend.
 * @param s Current iterated filter state.
 * @param ekfom_data EKF measurement Jacobian, residual, and weight outputs.
 */
void Mapping::ObservationModel(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data)
{
    if (config_.map_manager_config().type == pv_lio_plus::MapType::VoxelMap)
    {
        ObservationModelVoxelMap(s, ekfom_data);
    }
    else if (config_.map_manager_config().type == pv_lio_plus::MapType::VoxelMapPlus)
    {
        ObservationModelVoxelMapPlus(s, ekfom_data);
    }
    else if (config_.map_manager_config().type == pv_lio_plus::MapType::C3PVoxelMap)
    {
        ObservationModelC3P(s, ekfom_data);
    }
    else if (config_.map_manager_config().type == pv_lio_plus::MapType::OctVox)
    {
        ObservationModelOctVox(s, ekfom_data);
    }
    else if (config_.map_manager_config().type == pv_lio_plus::MapType::RVoxelMap)
    {
        ObservationModelRVoxelMap(s, ekfom_data);
    }
    else if (config_.map_manager_config().type == pv_lio_plus::MapType::HVox)
    {
        ObservationModelHVox(s, ekfom_data);
    }
    else
    {
        ObservationModelPointBackend(s, ekfom_data);
    }
}

/** @brief Builds the initial local map after EKF initialization. */
void Mapping::InitializeMap()
{
    const auto t1 = TimingClock::now();

    // Transform the first scan and build the backend-independent point list.
    PointCloudXYZI::Ptr feats_world(new PointCloudXYZI);
    if (config_.map_manager_config().type == MapType::IOctree)
        feats_world = feats_undistort_;
    else
        transformLidar2World(state_point_, feats_undistort_, feats_world);
    std::vector<M3D> cov_lidar;
    cov_lidar.reserve(feats_undistort_->size());
    for (const auto& point : feats_undistort_->points)
    {
        cov_lidar.emplace_back(calc_lidar_cov_for_backend(V3D(point.x, point.y, point.z)));
    }
    MapUpdateContext context;
    context.imu_position_world = V3D(state_point_.pos);
    MapPointList map_points = make_manager_points(feats_undistort_, feats_world, cov_lidar);
    if (config_.map_manager_config().type == MapType::IOctree) RestoreIOctreeWorldPoints(state_point_, map_points);
    try
    {
        map_manager_->initialize(map_points, context);
    }
    catch (const std::exception& exception)
    {
        if (config_.map_manager_config().type != MapType::IOctree) throw;
        ROS_WARN_THROTTLE(1.0, "i-Octree initialization rejected: %s", exception.what());
        return;
    }
    if (hvox_observations_) hvox_observations_->BeginFrame();

    const double map_build_time_ms = ElapsedMilliseconds(t1);
    std::cout << std::fixed << "[Init Map] Build " << pv_lio_plus::MapTypeName(config_.map_manager_config().type)
              << ": " << map_build_time_ms << " ms\n";

    map_initialized_ = true;
}

/** @brief Inserts the current scan into the selected local-map backend. */
void Mapping::UpdateMap()
{
    // Transform the filtered scan once and pass one common point type to MapManager.
    PointCloudXYZI::Ptr feats_world(new PointCloudXYZI);
    if (config_.map_manager_config().type == MapType::IOctree)
        feats_world = feats_undistort_down_;
    else
        transformLidar2World(state_point_, feats_undistort_down_, feats_world);

    MapPointList map_points = make_manager_points(feats_undistort_down_, feats_world, var_down_lidar_);
    if (config_.map_manager_config().type == MapType::IOctree)
    {
        RestoreIOctreeWorldPoints(state_point_, map_points);
        const bool invalid = std::any_of(map_points.begin(), map_points.end(), [](const MapPoint& point) {
            return !point.point_lidar.allFinite() || !point.point_world.allFinite() || !point.cov_world.allFinite();
        });
        if (invalid || (config_.map_manager_config().local_window_enabled &&
                        (!V3D(state_point_.pos).allFinite() || !std::isfinite(config_.det_range_) ||
                         config_.det_range_ <= 0)))
        {
            ROS_WARN_THROTTLE(1.0, "i-Octree update rejected nonfinite point, covariance, or window");
            return;
        }
    }
    if (config_.map_manager_config().type == MapType::RVoxelMap)
    {
        // The native backend rejects nonfinite inputs. Remove them before the
        // covariance sort too, because NaNs do not define a strict ordering.
        map_points.erase(std::remove_if(map_points.begin(), map_points.end(), [](const MapPoint& point) {
                             return !point.point_world.allFinite() || !point.cov_world.allFinite();
                         }),
                         map_points.end());
    }
    if (config_.map_manager_config().type != MapType::HVox)
        std::sort(map_points.begin(), map_points.end(),
                  [](const pv_lio_plus::MapPoint& lhs, const pv_lio_plus::MapPoint& rhs) {
                      return lhs.cov_world.diagonal().norm() < rhs.cov_world.diagonal().norm();
                  });
    MapUpdateContext context;
    context.imu_position_world = V3D(state_point_.pos);
    try
    {
        map_manager_->update(map_points, static_cast<std::uint32_t>(scan_total_count_), context);
        if (config_.map_manager_config().local_window_enabled)
            map_manager_->move_window(state_point_.pos, V3D(config_.det_range_, config_.det_range_, config_.det_range_));
    }
    catch (const std::exception& exception)
    {
        if (config_.map_manager_config().type != MapType::IOctree) throw;
        ROS_WARN_THROTTLE(1.0, "i-Octree update rejected: %s", exception.what());
    }
}

/** @brief Opens the optional per-frame timing CSV after configuration is known. */
void Mapping::OpenTimingLog()
{
    CloseTimingLog();
    if (!config_.timing_log_en_)
    {
        return;
    }

    if (config_.timing_log_path_.empty())
    {
        const std::string stem = result_file_stem(pv_lio_plus::MapTypeName(config_.map_manager_config().type));
        timing_output_path_ = (std::filesystem::path(config_.root_dir_) / (stem + "_timing.csv")).string();
    }
    else
    {
        timing_output_path_ = config_.timing_log_path_;
    }

    const std::filesystem::path output_path(timing_output_path_);
    std::error_code error;
    const std::filesystem::path parent = output_path.parent_path();
    if (!parent.empty())
    {
        std::filesystem::create_directories(parent, error);
    }
    if (error)
    {
        ROS_WARN("Cannot create timing-log directory for %s: %s", timing_output_path_.c_str(), error.message().c_str());
        timing_output_path_.clear();
        return;
    }

    timing_output_.open(timing_output_path_, std::ios::out | std::ios::trunc);
    if (!timing_output_.is_open())
    {
        ROS_WARN("Cannot open timing log: %s", timing_output_path_.c_str());
        timing_output_path_.clear();
        return;
    }

    timing_output_ << "frame,stamp_sec,elapsed_sec,converged,pos_x_m,pos_y_m,pos_z_m,quat_w,quat_x,quat_y,quat_z,"
                   << "search_ms,search_avg_ms,ikf_update_ms,ikf_update_avg_ms,map_update_ms,map_update_avg_ms,"
                   << "total_ms,total_avg_ms,effective_points,map_size_native\n";
    timing_output_ << std::fixed << std::setprecision(6);
    ROS_INFO("Per-frame timing log: %s", timing_output_path_.c_str());
}

/** @brief Emits one pose and its current/mean stage timings without forcing a disk flush. */
void Mapping::WriteTimingLog(double lidar_end_time, bool converged)
{
    if (!config_.timing_log_en_)
    {
        return;
    }

    std::printf("[%4.4f] Pos: %3.3f %3.3f %3.3f %d | ms cur(avg): search %.3f(%.3f), ikf %.3f(%.3f), "
                "update %.3f(%.3f), total %.3f(%.3f)\n",
                lidar_end_time - first_lidar_time_, state_point_.pos(0), state_point_.pos(1), state_point_.pos(2),
                static_cast<int>(converged), search_time_, avg_search_time_, eseikf_time_, avg_eseikf_time_,
                update_time_, avg_update_time_, total_time_, avg_total_time_);

    if (!timing_output_.is_open())
    {
        return;
    }

    timing_output_ << scan_total_count_ << ',' << lidar_end_time << ',' << lidar_end_time - first_lidar_time_ << ','
                   << static_cast<int>(converged) << ',' << state_point_.pos(0) << ',' << state_point_.pos(1) << ','
                   << state_point_.pos(2) << ',' << geo_quat_.w << ',' << geo_quat_.x << ',' << geo_quat_.y << ','
                   << geo_quat_.z << ',' << search_time_ << ',' << avg_search_time_ << ',' << eseikf_time_ << ','
                   << avg_eseikf_time_ << ',' << update_time_ << ',' << avg_update_time_ << ',' << total_time_ << ','
                   << avg_total_time_ << ',' << effective_feature_num_ << ',' << map_manager_->size() << '\n';
}

/** @brief Closes the optional timing log once the single-threaded mapping loop stops. */
void Mapping::CloseTimingLog()
{
    if (timing_output_.is_open())
    {
        timing_output_.close();
    }
    timing_output_path_.clear();
}

/** @brief Runs the configured ROS mapping loop and persists its outputs. */
int Mapping::Run()
{
    ros::NodeHandle nh;

    // 1. Load ROS parameters into the Mapping-owned state.
    if (!config_.Load(nh))
    {
        return 1;
    }

    // 2. Apply backend configuration and initialize the estimator.
    try
    {
        map_manager_->configure(config_.map_manager_config());
    }
    catch (const std::exception& exception)
    {
        ROS_FATAL("Invalid map configuration: %s", exception.what());
        return 1;
    }
    if (!map_manager_->supports_selected_backend())
    {
        ROS_FATAL("Selected map backend is not supported: %s",
                  pv_lio_plus::MapTypeName(config_.map_manager_config().type));
        return 1;
    }
    if (config_.map_manager_config().type == MapType::HVox)
        hvox_observations_ = std::make_unique<HVoxObservationGroup>();
    else
        hvox_observations_.reset();
    OpenTimingLog();

    down_size_filter_surf_.setLeafSize(config_.filter_size_surf_min_, config_.filter_size_surf_min_,
                                       config_.filter_size_surf_min_);

    lidar_t_wrt_imu_ << VEC_FROM_ARRAY(config_.extrin_t_);
    lidar_r_wrt_imu_ << MAT_FROM_ARRAY(config_.extrin_r_);
    imu_process_->set_extrinsic(lidar_t_wrt_imu_, lidar_r_wrt_imu_);
    imu_process_->set_gyr_cov(V3D(config_.gyr_cov_, config_.gyr_cov_, config_.gyr_cov_));
    imu_process_->set_acc_cov(V3D(config_.acc_cov_, config_.acc_cov_, config_.acc_cov_));
    imu_process_->set_gyr_bias_cov(V3D(config_.b_gyr_cov_, config_.b_gyr_cov_, config_.b_gyr_cov_));
    imu_process_->set_acc_bias_cov(V3D(config_.b_acc_cov_, config_.b_acc_cov_, config_.b_acc_cov_));

    double epsi[23] = {0.001};
    std::fill(epsi, epsi + 23, 0.001);
    kf_.init_dyn_share(get_f, df_dx, df_dw, ObservationModelTrampoline, config_.num_max_iterations_, epsi);

    // 3. Register sensor subscribers and output publishers.
    publish_subscribe_.Initialize(nh, config_, config_.preprocess());

    // 4. Process synchronized LiDAR/IMU groups until ROS shuts down.
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    while (ros::ok())
    {
        ros::spinOnce();

        const auto t0 = TimingClock::now();

        if (publish_subscribe_.SyncPackages())
        {
            const MeasureGroup& measures = publish_subscribe_.measures();
            const double lidar_end_time  = publish_subscribe_.lidar_end_time();
            if (first_scan_)
            {
                first_lidar_time_              = measures.lidar_beg_time;
                imu_process_->first_lidar_time = first_lidar_time_;
                first_scan_                    = false;
                continue;
            }

            // Undistort the scan using the IMU propagation step.
            imu_process_->Process(measures, kf_, feats_undistort_);
            state_point_ = kf_.get_x();

            if (feats_undistort_ == nullptr || feats_undistort_->empty())
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            ekf_initialized_ = (measures.lidar_beg_time - first_lidar_time_) < kInitTime ? false : true;

            if (ekf_initialized_ && !map_initialized_)
            {
                InitializeMap();
                continue;
            }

            // Downsample the scan and restore its time ordering.
            down_size_filter_surf_.setInputCloud(feats_undistort_);
            down_size_filter_surf_.filter(*feats_undistort_down_);
            std::sort(feats_undistort_down_->points.begin(), feats_undistort_down_->points.end(),
                      [](const PointType& lhs, const PointType& rhs) { return lhs.curvature < rhs.curvature; });
            feats_down_size_ = feats_undistort_down_->points.size();

            // Precompute LiDAR-frame covariance for the current filtered scan.
            var_down_lidar_.clear();
            var_down_lidar_.reserve(feats_undistort_down_->size());
            for (const auto& pt : feats_undistort_down_->points)
            {
                var_down_lidar_.push_back(calc_lidar_cov_for_backend(V3D(pt.x, pt.y, pt.z)));
            }

            if (feats_down_size_ < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            // Run the iterated EKF observation update.
            if (hvox_observations_) hvox_observations_->BeginFrame();
            search_time_     = 0.;
            const auto t2 = TimingClock::now();
            bool bConverged  = kf_.update_iterated_dyn_share_diagonal();
            eseikf_time_     = ElapsedMilliseconds(t2);
            // Extract the corrected state and update the local map.
            state_point_ = kf_.get_x();
            geo_quat_.x  = state_point_.rot.coeffs()[0];
            geo_quat_.y  = state_point_.rot.coeffs()[1];
            geo_quat_.z  = state_point_.rot.coeffs()[2];
            geo_quat_.w  = state_point_.rot.coeffs()[3];

            const auto t3 = TimingClock::now();
            UpdateMap();
            update_time_ = ElapsedMilliseconds(t3);

            // std::printf("BA: %.4f %.4f %.4f   \nBG: %.4f %.4f %.4f   \ng: %.4f %.4f %.4f\n",
            //             state_point_.ba.x(), state_point_.ba.y(), state_point_.ba.z(),
            //             state_point_.bg.x(), state_point_.bg.y(), state_point_.bg.z(),
            //             state_point_.grav.get_vect().x(), state_point_.grav.get_vect().y(),
            //             state_point_.grav.get_vect().z());
            // Publish the current state and optionally persist scan output.
            const auto map_cloud = config_.map_publish_en_ ? map_manager_->snapshot() : PointCloudXYZI::Ptr();
            publish_subscribe_.PublishFrame(feats_undistort_, feats_undistort_down_, state_point_, kf_, *imu_process_,
                                            geo_quat_, bConverged);
            if (config_.publish_voxel_map_)
                map_manager_->publish_planes(publish_subscribe_.voxel_map_publisher(),
                                             config_.publish_max_voxel_layer_);
            if (map_cloud) publish_subscribe_.PublishMapSnapshot(*map_cloud, lidar_end_time);

            total_time_ = ElapsedMilliseconds(t0);

            const double frame_count = static_cast<double>(scan_total_count_ + 1);
            avg_eseikf_time_ += (eseikf_time_ - avg_eseikf_time_) / frame_count;
            avg_search_time_ += (search_time_ - avg_search_time_) / frame_count;
            avg_update_time_ += (update_time_ - avg_update_time_) / frame_count;
            avg_total_time_ += (total_time_ - avg_total_time_) / frame_count;
            ++scan_total_count_;

            WriteTimingLog(lidar_end_time, bConverged);

            if (!config_.timing_log_en_)
            {
                std::printf("[%4.4f] Pos: %3.3f %3.3f %3.3f %d\n", lidar_end_time - first_lidar_time_,
                            state_point_.pos(0), state_point_.pos(1), state_point_.pos(2),
                            static_cast<int>(bConverged));
            }
        }

        rate.sleep();
    }

    // 5. Persist trajectory and point-cloud results after shutdown.
    CloseTimingLog();
    publish_subscribe_.SaveResults(config_, config_.map_manager_config().type);
    return 0;
}
/** @brief Initializes ROS and runs one node-owned Mapping instance. */
int RunMappingNode(int argc, char** argv)
{
    ros::init(argc, argv, "pv_lio_plus_node");
    Mapping mapping;
    return mapping.Run();
}

}  // namespace pv_lio_plus
