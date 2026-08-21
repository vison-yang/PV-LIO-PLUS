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

#include "imu_processing.hpp"
#include "map_manager/map_manager.h"
#include "utils.hpp"

#include <Eigen/Core>
#include <Python.h>
#include <algorithm>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <geometry_msgs/Vector3.h>
#include <limits>
#include <livox_ros_driver/CustomMsg.h>
#include <math.h>
#include <mutex>
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
#include <thread>
#include <unistd.h>
// The native PV map headers define non-inline functions.  Keep the manager
// implementation in this translation unit to avoid changing those headers.
#include "map_manager/map_manager.cpp"
#include "map_manager/native/voxelmap/voxel_map_util.hpp"
#include "map_manager/native/voxelmap_plus/voxelmapplus_util.hpp"

constexpr double kInitTime = 0.1;

namespace pv_lio_plus {

Mapping* Mapping::active_instance_ = nullptr;

/** @brief Creates the node-owned mapping state. */
Mapping::Mapping() : map_manager_config_(new pv_lio_plus::MapManagerConfig), map_manager_(new pv_lio_plus::MapManager) {
    active_instance_ = this;
}

/** @brief Releases the active mapping instance. */
Mapping::~Mapping() {
    if (active_instance_ == this) {
        active_instance_ = nullptr;
    }
}

/** @brief Loads ROS parameters directly into this Mapping instance. */
bool Mapping::load_parameters(ros::NodeHandle& nh) {
    nh.param<double>("time_offset", lidar_time_offset_, 0.0);
    nh.param<bool>("publish/path_en", path_pub_en_, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en_, true);
    nh.param<bool>("publish/dense_publish_en", scan_dense_pub_en_, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en_, true);
    nh.param<bool>("publish/scan_lidarframe_pub_en", scan_lidar_pub_en_, true);
    nh.param<std::string>("common/lid_topic", lidar_topic_, "/livox/lidar");
    nh.param<std::string>("common/imu_topic", imu_topic_, "/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en_, false);

    nh.param<float>("mapping/det_range", det_range_, 300.0f);
    nh.param<int>("mapping/max_iteration", num_max_iterations_, 4);
    nh.param<int>("mapping/max_points_size", max_points_size_, 100);
    nh.param<int>("mapping/max_cov_points_size", max_cov_points_size_, 100);
    nh.param<std::vector<double>>("mapping/layer_point_size", layer_point_size_, std::vector<double>());
    nh.param<int>("mapping/max_layer", max_layer_, 2);
    nh.param<double>("mapping/voxel_size", voxel_size_, 1.0);
    nh.param<double>("mapping/down_sample_size", filter_size_surf_min_, 0.5);
    nh.param<double>("mapping/plannar_threshold", plannar_threshold_, 0.01);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en_, true);
    nh.param<std::vector<double>>("mapping/extrinsic_T", extrin_t_, std::vector<double>());
    nh.param<std::vector<double>>("mapping/extrinsic_R", extrin_r_, std::vector<double>());
    nh.param<int>("mapping/update_size_threshold", map_manager_config_->plus_update_size_threshold, 5);
    nh.param<double>("mapping/sigma_num", sigma_num_, 3.0);

    std::string map_type_name;
    nh.param<std::string>("mapping/map_type", map_type_name, "voxelmap");
    try {
        map_manager_config_->type = pv_lio_plus::ParseMapType(map_type_name);
    } catch (const std::exception& exception) {
        ROS_FATAL("Invalid mapping/map_type: %s", exception.what());
        return false;
    }
    nh.param<bool>("mapping/local_window_en", map_manager_config_->local_window_enabled, false);
    nh.param<int>("mapping/nearest_point_count", map_manager_config_->nearest_point_count, NUM_MATCH_POINTS);
    nh.param<double>("mapping/nearest_max_range", map_manager_config_->nearest_max_range, 5.0);
    nh.param<double>("mapping/plane_fit_threshold", map_manager_config_->plane_fit_threshold, 0.1);
    nh.param<double>("mapping/point_map_downsample_size", map_manager_config_->point_map_downsample_size,
                     filter_size_surf_min_);
    nh.param<double>("mapping/ikd_delete_param", map_manager_config_->ikd_delete_param, 0.5);
    nh.param<double>("mapping/ikd_balance_param", map_manager_config_->ikd_balance_param, 0.7);
    nh.param<double>("mapping/ikd_box_length", map_manager_config_->ikd_box_length, 0.2);
    nh.param<double>("mapping/ivox_resolution", map_manager_config_->ivox_resolution, 0.2);
    nh.param<int>("mapping/ivox_nearby_type", map_manager_config_->ivox_nearby_type, 18);
    int ivox_capacity = static_cast<int>(map_manager_config_->ivox_capacity);
    nh.param<int>("mapping/ivox_capacity", ivox_capacity, 1000000);
    map_manager_config_->ivox_capacity = static_cast<std::size_t>(std::max(ivox_capacity, 1));
    nh.param<bool>("mapping/c3p_enable_voxel_merging", map_manager_config_->c3p_enable_voxel_merging, false);
    nh.param<double>("mapping/c3p_merge_theta_thresh", map_manager_config_->c3p_merge_theta_thresh, 0.05);
    nh.param<double>("mapping/c3p_merge_dist_thresh", map_manager_config_->c3p_merge_dist_thresh, 0.05);
    nh.param<double>("mapping/c3p_merge_cov_min_eigen_val_thresh",
                     map_manager_config_->c3p_merge_cov_min_eigen_val_thresh, 0.002);
    nh.param<double>("mapping/c3p_merge_x_coord_diff_thresh", map_manager_config_->c3p_merge_x_coord_diff_thresh, 5.0);
    nh.param<double>("mapping/c3p_merge_y_coord_diff_thresh", map_manager_config_->c3p_merge_y_coord_diff_thresh, 5.0);

    nh.param<double>("noise_model/ranging_cov", ranging_cov_, 0.02);
    nh.param<double>("noise_model/angle_cov", angle_cov_, 0.05);
    nh.param<double>("noise_model/gyr_cov", gyr_cov_, 0.1);
    nh.param<double>("noise_model/acc_cov", acc_cov_, 0.1);
    nh.param<double>("noise_model/b_gyr_cov", b_gyr_cov_, 0.0001);
    nh.param<double>("noise_model/b_acc_cov", b_acc_cov_, 0.0001);
    nh.param<bool>("publish/pub_voxel_map", publish_voxel_map_, false);
    nh.param<int>("publish/publish_max_voxel_layer", publish_max_voxel_layer_, 0);
    nh.param<bool>("publish/map_en", map_publish_en_, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en_, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval_, -1);

    nh.param<double>("preprocess/blind", preprocess_->blind, 0.01);
    nh.param<double>("preprocess/maximum_range", preprocess_->maximum_range, 70.0);
    nh.param<double>("preprocess/vertical_angle_min", preprocess_->vertical_angle_min, -45.0);
    nh.param<double>("preprocess/vertical_angle_max", preprocess_->vertical_angle_max, 45.0);
    nh.param<double>("preprocess/horizontal_angle_min", preprocess_->horizontal_angle_min, -60.0);
    nh.param<double>("preprocess/horizontal_angle_max", preprocess_->horizontal_angle_max, 60.0);
    nh.param<int>("preprocess/lidar_type", preprocess_->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", preprocess_->N_SCANS, 16);
    nh.param<int>("preprocess/scan_rate", preprocess_->SCAN_RATE, 10);
    nh.param<int>("preprocess/point_filter_num", preprocess_->point_filter_num, 1);
    nh.param<bool>("preprocess/feature_extract_enable", preprocess_->feature_enabled, false);

    layer_size_.clear();
    layer_size_.reserve(layer_point_size_.size());
    for (const double point_size : layer_point_size_) {
        layer_size_.push_back(static_cast<int>(point_size));
    }
    map_manager_config_->voxel_size = voxel_size_;
    map_manager_config_->max_layer = max_layer_;
    map_manager_config_->layer_point_size = layer_size_;
    map_manager_config_->max_points_size = max_points_size_;
    map_manager_config_->max_cov_points_size = max_cov_points_size_;
    map_manager_config_->plane_threshold = plannar_threshold_;
    map_manager_config_->sigma_num = sigma_num_;
    return true;
}

/** @brief Bridges IKFoM's raw callback to the active Mapping instance. */
void Mapping::ObservationModelTrampoline(state_ikfom& state, esekfom::dyn_share_datastruct<double>& ekfom_data) {
    if (active_instance_ != nullptr) {
        active_instance_->ObservationModel(state, ekfom_data);
    } else {
        ekfom_data.valid = false;
    }
}

// Backend-specific observation model helpers.
/** @brief Orders VoxelMap points by propagated covariance magnitude. */
const bool var_contrast(voxel_map_ns::pointWithCov& x, voxel_map_ns::pointWithCov& y) {
    return (x.cov_world.diagonal().norm() < y.cov_world.diagonal().norm());
};
/** @brief Orders VoxelMapPlus points by propagated covariance magnitude. */
const bool var_contrast_plus(voxel_map_plus_ns::pointWithCov& x, voxel_map_plus_ns::pointWithCov& y) {
    return (x.cov_world.diagonal().norm() < y.cov_world.diagonal().norm());
};
/** @brief Transforms a LiDAR-frame cloud into the world frame. */
void Mapping::transformLidar2World(const state_ikfom& state_point_, const PointCloudXYZI::Ptr& input_cloud,
                                   PointCloudXYZI::Ptr& trans_cloud) const {
    trans_cloud->clear();
    trans_cloud->reserve(input_cloud->size());
    for (std::size_t i = 0; i < input_cloud->size(); ++i) {
        pcl::PointXYZINormal p_c = input_cloud->points[i];
        Eigen::Vector3d p_lidar(p_c.x, p_c.y, p_c.z);
        Eigen::Vector3d p_world =
            state_point_.rot * (state_point_.offset_R_L_I * p_lidar + state_point_.offset_T_L_I) + state_point_.pos;

        PointType pi;
        pi.x = p_world(0);
        pi.y = p_world(1);
        pi.z = p_world(2);
        pi.intensity = p_c.intensity;
        trans_cloud->points.push_back(pi);
    }
}

/** @brief Propagates LiDAR point covariance through extrinsics and pose. */
M3D Mapping::transformLidarCovToWorld(Eigen::Vector3d& p_lidar, const esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_,
                                      const Eigen::Matrix3d& COV_lidar) const {
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(p_lidar);
    auto state = kf_.get_x();

    // lidar到body的方差传播
    // 注意外参的var是先rot 后pos
    M3D il_rot_var = kf_.get_P().block<3, 3>(6, 6);
    M3D il_t_var = kf_.get_P().block<3, 3>(9, 9);

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
    M3D t_var = kf_.get_P().block<3, 3>(0, 0);

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
std::vector<pv_lio_plus::MapPoint, Eigen::aligned_allocator<pv_lio_plus::MapPoint>>
Mapping::make_manager_points(const PointCloudXYZI::Ptr& points_lidar, const PointCloudXYZI::Ptr& points_world,
                             const std::vector<M3D>& cov_lidar) const {
    const std::size_t point_count = std::min(points_lidar->size(), points_world->size());
    pv_lio_plus::MapPointList result;
    result.reserve(point_count);
    for (std::size_t i = 0; i < point_count; ++i) {
        pv_lio_plus::MapPoint point;
        point.point_lidar << points_lidar->points[i].x, points_lidar->points[i].y, points_lidar->points[i].z;
        point.point_world << points_world->points[i].x, points_world->points[i].y, points_world->points[i].z;
        if (i < cov_lidar.size()) {
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
M3D Mapping::calc_lidar_cov_for_backend(const V3D& point) const {
    V3D safe_point = point;
    if (safe_point.z() == 0.0) {
        safe_point.z() = 0.001;
    }
    if (map_manager_config_->type == pv_lio_plus::MapType::VoxelMapPlus) {
        return voxel_map_plus_ns::calcLidarCov(safe_point, ranging_cov_, angle_cov_);
    }
    return voxel_map_ns::calcLidarCov(safe_point, ranging_cov_, angle_cov_);
}

/* -------------------------------------------------------------------------- */
/*                                   model                                    */
/* -------------------------------------------------------------------------- */

/** @brief Builds the native VoxelMap residual model. */
void Mapping::ObservationModelVoxelMap(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
    feats_with_correspondence_->clear();
    total_residual_ = 0.0;

    //* 1.用最新的位姿将点云转换到world地图系
    PointCloudXYZI::Ptr feats_world(new PointCloudXYZI);
    transformLidar2World(s, feats_undistort_down_, feats_world);

    //* 2.body系转world系
    std::vector<voxel_map_ns::pointWithCov> pv_list(feats_undistort_down_->size());
    for (std::size_t i = 0; i < feats_undistort_down_->size(); ++i) {
        // note: 每次迭代时，body系下的点云 pos 和 cov 不变
        voxel_map_ns::pointWithCov& pv = pv_list[i];
        pv.point_lidar << feats_undistort_down_->points[i].x, feats_undistort_down_->points[i].y,
            feats_undistort_down_->points[i].z;
        pv.point_world << feats_world->points[i].x, feats_world->points[i].y, feats_world->points[i].z;
        pv.cov_lidar = var_down_lidar_[i];
        pv.cov_world = transformLidarCovToWorld(pv.point_lidar, kf_, pv.cov_lidar);
        // pv_list[i] = pv;
    }

    ros::WallTime t0 = ros::WallTime::now();
    //* 3.查找最近点，计算残差
    std::vector<voxel_map_ns::ptpl> ptpl_list;
    std::vector<V3D> non_match_list;
    map_manager_->search(pv_list, ptpl_list, non_match_list);
    search_time_ += (ros::WallTime::now() - t0).toSec() * 1000;

    //* 4.计算用于构建法方程的各矩阵向量，Computation of Measuremnt Jacobian matrix H and measurents vector
    // TODO 为什么不加上状态量对状态量误差的导数？？？？像quaternion那本书？
    effective_feature_num_ = ptpl_list.size();
    if (effective_feature_num_ < 1) {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }
    ekfom_data.h_x = MatrixXd::Zero(effective_feature_num_,
                                    12); //< 观测值对状态量的导数，点面距离只和位姿、外参有关，对其他状态量的导数都是0
    ekfom_data.h.resize(effective_feature_num_);
    ekfom_data.R.resize(effective_feature_num_, 1); // note: 把R作为向量 用的时候转换成diag

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < effective_feature_num_; i++) {
        voxel_map_ns::ptpl& ptpl_i = ptpl_list[i]; // note: modified

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
        if (extrinsic_est_en_) {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec.x(), norm_vec.y(), norm_vec.z(), VEC_FROM_ARRAY(A),
                VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        } else {
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
        double sigma_l = J_nq * ptpl_i.plane_cov * J_nq.transpose();

        // HACK 1. 因为是标量 所以求逆直接用1除
        // HACK 2. 不同分量的方差用加法来合成 因为公式(12)中的Sigma是对角阵，逐元素运算之后就是对角线上的项目相加
        // M3D cov         = s.rot * s.offset_R_L_I * ptpl_i.cov_lidar * s.offset_R_L_I.conjugate() * s.rot.conjugate();
        M3D cov = ptpl_i.cov_world;
        ekfom_data.R(i) = 1.0 / (sigma_l + norm_vec.transpose() * cov * norm_vec);
    }

    res_mean_last_ = total_residual_ / effective_feature_num_; //? 未使用，total_residual_ 未计算
}

/** @brief Builds the native VoxelMapPlus residual model. */
void Mapping::ObservationModelVoxelMapPlus(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
    total_residual_ = 0.0;

    //* 1.用最新的位姿将点云转换到world地图系
    PointCloudXYZI::Ptr feats_world(new PointCloudXYZI);
    transformLidar2World(s, feats_undistort_down_, feats_world);

    //* 2.body系转world系
    std::vector<voxel_map_plus_ns::pointWithCov> pv_list(feats_undistort_down_->size());
    for (std::size_t i = 0; i < feats_undistort_down_->size(); ++i) {
        // note: 每次迭代时，body系下的点云 pos 和 cov 不变
        voxel_map_plus_ns::pointWithCov& pv = pv_list[i];
        pv.point_lidar << feats_undistort_down_->points[i].x, feats_undistort_down_->points[i].y,
            feats_undistort_down_->points[i].z;
        pv.point_world << feats_world->points[i].x, feats_world->points[i].y, feats_world->points[i].z;
        Eigen::Matrix3d& cov_lidar = var_down_lidar_[i];
        pv.cov_world = transformLidarCovToWorld(pv.point_lidar, kf_, cov_lidar);
    }

    ros::WallTime t0 = ros::WallTime::now();
    //* 3.查找最近点，计算残差
    std::vector<voxel_map_plus_ns::ptpl> ptpl_list;
    std::vector<V3D> non_match_list;
    map_manager_->search(pv_list, ptpl_list, non_match_list);
    search_time_ += (ros::WallTime::now() - t0).toSec() * 1000;

    //* 4.计算用于构建法方程的各矩阵向量，Computation of Measuremnt Jacobian matrix H and measurents vector
    // TODO 为什么不加上状态量对状态量误差的导数？？？？像quaternion那本书？
    effective_feature_num_ = ptpl_list.size();
    if (effective_feature_num_ < 1) {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points!");
        return;
    }
    ekfom_data.h_x = MatrixXd::Zero(effective_feature_num_,
                                    12); //< 观测值对状态量的导数，点面距离只和位姿、外参有关，对其他状态量的导数都是0
    ekfom_data.h.resize(effective_feature_num_);
    ekfom_data.R.resize(effective_feature_num_, 1); // note: 把R作为向量 用的时候转换成diag

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < effective_feature_num_; i++) {
        voxel_map_plus_ns::ptpl& ptpl_i = ptpl_list[i]; // note: modified

        //* 点坐标及其反对称矩阵、法向量
        V3D point_this_be(ptpl_i.point);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        V3D norm_vec(ptpl_i.omega / ptpl_i.omega_norm);

        //! 计算 Measuremnt Jacobian matrix H，即 h_x
        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D A(point_crossmat * s.rot.conjugate() * norm_vec); // 与 voxelmap_plus 一致
        {
            // 不估计外参，不需要求后六位的导数
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec.x(), norm_vec.y(), norm_vec.z(), VEC_FROM_ARRAY(A), 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0;
        }

        //! 计算 estimate measurement，其几何意义是 distance to the closest surface/corner
        //! NOTE 赋值给 ekfom_data.h(i) 前需先强制转换为 float，否则误差会随着每次转角逐渐累积，导致最终漂移
        ekfom_data.h(i) = -float(ptpl_i.dist);

        Eigen::Matrix<double, 1, 3> J_abd;
        Eigen::Matrix<double, 1, 3> J_pw;
        V3D pw = ptpl_i.point_world;
        double tmp = ptpl_i.dist / (ptpl_i.omega_norm * ptpl_i.omega_norm);
        if (ptpl_list[i].main_direction == 0) { // Plane equation: ax+by+z+d = 0
            J_abd << pw(0) - ptpl_i.omega[0] * tmp, pw(1) - ptpl_i.omega[1] * tmp, 1;
        } else if (ptpl_list[i].main_direction == 1) { // Plane equation: ax+y+bz+d = 0
            J_abd << pw(0) - ptpl_i.omega[0] * tmp, pw(2) - ptpl_i.omega[2] * tmp, 1;
        } else { // Plane equation: x+ay+bz+d = 0
            J_abd << pw(1) - ptpl_i.omega[1] * tmp, pw(2) - ptpl_i.omega[2] * tmp, 1;
        }
        J_abd /= ptpl_i.omega_norm;
        double sigma_l = J_abd * ptpl_i.plane_cov * J_abd.transpose();
        J_pw = ptpl_i.omega.transpose() / ptpl_i.omega_norm;

        // note: 原始 voxelmap++ 中，ptpl_i.point_cov 处使用 lidar 系的 cov，这是导致算法不稳定的一个原因
        ekfom_data.R(i) = 1.0 / (sigma_l + J_pw * ptpl_i.point_cov * J_pw.transpose());
    }

    res_mean_last_ = total_residual_ / effective_feature_num_; //? 未使用，total_residual_ 未计算
}

/**
 * @brief Builds EKF residuals from ikd-tree or iVox matches.
 * @param s Current iterated filter state.
 * @param ekfom_data EKF measurement Jacobian, residual, and weight outputs.
 */
void Mapping::ObservationModelPointBackend(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
    total_residual_ = 0.0;

    PointCloudXYZI::Ptr feats_world(new PointCloudXYZI);
    transformLidar2World(s, feats_undistort_down_, feats_world);
    pv_lio_plus::MapPointList map_points = make_manager_points(feats_undistort_down_, feats_world, var_down_lidar_);

    ros::WallTime t0 = ros::WallTime::now();
    pv_lio_plus::PlaneMatchList matches;
    std::vector<V3D> non_match_list;
    map_manager_->search(map_points, matches, non_match_list);
    search_time_ += (ros::WallTime::now() - t0).toSec() * 1000;

    effective_feature_num_ = static_cast<int>(matches.size());
    if (effective_feature_num_ < 1) {
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
    for (int i = 0; i < effective_feature_num_; ++i) {
        const pv_lio_plus::PlaneMatch& match = matches[static_cast<std::size_t>(i)];
        const V3D point_this_be = match.point;
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);

        const V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        V3D norm_vec = match.normal;
        const double norm = norm_vec.norm();
        if (norm > std::numeric_limits<double>::epsilon()) {
            norm_vec /= norm;
        }

        const V3D C = s.rot.conjugate() * norm_vec;
        const V3D A = point_crossmat * C;
        if (extrinsic_est_en_) {
            const V3D B = point_be_crossmat * s.offset_R_L_I.conjugate() * C;
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec.x(), norm_vec.y(), norm_vec.z(), VEC_FROM_ARRAY(A),
                VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        } else {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec.x(), norm_vec.y(), norm_vec.z(), VEC_FROM_ARRAY(A), 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0;
        }

        ekfom_data.h(i) = -match.distance;

        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq.block<1, 3>(0, 0) = match.point_world - match.center;
        J_nq.block<1, 3>(0, 3) = -match.normal;
        const double sigma_l = (J_nq * match.plane_cov * J_nq.transpose())(0, 0);
        const double sigma_point = (norm_vec.transpose() * match.point_cov * norm_vec)(0, 0);
        double total_sigma = sigma_l + sigma_point;
        if (!std::isfinite(total_sigma) || total_sigma <= 1e-9) {
            total_sigma = 1e-9;
        }
        ekfom_data.R(i) = 1.0 / total_sigma;
        total_residual_ += std::abs(match.distance);
    }

    res_mean_last_ = total_residual_ / effective_feature_num_;
}

/**
 * @brief Builds EKF residuals from native C3P-VoxelMap matches.
 * @param s Current iterated filter state.
 * @param ekfom_data EKF measurement Jacobian, residual, and weight outputs.
 */
void Mapping::ObservationModelC3P(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
    total_residual_ = 0.0;

    PointCloudXYZI::Ptr feats_world(new PointCloudXYZI);
    transformLidar2World(s, feats_undistort_down_, feats_world);

    const std::size_t point_count = std::min(feats_undistort_down_->size(), feats_world->size());
    pv_lio_plus::MapPointList map_points;
    map_points.reserve(point_count);
    for (std::size_t i = 0; i < point_count; ++i) {
        pv_lio_plus::MapPoint point;
        point.point_lidar << feats_undistort_down_->points[i].x, feats_undistort_down_->points[i].y,
            feats_undistort_down_->points[i].z;
        point.point_world << feats_world->points[i].x, feats_world->points[i].y, feats_world->points[i].z;
        if (i < var_down_lidar_.size()) {
            point.cov_lidar = var_down_lidar_[i];
            point.cov_world = transformLidarCovToWorld(point.point_lidar, kf_, point.cov_lidar);
        }
        map_points.emplace_back(point);
    }

    ros::WallTime t0 = ros::WallTime::now();
    pv_lio_plus::PlaneMatchList matches;
    std::vector<V3D> non_match_list;
    map_manager_->search(map_points, matches, non_match_list);
    search_time_ += (ros::WallTime::now() - t0).toSec() * 1000;

    effective_feature_num_ = static_cast<int>(matches.size());
    if (effective_feature_num_ < 1) {
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
    for (int i = 0; i < effective_feature_num_; ++i) {
        const pv_lio_plus::PlaneMatch& match = matches[static_cast<std::size_t>(i)];
        const V3D point_this_be = match.point;
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);

        const V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        const V3D& norm_vec = match.normal;
        const V3D C = s.rot.conjugate() * norm_vec;
        const V3D A = point_crossmat * C;
        if (extrinsic_est_en_) {
            const V3D B = point_be_crossmat * s.offset_R_L_I.conjugate() * C;
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec.x(), norm_vec.y(), norm_vec.z(), VEC_FROM_ARRAY(A),
                VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        } else {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec.x(), norm_vec.y(), norm_vec.z(), VEC_FROM_ARRAY(A), 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0;
        }

        const double signed_point_plane_residual = norm_vec.dot(match.point_world) + match.d;
        ekfom_data.h(i) = -signed_point_plane_residual;

        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq.block<1, 3>(0, 0) = match.point_world - match.center;
        J_nq.block<1, 3>(0, 3) = -norm_vec;
        const double sigma_plane = (J_nq * match.plane_cov * J_nq.transpose())(0, 0);
        const double sigma_point = (norm_vec.transpose() * match.point_cov * norm_vec)(0, 0);
        ekfom_data.R(i) = 1.0 / (sigma_plane + sigma_point);
        total_residual_ += std::abs(signed_point_plane_residual);
    }

    res_mean_last_ = total_residual_ / effective_feature_num_;
}

/**
 * @brief Dispatches the EKF observation model to the selected map backend.
 * @param s Current iterated filter state.
 * @param ekfom_data EKF measurement Jacobian, residual, and weight outputs.
 */
void Mapping::ObservationModel(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
    if (map_manager_config_->type == pv_lio_plus::MapType::VoxelMap) {
        ObservationModelVoxelMap(s, ekfom_data);
    } else if (map_manager_config_->type == pv_lio_plus::MapType::VoxelMapPlus) {
        ObservationModelVoxelMapPlus(s, ekfom_data);
    } else if (map_manager_config_->type == pv_lio_plus::MapType::C3PVoxelMap) {
        ObservationModelC3P(s, ekfom_data);
    } else {
        ObservationModelPointBackend(s, ekfom_data);
    }
}

/** @brief Builds the initial local map after EKF initialization. */
void Mapping::InitializeMap() {
    ros::WallTime t1 = ros::WallTime::now();

    // Transform the first scan before handing it to the selected backend.
    PointCloudXYZI::Ptr feats_world(new PointCloudXYZI);
    transformLidar2World(state_point_, feats_undistort_, feats_world);
    if (map_manager_config_->type == pv_lio_plus::MapType::VoxelMapPlus) {
        std::vector<voxel_map_plus_ns::pointWithCov> pv_list(feats_undistort_->size());
        for (std::size_t i = 0; i < feats_world->size(); ++i) {
            voxel_map_plus_ns::pointWithCov& pv = pv_list[i];
            pv.point_lidar << feats_undistort_->points[i].x, feats_undistort_->points[i].y,
                feats_undistort_->points[i].z;
            pv.point_world << feats_world->points[i].x, feats_world->points[i].y, feats_world->points[i].z;

            pv.cov_lidar = voxel_map_plus_ns::calcLidarCov(pv.point_lidar, ranging_cov_, angle_cov_);
            pv.cov_world = transformLidarCovToWorld(pv.point_lidar, kf_, pv.cov_lidar);
        }
        map_manager_->initialize(pv_list);
    } else if (map_manager_config_->type == pv_lio_plus::MapType::VoxelMap) {
        std::vector<voxel_map_ns::pointWithCov> pv_list(feats_undistort_->size());
        for (std::size_t i = 0; i < feats_world->size(); ++i) {
            voxel_map_ns::pointWithCov& pv = pv_list[i];
            pv.point_lidar << feats_undistort_->points[i].x, feats_undistort_->points[i].y,
                feats_undistort_->points[i].z;
            pv.point_world << feats_world->points[i].x, feats_world->points[i].y, feats_world->points[i].z;

            // if z=0, error will occur in calcBodyCov. To be solved
            if (pv.point_lidar[2] == 0) {
                pv.point_lidar[2] = 0.001;
            }
            pv.cov_lidar = voxel_map_ns::calcLidarCov(pv.point_lidar, ranging_cov_, angle_cov_);
            pv.cov_world = transformLidarCovToWorld(pv.point_lidar, kf_, pv.cov_lidar);
        }
        map_manager_->initialize(pv_list);
    } else {
        std::vector<M3D> cov_lidar;
        cov_lidar.reserve(feats_undistort_->size());
        for (const auto& point : feats_undistort_->points) {
            cov_lidar.emplace_back(calc_lidar_cov_for_backend(V3D(point.x, point.y, point.z)));
        }
        map_manager_->initialize(make_manager_points(feats_undistort_, feats_world, cov_lidar));
    }

    double map_build_time = (ros::WallTime::now() - t1).toSec();
    std::cout << std::fixed << "[Init Map] Build " << pv_lio_plus::MapTypeName(map_manager_config_->type) << ": "
              << map_build_time * 1000 << " ms\n";

    map_initialized_ = true;
}

/** @brief Inserts the current scan into the selected local-map backend. */
void Mapping::UpdateMap() {
    // Transform the filtered scan once; all backends consume world-frame points.
    PointCloudXYZI::Ptr feats_world(new PointCloudXYZI);
    transformLidar2World(state_point_, feats_undistort_down_, feats_world);

    // Propagate covariance and dispatch the backend-specific point container.
    if (map_manager_config_->type == pv_lio_plus::MapType::VoxelMapPlus) {
        std::vector<voxel_map_plus_ns::pointWithCov> pv_list(feats_undistort_down_->size());
        for (std::size_t i = 0; i < feats_undistort_down_->size(); ++i) {
            voxel_map_plus_ns::pointWithCov& pv = pv_list[i];
            pv.point_lidar << feats_undistort_down_->points[i].x, feats_undistort_down_->points[i].y,
                feats_undistort_down_->points[i].z;
            pv.point_world << feats_world->points[i].x, feats_world->points[i].y, feats_world->points[i].z;
            pv.cov_lidar = var_down_lidar_[i];
            pv.cov_world = transformLidarCovToWorld(pv.point_lidar, kf_, pv.cov_lidar);
        }
        std::sort(pv_list.begin(), pv_list.end(), var_contrast_plus);
        map_manager_->update(pv_list, static_cast<std::uint32_t>(scan_total_count_));
    } else if (map_manager_config_->type == pv_lio_plus::MapType::VoxelMap) {
        std::vector<voxel_map_ns::pointWithCov> pv_list(feats_undistort_down_->size());
        for (std::size_t i = 0; i < feats_undistort_down_->size(); ++i) {
            voxel_map_ns::pointWithCov& pv = pv_list[i];
            pv.point_lidar << feats_undistort_down_->points[i].x, feats_undistort_down_->points[i].y,
                feats_undistort_down_->points[i].z;
            pv.point_world << feats_world->points[i].x, feats_world->points[i].y, feats_world->points[i].z;
            pv.cov_lidar = var_down_lidar_[i];
            pv.cov_world = transformLidarCovToWorld(pv.point_lidar, kf_, pv.cov_lidar);
        }
        std::sort(pv_list.begin(), pv_list.end(), var_contrast);
        map_manager_->update(pv_list, static_cast<std::uint32_t>(scan_total_count_));
    } else {
        MapPointList map_points = make_manager_points(feats_undistort_down_, feats_world, var_down_lidar_);
        std::sort(map_points.begin(), map_points.end(),
                  [](const pv_lio_plus::MapPoint& lhs, const pv_lio_plus::MapPoint& rhs) {
                      return lhs.cov_world.diagonal().norm() < rhs.cov_world.diagonal().norm();
                  });
        map_manager_->update(map_points, static_cast<std::uint32_t>(scan_total_count_));
    }
    if (map_manager_config_->local_window_enabled) {
        map_manager_->move_window(state_point_.pos, V3D(det_range_, det_range_, det_range_));
    }
}

/** @brief Runs the configured ROS mapping loop and persists its outputs. */
int Mapping::Run() {
    ros::NodeHandle nh;

    // 1. Load ROS parameters into the Mapping-owned state.
    if (!load_parameters(nh)) {
        return 1;
    }

    // 2. Apply backend configuration and initialize the estimator.
    map_manager_->configure(*map_manager_config_);
    if (!map_manager_->supports_selected_backend()) {
        ROS_FATAL("Selected map backend is not supported: %s", pv_lio_plus::MapTypeName(map_manager_config_->type));
        return 1;
    }

    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = "camera_init";

    down_size_filter_surf_.setLeafSize(filter_size_surf_min_, filter_size_surf_min_, filter_size_surf_min_);

    lidar_t_wrt_imu_ << VEC_FROM_ARRAY(extrin_t_);
    lidar_r_wrt_imu_ << MAT_FROM_ARRAY(extrin_r_);
    imu_process_->set_extrinsic(lidar_t_wrt_imu_, lidar_r_wrt_imu_);
    imu_process_->set_gyr_cov(V3D(gyr_cov_, gyr_cov_, gyr_cov_));
    imu_process_->set_acc_cov(V3D(acc_cov_, acc_cov_, acc_cov_));
    imu_process_->set_gyr_bias_cov(V3D(b_gyr_cov_, b_gyr_cov_, b_gyr_cov_));
    imu_process_->set_acc_bias_cov(V3D(b_acc_cov_, b_acc_cov_, b_acc_cov_));

    double epsi[23] = {0.001};
    std::fill(epsi, epsi + 23, 0.001);
    kf_.init_dyn_share(get_f, df_dx, df_dw, ObservationModelTrampoline, num_max_iterations_, epsi);

    // 3. Register sensor subscribers and output publishers.
    ros::Subscriber sub_pcl;
    if (preprocess_->lidar_type == AVIA) {
        sub_pcl = nh.subscribe<livox_ros_driver::CustomMsg>(
            lidar_topic_, 200000, [this](const livox_ros_driver::CustomMsg::ConstPtr& msg) {
                livox_pcl_cbk(msg, time_sync_en_, buffer_mutex_, buffer_condition_, lidar_buffer_, time_buffer_,
                              imu_buffer_, last_timestamp_lidar_, last_timestamp_imu_, timediff_set_,
                              timediff_lidar_wrt_imu_, scan_count_, *preprocess_);
            });
    } else {
        sub_pcl = nh.subscribe<sensor_msgs::PointCloud2>(
            lidar_topic_, 200000, [this](const sensor_msgs::PointCloud2::ConstPtr& msg) {
                standard_pcl_cbk(msg, lidar_time_offset_, buffer_mutex_, buffer_condition_, lidar_buffer_, time_buffer_,
                                 last_timestamp_lidar_, scan_count_, *preprocess_);
            });
    }
    ros::Subscriber sub_imu =
        nh.subscribe<sensor_msgs::Imu>(imu_topic_, 200000, [this](const sensor_msgs::Imu::ConstPtr& msg) {
            imu_cbk(msg, time_sync_en_, timediff_lidar_wrt_imu_, buffer_mutex_, buffer_condition_, imu_buffer_,
                    last_timestamp_imu_);
        });
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudFull_lidar = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_lidar", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 100000);
    ros::Publisher voxel_map_pub = nh.advertise<visualization_msgs::MarkerArray>("/planes", 10000);

    std::size_t n_converged = 0;
    std::size_t n_not_converged = 0;

    // 4. Process synchronized LiDAR/IMU groups until ROS shuts down.
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    while (ros::ok()) {
        ros::spinOnce();

        ros::WallTime t0 = ros::WallTime::now();

        if (sync_packages(measures_, lidar_buffer_, time_buffer_, imu_buffer_, lidar_pushed_, lidar_end_time_,
                          last_timestamp_imu_, lidar_mean_scantime_, scan_num_)) {
            if (first_scan_) {
                first_lidar_time_ = measures_.lidar_beg_time;
                imu_process_->first_lidar_time = first_lidar_time_;
                first_scan_ = false;
                continue;
            }

            // Undistort the scan using the IMU propagation step.
            imu_process_->Process(measures_, kf_, feats_undistort_);
            state_point_ = kf_.get_x();

            if (feats_undistort_ == nullptr || feats_undistort_->empty()) {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            ekf_initialized_ = (measures_.lidar_beg_time - first_lidar_time_) < kInitTime ? false : true;

            if (ekf_initialized_ && !map_initialized_) {
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
            for (const auto& pt : feats_undistort_down_->points) {
                var_down_lidar_.push_back(calc_lidar_cov_for_backend(V3D(pt.x, pt.y, pt.z)));
            }

            if (feats_down_size_ < 5) {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            // Run the iterated EKF observation update.
            search_time_ = 0.;
            ros::WallTime t2 = ros::WallTime::now();
            bool bConverged = kf_.update_iterated_dyn_share_diagonal();
            eseikf_time_ = (ros::WallTime::now() - t2).toSec() * 1000;
            // Extract the corrected state and update the local map.
            state_point_ = kf_.get_x();
            geo_quat_.x = state_point_.rot.coeffs()[0];
            geo_quat_.y = state_point_.rot.coeffs()[1];
            geo_quat_.z = state_point_.rot.coeffs()[2];
            geo_quat_.w = state_point_.rot.coeffs()[3];

            ros::WallTime t3 = ros::WallTime::now();
            UpdateMap();
            update_time_ = (ros::WallTime::now() - t3).toSec() * 1000;

            // std::printf("BA: %.4f %.4f %.4f   \nBG: %.4f %.4f %.4f   \ng: %.4f %.4f %.4f\n",
            //             state_point_.ba.x(), state_point_.ba.y(), state_point_.ba.z(),
            //             state_point_.bg.x(), state_point_.bg.y(), state_point_.bg.z(),
            //             state_point_.grav.get_vect().x(), state_point_.grav.get_vect().y(),
            //             state_point_.grav.get_vect().z());
            avg_eseikf_time_ = avg_eseikf_time_ * (scan_total_count_ / double(scan_total_count_ + 1)) +
                               eseikf_time_ / (scan_total_count_ + 1);
            avg_search_time_ = avg_search_time_ * (scan_total_count_ / double(scan_total_count_ + 1)) +
                               search_time_ / (scan_total_count_ + 1);
            avg_update_time_ = avg_update_time_ * (scan_total_count_ / double(scan_total_count_ + 1)) +
                               update_time_ / (scan_total_count_ + 1);
            avg_total_time_ = avg_total_time_ * (scan_total_count_ / double(scan_total_count_ + 1)) +
                              total_time_ / (scan_total_count_ + 1);
            scan_total_count_++;

            // Publish the current state and optionally persist scan output.
            publish_odometry(pubOdomAftMapped, odom_after_mapped_, state_point_, kf_, *imu_process_, lidar_end_time_,
                             geo_quat_);
            if (path_pub_en_)
                publish_path(pubPath, path_, body_pose_, poses_, lidar_end_time_, state_point_, geo_quat_);
            else
                record_pose(poses_, lidar_end_time_, state_point_, geo_quat_);
            if (scan_pub_en_ || pcd_save_en_)
                publish_frame_world(pubLaserCloudFull, scan_pub_en_, scan_dense_pub_en_, pcd_save_en_, feats_undistort_,
                                    feats_undistort_down_, state_point_, lidar_end_time_, root_dir_, pcd_save_interval_,
                                    pcd_index_, scan_wait_num_, *pcl_wait_save_);
            if (scan_pub_en_ && scan_body_pub_en_)
                publish_frame_body(pubLaserCloudFull_body, scan_dense_pub_en_, feats_undistort_, feats_undistort_down_,
                                   state_point_, lidar_end_time_);
            if (scan_pub_en_ && scan_lidar_pub_en_)
                publish_frame_lidar(pubLaserCloudFull_lidar, scan_dense_pub_en_, feats_undistort_,
                                    feats_undistort_down_, lidar_end_time_);
            if (publish_voxel_map_)
                map_manager_->publish_planes(voxel_map_pub, publish_max_voxel_layer_);
            if (map_publish_en_)
                publish_map_snapshot(pubLaserCloudMap, *map_manager_->snapshot(), lidar_end_time_);

            total_time_ = (ros::WallTime::now() - t0).toSec() * 1000;

            // verbose
            std::printf("[%4.4f] Pos: %3.3f %3.3f %3.3f %u\n", lidar_end_time_ - first_lidar_time_, state_point_.pos(0),
                        state_point_.pos(1), state_point_.pos(2), bConverged);
            if (bConverged)
                ++n_converged;
            else
                ++n_not_converged;
            // printf("Time(ms): eseikf %3.3lf, search %3.3lf, update %3.3lf, total %3.3lf \n",
            //        eseikf_time_, search_time_, update_time_, total_time_);
        }

        rate.sleep();
    }

    // 5. Persist trajectory and point-cloud results after shutdown.
    save_results(n_converged, n_not_converged, poses_, root_dir_, map_manager_config_->type, pcd_save_en_,
                 *pcl_wait_save_);
    return 0;
}
/** @brief Initializes ROS and runs one node-owned Mapping instance. */
int RunMappingNode(int argc, char** argv) {
    ros::init(argc, argv, "pv_lio_plus_node");
    Mapping mapping;
    return mapping.Run();
}

} // namespace pv_lio_plus
