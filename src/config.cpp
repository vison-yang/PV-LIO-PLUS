/**
 * @file config.cpp
 * @brief ROS parameter loading for PV-LIO-PLUS.
 */

#include "config.hpp"

#include <algorithm>
#include <exception>

#include "map_manager/map_manager.h"

namespace pv_lio_plus
{
Config::Config() : map_manager_config_(new MapManagerConfig), preprocess_(new Preprocess)
{
#if defined(ROOT_DIR)
    root_dir_ = ROOT_DIR;
#endif
}

Config::~Config() = default;

bool Config::Load(ros::NodeHandle& nh)
{
    // ---------------------------- Common parameters --------------------------
    nh.param<double>("time_offset", lidar_time_offset_, 0.0);
    nh.param<bool>("publish/path_en", path_pub_en_, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en_, true);
    nh.param<bool>("publish/dense_publish_en", scan_dense_pub_en_, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en_, true);
    nh.param<bool>("publish/scan_lidarframe_pub_en", scan_lidar_pub_en_, true);
    nh.param<std::string>("common/lid_topic", lidar_topic_, "/livox/lidar");
    nh.param<std::string>("common/imu_topic", imu_topic_, "/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en_, false);

    // ----------------------------- Mapping parameters ------------------------
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
    try
    {
        map_manager_config_->type = ParseMapType(map_type_name);
    } catch (const std::exception& exception)
    {
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

    // ------------------------------ Noise and output --------------------------
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

    // ------------------------------ Preprocessing -----------------------------
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

    // Derived values are kept with the backend configuration.
    layer_size_.clear();
    layer_size_.reserve(layer_point_size_.size());
    for (const double point_size : layer_point_size_)
    {
        layer_size_.push_back(static_cast<int>(point_size));
    }
    map_manager_config_->voxel_size          = voxel_size_;
    map_manager_config_->max_layer           = max_layer_;
    map_manager_config_->layer_point_size    = layer_size_;
    map_manager_config_->max_points_size     = max_points_size_;
    map_manager_config_->max_cov_points_size = max_cov_points_size_;
    map_manager_config_->plane_threshold     = plannar_threshold_;
    map_manager_config_->sigma_num           = sigma_num_;
    return true;
}

MapManagerConfig& Config::map_manager_config()
{
    return *map_manager_config_;
}

const MapManagerConfig& Config::map_manager_config() const
{
    return *map_manager_config_;
}

Preprocess& Config::preprocess()
{
    return *preprocess_;
}

const Preprocess& Config::preprocess() const
{
    return *preprocess_;
}

}  // namespace pv_lio_plus
