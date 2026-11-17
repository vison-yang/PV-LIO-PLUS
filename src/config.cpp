/**
 * @file config.cpp
 * @brief ROS parameter loading for PV-LIO-PLUS.
 */

#include "config.hpp"

#include <algorithm>
#include <exception>

#include "map_manager/map_manager.h"
#include "observation/weight_config.h"

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
    if (map_manager_config_->type == MapType::OctVox && map_manager_config_->local_window_enabled)
    {
        ROS_FATAL("mapping/local_window_en is not supported by Super-LIO OctVox; disable it or select another map_type");
        return false;
    }
    if (map_manager_config_->type == MapType::RVoxelMap && map_manager_config_->local_window_enabled)
    {
        ROS_FATAL("mapping/local_window_en is not supported by R-VoxelMap; it uses native LRU eviction");
        return false;
    }
    if (map_manager_config_->type == MapType::HVox && map_manager_config_->local_window_enabled)
    {
        ROS_FATAL("mapping/local_window_en is not supported by hVox; it uses its native moving box");
        return false;
    }
    if (map_manager_config_->type == MapType::IGLIOPlane && map_manager_config_->local_window_enabled)
    {
        ROS_FATAL("mapping/local_window_en is not supported by ig_lio_plane; it uses native LRU eviction");
        return false;
    }
    nh.param<int>("mapping/nearest_point_count", map_manager_config_->nearest_point_count, NUM_MATCH_POINTS);
    nh.param<double>("mapping/nearest_max_range", map_manager_config_->nearest_max_range, 5.0);
    if ((map_manager_config_->type == MapType::IKDTree || map_manager_config_->type == MapType::IVox ||
         map_manager_config_->type == MapType::IOctree || map_manager_config_->type == MapType::IGLIOPlane) &&
        !ValidNearestMaxRange(map_manager_config_->nearest_max_range))
    {
        ROS_FATAL("mapping/nearest_max_range must be positive and finite in metres, with a finite nonzero square");
        return false;
    }
    nh.param<double>("mapping/plane_fit_threshold", map_manager_config_->plane_fit_threshold, 0.1);
    nh.param<double>("mapping/point_map_downsample_size", map_manager_config_->point_map_downsample_size,
                     filter_size_surf_min_);
    nh.param<double>("mapping/ikd_delete_param", map_manager_config_->ikd_delete_param, 0.5);
    nh.param<double>("mapping/ikd_balance_param", map_manager_config_->ikd_balance_param, 0.7);
    nh.param<double>("mapping/ikd_box_length", map_manager_config_->ikd_box_length, 0.2);
    nh.param<double>("mapping/ivox_resolution", map_manager_config_->ivox_resolution, 0.2);
    nh.param<int>("mapping/ivox_nearby_type", map_manager_config_->ivox_nearby_type, 18);
    nh.param<bool>("mapping/ivox_optimized", map_manager_config_->ivox_optimized, false);

    int ivox_capacity = static_cast<int>(map_manager_config_->ivox_capacity);
    nh.param<int>("mapping/ivox_capacity", ivox_capacity, 1000000);
    map_manager_config_->ivox_capacity = static_cast<std::size_t>(std::max(ivox_capacity, 1));

    nh.param<double>("mapping/octvox_resolution", map_manager_config_->octvox_resolution, 0.5);
    int octvox_capacity = static_cast<int>(map_manager_config_->octvox_capacity);
    nh.param<int>("mapping/octvox_capacity", octvox_capacity, 1000000);
    map_manager_config_->octvox_capacity = static_cast<std::size_t>(std::max(octvox_capacity, 1));

    nh.param<bool>("mapping/c3p_enable_voxel_merging", map_manager_config_->c3p_enable_voxel_merging, false);
    nh.param<double>("mapping/c3p_merge_theta_thresh", map_manager_config_->c3p_merge_theta_thresh, 0.05);
    nh.param<double>("mapping/c3p_merge_dist_thresh", map_manager_config_->c3p_merge_dist_thresh, 0.05);
    nh.param<double>("mapping/c3p_merge_cov_min_eigen_val_thresh",
                     map_manager_config_->c3p_merge_cov_min_eigen_val_thresh, 0.002);
    nh.param<double>("mapping/c3p_merge_x_coord_diff_thresh", map_manager_config_->c3p_merge_x_coord_diff_thresh, 5.0);
    nh.param<double>("mapping/c3p_merge_y_coord_diff_thresh", map_manager_config_->c3p_merge_y_coord_diff_thresh, 5.0);

    nh.param<int>("mapping/r_voxelmap_max_root_points_size", map_manager_config_->r_max_root_points_size, 1000);
    nh.param<int>("mapping/r_voxelmap_update_size_threshold", map_manager_config_->r_update_size_threshold, 5);
    nh.param<double>("mapping/r_voxelmap_update_inlier_distance_threshold",
                     map_manager_config_->r_update_inlier_distance_threshold, 0.1);
    nh.param<std::vector<int>>("mapping/r_voxelmap_reinit_size_vec", map_manager_config_->r_reinit_size_vec,
                               std::vector<int>{100, 200, 300, 500, 700, 1000});
    nh.param<int>("mapping/r_voxelmap_lru_capacity", map_manager_config_->r_lru_capacity, 1000000);
    nh.param<int>("mapping/r_voxelmap_ransac_max_iter", map_manager_config_->r_ransac_max_iter, 10);
    nh.param<int>("mapping/r_voxelmap_ransac_sample_num", map_manager_config_->r_ransac_sample_num, 4);
    nh.param<double>("mapping/r_voxelmap_ransac_inlier_distance_threshold",
                     map_manager_config_->r_ransac_inlier_distance_threshold, 0.1);
    nh.param<double>("mapping/r_voxelmap_ransac_isplane_p_threshold",
                     map_manager_config_->r_ransac_isplane_p_threshold, 0.8);
    nh.param<int>("mapping/r_voxelmap_sample_seed", map_manager_config_->r_sample_seed, 42);
    nh.param<int>("mapping/r_voxelmap_valid_check_max_layer", map_manager_config_->r_valid_check_max_layer, 0);
    nh.param<int>("mapping/r_voxelmap_valid_check_min_points_size",
                  map_manager_config_->r_valid_check_min_points_size, 10);
    nh.param<int>("mapping/r_voxelmap_valid_check_resolution", map_manager_config_->r_valid_check_resolution, 5);

    nh.param<double>("mapping/hvox_voxel_size", map_manager_config_->hvox_voxel_size, 0.4);
    nh.param<int>("mapping/hvox_hierarchy_factor", map_manager_config_->hvox_hierarchy_factor, 3);
    nh.param<double>("mapping/hvox_planarity_threshold", map_manager_config_->hvox_planarity_threshold, 0.01);
    nh.param<double>("mapping/hvox_max_distance", map_manager_config_->hvox_max_distance, 50.0);
    nh.param<double>("mapping/hvox_box_multiplier", map_manager_config_->hvox_box_multiplier, 2.0);
    nh.param<double>("mapping/hvox_lidar_noise_std", map_manager_config_->hvox_lidar_noise_std, 0.05);

    nh.param<int>("mapping/i_octree_bucket_size", map_manager_config_->i_octree_bucket_size, 32);
    nh.param<double>("mapping/i_octree_min_extent", map_manager_config_->i_octree_min_extent, 0.01);
    nh.param<bool>("mapping/i_octree_on_tree_downsample", map_manager_config_->i_octree_on_tree_downsample, false);

    // iG 地图保留原生统计和 LRU；该 selector 使用公共五点点面观测。
    nh.param<double>("mapping/ig_lio_resolution", map_manager_config_->igResolution_, 0.5);
    nh.param<int>("mapping/ig_lio_search_method", map_manager_config_->igSearchMethod_, 7);
    nh.param<int>("mapping/ig_lio_capacity", map_manager_config_->igCapacity_, 5000000);
    nh.param<int>("mapping/ig_lio_grid_max_points", map_manager_config_->igGridMaxPoints_, 20);

    // 高度图独立参数；实际量程在每帧按当前 LiDAR 到 IMU 外参计算。
    nh.param<double>("mapping/bievr_voxel_size", map_manager_config_->bievrVoxelSize_, 0.5);
    nh.param<double>("mapping/bievr_pixel_size", map_manager_config_->bievrPixelSize_, 0.05);
    nh.param<int>("mapping/bievr_capacity", map_manager_config_->bievrCapacity_, 1000000);
    nh.param<int>("mapping/bievr_max_image_pixels", map_manager_config_->bievrMaxImagePixels_, 1048576);
    nh.param<bool>("mapping/bievr_weighted", map_manager_config_->bievrWeighted_, true);
    nh.param<bool>("mapping/bievr_smooth", map_manager_config_->bievrSmooth_, true);
    nh.param<double>("mapping/bievr_normal_tolerance_deg", map_manager_config_->bievrNormalToleranceDeg_, 3.0);
    nh.param<double>("mapping/bievr_height_sigma_m", map_manager_config_->bievrHeightSigmaM_, 0.1);
    nh.param<double>("mapping/bievr_huber_delta_m", map_manager_config_->bievrHuberDeltaM_, 0.1);

    // 权重组件独立于地图；不支持的组合必须在初始化前拒绝，不能静默忽略。
    nh.param<std::string>("observation/weight_model", observationWeightModel_, "backend_default");
    nh.param<double>("observation/gm_scale_m", gmScaleM_, 0.1);
    try
    {
        observation::ValidateWeightConfig(observationWeightModel_, MapTypeName(map_manager_config_->type), gmScaleM_);
    } catch (const std::invalid_argument& error)
    {
        ROS_FATAL("Invalid observation configuration: %s", error.what());
        return false;
    }
    // 预留接口尚未实现，显式拒绝误配，避免启动后误以为 GenZ 已启用。
    std::string observationModel;
    nh.param<std::string>("observation/model", observationModel, "backend_default");
    if (observationModel != "backend_default")
    {
        ROS_FATAL("observation/model is not implemented: %s", observationModel.c_str());
        return false;
    }

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
    nh.param<bool>("logging/timing_en", timing_log_en_, false);
    nh.param<std::string>("logging/timing_path", timing_log_path_, std::string());
    // 实验可显式隔离输出，避免覆盖编译时目录中的已有地图和轨迹。
    nh.param<std::string>("output/root_dir", root_dir_, root_dir_);
    if (root_dir_.empty())
    {
        ROS_FATAL("output/root_dir must not be empty");
        return false;
    }
    if (root_dir_.back() != '/') root_dir_ += '/';

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
    nh.param<std::string>("preprocess/solid_timestamp_mode", preprocess_->solidTimestampMode_, "frame_absolute");
    if (preprocess_->lidar_type == SOLID &&
        (preprocess_->SCAN_RATE <= 0 || (preprocess_->solidTimestampMode_ != "frame_absolute" &&
                                       preprocess_->solidTimestampMode_ != "point_absolute")))
    {
        ROS_FATAL("SOLID requires a positive scan_rate and solid_timestamp_mode=frame_absolute/point_absolute");
        return false;
    }
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
