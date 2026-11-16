/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef PV_LIO_PLUS_MAP_CONFIG_H
#define PV_LIO_PLUS_MAP_CONFIG_H

/**
 * @file map_config.h
 * @brief 地图后端配置及公共参数校验。
 */

#include <cmath>
#include <cstddef>
#include <vector>

#include "map_manager/map_types.h"

namespace pv_lio_plus
{
/** @brief 检查点图搜索半径（米）及其平方是否为有限正数。 */
inline bool ValidNearestMaxRange(double radius)
{
    const double squaredRadius = radius * radius;
    return std::isfinite(radius) && radius > 0.0 && std::isfinite(squaredRadius) && squaredRadius > 0.0;
}

/** @brief 所有局部地图后端共用的配置。 */
struct MapManagerConfig
{
    MapType type = MapType::VoxelMap;  ///< 当前选择的地图后端。

    double voxel_size = 1.0;                ///< 体素地图的基础体素尺寸。
    int max_layer     = 2;                  ///< 搜索时允许的最大体素层级。
    std::vector<int> layer_point_size;      ///< 各体素层级的点数限制。
    int max_points_size            = 100;   ///< 单个体素保留的最大点数。
    int max_cov_points_size        = 100;   ///< 单个体素保留的最大协方差点数。
    double plane_threshold         = 0.01;  ///< 体素地图平面拟合阈值。
    double sigma_num               = 3.0;   ///< 概率体素匹配使用的 sigma 倍数。
    int plus_update_size_threshold = 5;     ///< VoxelMap++ 更新阈值。

    int nearest_point_count = NUM_MATCH_POINTS;  ///< 点图搜索请求的邻点数量。
    /// 搜索最大物理半径，单位为米；ikd-tree 包含等于边界的点，iVox 严格小于半径且受附近格域限制。
    double nearest_max_range         = 5.0;
    double plane_fit_threshold       = 0.1;  ///< 点图平面拟合残差阈值。
    double point_map_downsample_size = 0.5;  ///< 点图降采样叶尺寸。

    double ikd_delete_param  = 0.5;  ///< ikd-tree 删除参数。
    double ikd_balance_param = 0.7;  ///< ikd-tree 平衡参数。
    double ikd_box_length    = 0.2;  ///< ikd-tree 盒长参数。

    double ivox_resolution    = 0.2;      ///< iVox 体素分辨率。
    int ivox_nearby_type      = 18;       ///< iVox 邻域类型：0、6、18 或 26。
    std::size_t ivox_capacity = 1000000;  ///< iVox 最大网格数。
    bool ivox_optimized       = false;    ///< 是否启用保持语义的 iVox 存储/查询优化。

    double octvox_resolution    = 0.5;      ///< Super-LIO OctVox 网格分辨率。
    std::size_t octvox_capacity = 1000000;  ///< OctVox 原生 LRU 保留的最大网格数。

    bool c3p_enable_voxel_merging             = false;  ///< 是否启用 C3P-VoxelMap 平面合并。
    double c3p_merge_theta_thresh             = 0.05;   ///< C3P 平面合并角度阈值。
    double c3p_merge_dist_thresh              = 0.05;   ///< C3P 平面合并距离阈值。
    double c3p_merge_cov_min_eigen_val_thresh = 0.002;  ///< C3P 平面协方差最小特征值阈值。
    double c3p_merge_x_coord_diff_thresh      = 5.0;    ///< C3 平面沿 x 轴的最大合并距离。
    double c3p_merge_y_coord_diff_thresh      = 5.0;    ///< C3 平面沿 y 轴的最大合并距离。

    int r_max_root_points_size                = 1000;  ///< R-VoxelMap 根体素最大保留点数。
    int r_update_size_threshold               = 5;     ///< R-VoxelMap 更新点数阈值。
    double r_update_inlier_distance_threshold = 0.1;   ///< R-VoxelMap 更新内点距离阈值。
    std::vector<int> r_reinit_size_vec        = {100, 200, 300, 500, 700, 1000};  ///< R-VoxelMap 重建点数阈值。
    int r_lru_capacity                        = 1000000;  ///< R-VoxelMap 根体素 LRU 容量。
    int r_ransac_max_iter                     = 10;       ///< R-VoxelMap RANSAC 最大迭代次数。
    int r_ransac_sample_num                   = 4;        ///< R-VoxelMap RANSAC 每次采样点数。
    double r_ransac_inlier_distance_threshold = 0.1;      ///< R-VoxelMap RANSAC 内点距离阈值。
    double r_ransac_isplane_p_threshold       = 0.8;      ///< R-VoxelMap 平面判定概率阈值。
    int r_sample_seed                         = 42;       ///< R-VoxelMap 随机采样种子。
    int r_valid_check_max_layer               = 0;        ///< R-VoxelMap 有效性检查的最大层数。
    int r_valid_check_min_points_size         = 10;       ///< R-VoxelMap 有效性检查的最小点数。
    int r_valid_check_resolution              = 5;        ///< R-VoxelMap 有效性检查分辨率。

    double hvox_voxel_size          = 0.4;   ///< Surfel-LIO 原生 hVox 体素尺寸。
    int hvox_hierarchy_factor       = 3;     ///< hVox 层级倍率。
    double hvox_planarity_threshold = 0.01;  ///< hVox 平面性阈值。
    double hvox_max_distance        = 50.0;  ///< hVox 传感器最大距离。
    double hvox_box_multiplier      = 2.0;   ///< hVox 移动盒倍率。
    double hvox_lidar_noise_std     = 0.05;  ///< hVox LiDAR 噪声标准差。

    int i_octree_bucket_size         = 32;     ///< i-Octree 路由节点桶容量。
    double i_octree_min_extent       = 0.01;   ///< i-Octree 最小空间范围。
    bool i_octree_on_tree_downsample = false;  ///< 是否在树内执行降采样。

    bool local_window_enabled = false;  ///< 是否启用滑动局部窗口。
};

}  // namespace pv_lio_plus

#endif  // PV_LIO_PLUS_MAP_CONFIG_H
