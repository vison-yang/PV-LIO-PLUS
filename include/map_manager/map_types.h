/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef PV_LIO_PLUS_MAP_TYPES_H
#define PV_LIO_PLUS_MAP_TYPES_H

/**
 * @file map_types.h
 * @brief 地图管理器使用的公共类型。
 */

#include <optional>
#include <string>
#include <vector>

#include "common_lib.h"

namespace pv_lio_plus
{
/** @brief PV-LIO-PLUS 支持选择的局部地图后端。 */
enum class MapType
{
    /// PV-LIO 原生概率自适应体素地图。
    VoxelMap,
    /// 使用原生残差和更新逻辑的 VoxelMap++。
    VoxelMapPlus,
    /// FAST-LIO2 增量 ikd-tree 点地图。
    IKDTree,
    /// Faster-LIO iVox 点地图。
    IVox,
    /// C3P-VoxelMap 概率体素平面地图。
    C3PVoxelMap,
    /// Super-LIO LRU 八分体聚合点地图。
    OctVox,
    /// 使用递归内点/外点平面和原生 LRU 的 R-VoxelMap。
    RVoxelMap,
    /// Surfel-LIO 双层 hVox 地图及预计算 L1 surfel。
    HVox,
    /// 使用原生叶块和数据 AABB 搜索的增量八叉树。
    IOctree,
    /// iG-LIO 增量统计地图，使用公共五点点面观测。
    IGLIOPlane,
    /// 定向像素高度图及公共滤波高度观测。
    BIEVR
};

MapType ParseMapType(const std::string &name);

const char *MapTypeName(MapType type);

/** @brief LiDAR 系和世界系中的一个地图点。 */
struct MapPoint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    V3D point_lidar = V3D::Zero();  ///< 当前 LiDAR 系点坐标。
    V3D point_world = V3D::Zero();  ///< 世界系点坐标。
    M3D cov_lidar   = M3D::Zero();  ///< LiDAR 系点协方差。
    M3D cov_world   = M3D::Zero();  ///< 世界系点协方差。
};

/** @brief 使用 Eigen 对齐分配器的地图点序列。 */
using MapPointList = std::vector<MapPoint, Eigen::aligned_allocator<MapPoint>>;

/** @brief 提供 hVox 原生移动盒所需的世界系 IMU 位置。 */
struct MapUpdateContext
{
    std::optional<V3D> imu_position_world;  ///< 世界系 IMU 位置；缺省时不提供上下文位置。
    std::optional<M3D> rotImuLidar_;        ///< BIEVR 量程所需的当前 LiDAR 到 IMU 旋转。
    std::optional<V3D> transImuLidar_;      ///< BIEVR 量程所需的当前 LiDAR 到 IMU 平移，单位米。
};

/** @brief 后端返回的点面匹配及其观测所需数据。 */
struct PlaneMatch
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MapType map_type                      = MapType::VoxelMap;  ///< 产生该匹配的地图后端。
    V3D point                             = V3D::Zero();        ///< LiDAR 系查询点。
    V3D point_world                       = V3D::Zero();        ///< 世界系查询点。
    V3D normal                            = V3D::Zero();        ///< 匹配平面的单位法向。
    V3D center                            = V3D::Zero();        ///< 匹配平面的代表中心。
    V3D omega                             = V3D::Zero();        ///< VoxelMap++ 原生残差方向。
    M3D point_cov                         = M3D::Zero();        ///< 查询点的世界系协方差。
    M3D lidar_cov                         = M3D::Zero();        ///< LiDAR 系传感器协方差（R-VoxelMap）。
    Eigen::Matrix<double, 6, 6> plane_cov = Eigen::Matrix<double, 6, 6>::Zero();  ///< 后端提供的六维平面协方差。
    double d                              = 0.0;  ///< 平面方程偏移或后端对应量。
    double distance                       = 0.0;  ///< 带符号点面残差。
    double omega_norm                     = 0.0;  ///< omega 的模长。
    int layer                             = 0;    ///< 匹配所在的原生体素层。
    int main_direction                    = 0;    ///< VoxelMap++ 原生主方向。
};

/** @brief 使用 Eigen 对齐分配器的点面匹配序列。 */
using PlaneMatchList = std::vector<PlaneMatch, Eigen::aligned_allocator<PlaneMatch>>;

/** @brief 活动局部地图的世界系轴对齐边界。 */
struct MapWindow
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool initialized = false;        ///< 边界是否已初始化。
    V3D min_bound    = V3D::Zero();  ///< 窗口 x/y/z 最小坐标。
    V3D max_bound    = V3D::Zero();  ///< 窗口 x/y/z 最大坐标。
};

}  // namespace pv_lio_plus

#endif  // PV_LIO_PLUS_MAP_TYPES_H
