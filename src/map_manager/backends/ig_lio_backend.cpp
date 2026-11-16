/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/**
 * @file ig_lio_backend.cpp
 * @brief iG-LIO 原生格地图与共享五点观测之间的适配。
 */

#include "ig_lio_backend.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "../point_plane.h"

namespace pv_lio_plus::backend_ig_lio
{
namespace
{
/** @brief 将经过基本边界检查的公共配置转为原生配置。 */
ig_lio_native::Config MakeNativeConfig(const MapManagerConfig &config)
{
    if (config.igCapacity_ < 2 || config.igGridMaxPoints_ <= 0)
    {
        throw std::invalid_argument("iG-LIO capacity must be at least 2 and grid max points must be positive");
    }

    ig_lio_native::Config nativeConfig;
    nativeConfig.resolution_    = config.igResolution_;
    nativeConfig.searchMethod_  = config.igSearchMethod_;
    nativeConfig.capacity_      = static_cast<std::size_t>(config.igCapacity_);
    nativeConfig.gridMaxPoints_ = static_cast<std::size_t>(config.igGridMaxPoints_);
    return nativeConfig;
}

/** @brief 保持输入帧顺序提取世界系 XYZ。 */
std::vector<Eigen::Vector3d> ToWorldPoints(const MapPointList &points)
{
    std::vector<Eigen::Vector3d> worldPoints;
    worldPoints.reserve(points.size());
    for (const auto &point : points) worldPoints.push_back(point.point_world);
    return worldPoints;
}
}  // namespace

/**
 * @brief 在 facade 清旧图前校验 iG 配置、原生探针、搜索半径和平面阈值。
 * @throws std::invalid_argument 配置无效或启用公共几何窗口时抛出。
 */
void ValidateConfig(const MapManagerConfig &config)
{
    if (config.local_window_enabled)
    {
        throw std::invalid_argument("mapping/local_window_en is not supported by iG-LIO; it uses native LRU eviction");
    }
    if (config.nearest_point_count <= 0)
    {
        throw std::invalid_argument("mapping/nearest_point_count must be positive for iG-LIO");
    }

    const ig_lio_native::Config nativeConfig = MakeNativeConfig(config);
    ig_lio_native::VoxelMap probe(nativeConfig);
    static_cast<void>(probe);

    if (!ValidNearestMaxRange(config.nearest_max_range))
    {
        throw std::invalid_argument(
            "mapping/nearest_max_range must be positive and finite in metres, with a finite nonzero square");
    }
    if (!std::isfinite(config.plane_fit_threshold) || config.plane_fit_threshold < 0.0 ||
        config.plane_fit_threshold > static_cast<double>(std::numeric_limits<float>::max()))
    {
        throw std::invalid_argument("invalid mapping/plane_fit_threshold for iG-LIO");
    }
}

/** @brief 创建空原生地图；仅校验原生格参数。 */
std::unique_ptr<ig_lio_native::VoxelMap> Create(const MapManagerConfig &config)
{
    return std::make_unique<ig_lio_native::VoxelMap>(MakeNativeConfig(config));
}

/** @brief 完整构建替换地图，失败时不修改 facade 当前实例。 */
std::unique_ptr<ig_lio_native::VoxelMap> Build(const MapManagerConfig &config, const MapPointList &points)
{
    auto replacement = Create(config);
    replacement->AddCloud(ToWorldPoints(points));
    return replacement;
}

/** @brief 将一批世界系点交给 native；非法批次在 native 校验阶段整体拒绝。 */
void Update(ig_lio_native::VoxelMap &map, const MapPointList &points)
{
    map.AddCloud(ToWorldPoints(points));
}

/**
 * @brief 查询原生邻点并生成公共五点点面观测。
 * @note 本函数仅追加结果；调用 facade 先清空输出。改形格矩阵不作为平面协方差传递。
 */
void Search(const ig_lio_native::VoxelMap *map, const MapManagerConfig &config, const MapPointList &points,
            PlaneMatchList &matches, std::vector<V3D> &nonMatch)
{
    const int nearestCount = std::max(config.nearest_point_count, NUM_MATCH_POINTS);
    for (const auto &point : points)
    {
        if (!map || !point.point_lidar.allFinite() || !point.point_world.allFinite() || !point.cov_world.allFinite())
        {
            nonMatch.push_back(point.point_world);
            continue;
        }

        ig_lio_native::NeighborList found;
        if (!map->KNNByCondition(point.point_world, static_cast<std::size_t>(nearestCount), config.nearest_max_range,
                                 found) ||
            found.size() < NUM_MATCH_POINTS)
        {
            nonMatch.push_back(point.point_world);
            continue;
        }

        PointVector neighbors;
        neighbors.reserve(found.size());
        for (const auto &neighbor : found)
        {
            PointType nativePoint{};
            nativePoint.x = static_cast<float>(neighbor.point_.x());
            nativePoint.y = static_cast<float>(neighbor.point_.y());
            nativePoint.z = static_cast<float>(neighbor.point_.z());
            neighbors.push_back(nativePoint);
        }

        PlaneMatch match;
        if (!backend_point_plane::FitPointPlane(point, neighbors, config, MapType::IGLIOPlane, match) ||
            !match.normal.allFinite() || !match.center.allFinite() || !std::isfinite(match.d) ||
            !std::isfinite(match.distance) || !std::isfinite(match.normal.norm()) ||
            match.normal.norm() <= std::numeric_limits<double>::epsilon() ||
            !std::isfinite((match.normal.transpose() * point.cov_world * match.normal)(0, 0)))
        {
            nonMatch.push_back(point.point_world);
            continue;
        }
        matches.push_back(match);
    }
}

/** @brief 返回活动统计格数；空指针对应零。 */
std::size_t Size(const ig_lio_native::VoxelMap *map)
{
    return map ? map->Size() : 0;
}

/** @brief 返回实际保留 XYZ 点数；空指针对应零。 */
std::size_t PointCount(const ig_lio_native::VoxelMap *map)
{
    return map ? map->PointCount() : 0;
}

/** @brief 返回原生 LRU/格内顺序的存点快照；空指针对应空集合。 */
std::vector<Eigen::Vector3d> Snapshot(const ig_lio_native::VoxelMap *map)
{
    return map ? map->Snapshot() : std::vector<Eigen::Vector3d>();
}

}  // namespace pv_lio_plus::backend_ig_lio
