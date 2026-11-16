/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/**
 * @file i_octree_backend.cpp
 * @brief MapManager 内部 i-Octree 操作实现。
 */

#include "i_octree_backend.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "../point_plane.h"

namespace pv_lio_plus::backend_i_octree
{
namespace
{
/** @brief 校验 i-Octree 参数并映射为原生树配置。 */
i_octree_native::Config MakeNativeConfig(const MapManagerConfig &config)
{
    if (config.i_octree_bucket_size <= 0 || config.nearest_point_count <= 0 ||
        !std::isfinite(config.plane_fit_threshold) || config.plane_fit_threshold < 0.0 ||
        config.plane_fit_threshold > double(std::numeric_limits<float>::max()))
    {
        throw std::invalid_argument("invalid i-Octree bucket, neighbor count, or plane threshold");
    }

    i_octree_native::Config native;
    native.bucket_size = static_cast<std::size_t>(config.i_octree_bucket_size);
    native.min_extent  = config.i_octree_min_extent;
    return native;
}

/** @brief 将地图点的世界坐标按原顺序转换为原生输入。 */
std::vector<i_octree_native::Point3d> ToNativePoints(const MapPointList &points)
{
    std::vector<i_octree_native::Point3d> native;
    native.reserve(points.size());
    for (const auto &point : points)
    {
        native.push_back({point.point_world.x(), point.point_world.y(), point.point_world.z()});
    }
    return native;
}
}  // namespace

/** @brief 以栈上原生探针校验树配置，再按原顺序校验搜索半径。 */
void ValidateConfig(const MapManagerConfig &config)
{
    const auto nativeConfig = MakeNativeConfig(config);
    i_octree_native::Octree probe(nativeConfig);
    static_cast<void>(probe);
    static_cast<void>(i_octree_native::radius_squared(config.nearest_max_range));
}

/** @brief 按配置创建空树；构造过程也会校验原生 extent 参数。 */
std::unique_ptr<i_octree_native::Octree> Create(const MapManagerConfig &config)
{
    return std::make_unique<i_octree_native::Octree>(MakeNativeConfig(config));
}

/** @brief 先完整构建替换树，失败时不触碰 facade 当前地图。 */
std::unique_ptr<i_octree_native::Octree> Build(const MapManagerConfig &config, const MapPointList &points)
{
    auto replacement = Create(config);
    replacement->initialize(ToNativePoints(points));
    return replacement;
}

/** @brief 将一批世界坐标点追加到树中。 */
void Update(i_octree_native::Octree &tree, const MapManagerConfig &config, const MapPointList &points)
{
    tree.update(ToNativePoints(points), config.i_octree_on_tree_downsample);
}

/**
 * @brief 查询最近邻并按 i-Octree 的距离、拟合及有限值规则筛选平面。
 * @note 调用方先清空输出；本函数只追加结果。距离单位为平方米。
 */
void Search(const i_octree_native::Octree *tree, const MapManagerConfig &config, const MapPointList &points,
            PlaneMatchList &matches, std::vector<V3D> &nonMatch)
{
    const int nearestCount = std::max(config.nearest_point_count, NUM_MATCH_POINTS);
    // 即使树为空或查询为空，也保留原生半径函数的数值环境检查。
    const double maxSquared      = i_octree_native::radius_squared(config.nearest_max_range);
    const double coordinateLimit = double(std::numeric_limits<float>::max());
    for (const auto &point : points)
    {
        if (!tree || !point.point_lidar.allFinite() || !point.point_world.allFinite() || !point.cov_world.allFinite() ||
            (point.point_world.array().abs() > coordinateLimit).any())
        {
            nonMatch.push_back(point.point_world);
            continue;
        }

        std::vector<i_octree_native::Neighbor> found;
        tree->knn({point.point_world.x(), point.point_world.y(), point.point_world.z()}, nearestCount, found);
        if (found.size() < NUM_MATCH_POINTS || found[NUM_MATCH_POINTS - 1].distance_squared > maxSquared)
        {
            nonMatch.push_back(point.point_world);
            continue;
        }

        PointVector neighbors;
        neighbors.reserve(found.size());
        for (const auto &neighbor : found)
        {
            PointType nativePoint{};
            nativePoint.x = neighbor.point.x;
            nativePoint.y = neighbor.point.y;
            nativePoint.z = neighbor.point.z;
            neighbors.push_back(nativePoint);
        }

        PlaneMatch match;
        // 新后端额外拒绝退化 QR 产生的非有限系数及无效法向/协方差。
        if (!backend_point_plane::FitPointPlane(point, neighbors, config, MapType::IOctree, match) ||
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

/** @brief 将树裁剪到闭区间窗口，边界点仍保留。 */
std::size_t Retain(i_octree_native::Octree &tree, const MapWindow &window)
{
    return tree.retain_box({window.min_bound.x(), window.min_bound.y(), window.min_bound.z()},
                           {window.max_bound.x(), window.max_bound.y(), window.max_bound.z()});
}

/** @brief 返回树中原生点数；空树指针对应零。 */
std::size_t Size(const i_octree_native::Octree *tree)
{
    return tree ? tree->size() : 0;
}

/** @brief 返回叶节点点云快照；空树指针对应空结果。 */
std::vector<i_octree_native::Point3f> Snapshot(const i_octree_native::Octree *tree)
{
    return tree ? tree->snapshot() : std::vector<i_octree_native::Point3f>();
}

}  // namespace pv_lio_plus::backend_i_octree
