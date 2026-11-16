/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/**
 * @file point_plane.cpp
 * @brief 点图后端共享的点面拟合实现。
 */

#include "point_plane.h"

#include <cmath>

namespace pv_lio_plus::backend_point_plane
{
/**
 * @brief 使用原点图拟合门限和后端残差条件生成平面匹配。
 * @param query 当前查询点及协方差。
 * @param neighbors 世界系邻点。
 * @param config 当前地图配置。
 * @param backend 应用原生残差门限的后端。
 * @param match 输出匹配。
 * @return 邻点数量、拟合及后端门限均通过时返回 true。
 */
bool FitPointPlane(const MapPoint &query, const PointVector &neighbors, const MapManagerConfig &config,
                   const MapType backend, PlaneMatch &match)
{
    if (neighbors.size() < NUM_MATCH_POINTS)
    {
        return false;
    }

    VF(4) planeCoeff;
    if (!esti_plane(planeCoeff, neighbors, static_cast<float>(config.plane_fit_threshold)))
    {
        return false;
    }

    const V3D pointWorld = query.point_world;
    const V3D normal(planeCoeff(0), planeCoeff(1), planeCoeff(2));
    const double pd2 = normal.dot(pointWorld) + planeCoeff(3);

    // ikd-tree 和 iVox 沿用各自原有的残差门限；其他后端不增加筛选。
    if (backend == MapType::IKDTree)
    {
        const double score = 1.0 - 0.9 * std::abs(pd2) / std::sqrt(query.point_lidar.norm());
        if (score <= 0.9)
        {
            return false;
        }
    }
    else if (backend == MapType::IVox && query.point_lidar.norm() <= 81.0 * pd2 * pd2)
    {
        return false;
    }

    match.map_type    = backend;
    match.point       = query.point_lidar;
    match.point_world = pointWorld;
    match.normal      = normal;
    match.omega       = normal;
    match.omega_norm  = normal.norm();
    match.d           = planeCoeff(3);
    match.distance    = pd2;
    match.point_cov   = query.cov_world;
    match.plane_cov.setZero();
    match.layer = 0;

    match.center.setZero();
    for (int i = 0; i < NUM_MATCH_POINTS; ++i)
    {
        match.center += V3D(neighbors[i].x, neighbors[i].y, neighbors[i].z);
    }
    match.center /= static_cast<double>(NUM_MATCH_POINTS);
    return true;
}

}  // namespace pv_lio_plus::backend_point_plane
