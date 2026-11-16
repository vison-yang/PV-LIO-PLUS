/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/**
 * @file point_plane.h
 * @brief 点图后端共享的点面拟合接口。
 */
#pragma once

#include "map_manager/map_config.h"
#include "map_manager/map_types.h"

namespace pv_lio_plus::backend_point_plane
{
bool FitPointPlane(const MapPoint &query, const PointVector &neighbors, const MapManagerConfig &config, MapType backend,
                   PlaneMatch &match);

}  // namespace pv_lio_plus::backend_point_plane
