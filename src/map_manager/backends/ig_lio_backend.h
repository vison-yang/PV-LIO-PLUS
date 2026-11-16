/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/**
 * @file ig_lio_backend.h
 * @brief MapManager 内部 iG-LIO 点面变体接口。
 */
#pragma once

#include <memory>
#include <vector>

#include "map_manager/map_config.h"
#include "map_manager/map_types.h"
#include "map_manager/native/ig_lio/voxel_map.h"

namespace pv_lio_plus::backend_ig_lio
{
void ValidateConfig(const MapManagerConfig &config);

std::unique_ptr<ig_lio_native::VoxelMap> Create(const MapManagerConfig &config);

std::unique_ptr<ig_lio_native::VoxelMap> Build(const MapManagerConfig &config, const MapPointList &points);

void Update(ig_lio_native::VoxelMap &map, const MapPointList &points);

void Search(const ig_lio_native::VoxelMap *map, const MapManagerConfig &config, const MapPointList &points,
            PlaneMatchList &matches, std::vector<V3D> &nonMatch);

std::size_t Size(const ig_lio_native::VoxelMap *map);

std::size_t PointCount(const ig_lio_native::VoxelMap *map);

std::vector<Eigen::Vector3d> Snapshot(const ig_lio_native::VoxelMap *map);

}  // namespace pv_lio_plus::backend_ig_lio
