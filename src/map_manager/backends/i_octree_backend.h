/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/**
 * @file i_octree_backend.h
 * @brief MapManager 内部 i-Octree 操作接口。
 */
#pragma once

#include <memory>
#include <vector>

#include "map_manager/map_config.h"
#include "map_manager/map_types.h"
#include "map_manager/native/i_octree/i_octree.hpp"

namespace pv_lio_plus::backend_i_octree
{
void ValidateConfig(const MapManagerConfig &config);

std::unique_ptr<i_octree_native::Octree> Create(const MapManagerConfig &config);

std::unique_ptr<i_octree_native::Octree> Build(const MapManagerConfig &config, const MapPointList &points);

void Update(i_octree_native::Octree &tree, const MapManagerConfig &config, const MapPointList &points);

void Search(const i_octree_native::Octree *tree, const MapManagerConfig &config, const MapPointList &points,
            PlaneMatchList &matches, std::vector<V3D> &nonMatch);

std::size_t Retain(i_octree_native::Octree &tree, const MapWindow &window);

std::size_t Size(const i_octree_native::Octree *tree);

std::vector<i_octree_native::Point3f> Snapshot(const i_octree_native::Octree *tree);

}  // namespace pv_lio_plus::backend_i_octree
