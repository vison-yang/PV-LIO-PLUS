/** @file bievr_backend.h @brief 定向高度图的内部生命周期适配。 */
#pragma once

#include <memory>

#include "map_manager/map_config.h"
#include "map_manager/native/bievr/voxel_map.h"
#include "observation/measurement.h"

namespace pv_lio_plus::backend_bievr
{
bievr_native::Config MakeConfig(const MapManagerConfig &config);
std::unique_ptr<bievr_native::Map> Build(const MapManagerConfig &config, const MapPointList &points,
                                         const MapUpdateContext &context);
void Update(bievr_native::Map &map, const MapPointList &points, const MapUpdateContext &context);
bool Sample(const bievr_native::Map &map, const MapManagerConfig &config, const V3D &pointWorld,
            observation::HeightSample &sample);
}  // namespace pv_lio_plus::backend_bievr
