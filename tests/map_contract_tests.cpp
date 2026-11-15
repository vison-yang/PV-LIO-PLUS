/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

// The production translation unit includes the manager implementation for
// the same reason: imported native headers define non-inline functions.
#include "map_manager/map_manager.cpp"

namespace
{
using pv_lio_plus::MapManager;
using pv_lio_plus::MapManagerConfig;
using pv_lio_plus::MapPoint;
using pv_lio_plus::MapPointList;
using pv_lio_plus::MapType;
using pv_lio_plus::PlaneMatchList;

MapPoint Point(double x, double y, double z)
{
    MapPoint point;
    point.point_world = V3D(x, y, z);
    point.point_lidar = V3D(4.2, -1.1, 2.3);
    point.cov_lidar = M3D::Identity() * 0.01;
    point.cov_world = M3D::Identity() * 0.01;
    return point;
}

MapManagerConfig Config(MapType type)
{
    MapManagerConfig config;
    config.type = type;
    config.voxel_size = 4.0;
    config.max_layer = 2;
    config.layer_point_size = {5, 5, 5};
    config.plus_update_size_threshold = 5;
    config.ivox_resolution = 4.0;
    config.ivox_nearby_type = 26;
    config.octvox_resolution = 0.5;
    return config;
}

MapPointList Plane()
{
    MapPointList points;
    for (int x = 0; x < 9; ++x)
    {
        for (int y = 0; y < 9; ++y)
        {
            points.emplace_back(Point(0.6 + 0.1 * x, 0.6 + 0.1 * y, 1.0));
        }
    }
    return points;
}

PlaneMatchList Search(MapManager &manager, const MapPoint &query)
{
    PlaneMatchList matches;
    std::vector<V3D> non_matches;
    manager.search(MapPointList{query}, matches, non_matches);
    return matches;
}

class BackendLifecycle : public testing::TestWithParam<MapType>
{
};

TEST_P(BackendLifecycle, EmptyPlaneUpdateAndClear)
{
    const MapType type = GetParam();
    const MapPoint query = Point(1.05, 1.05, 1.01);
    const MapPoint far_query = Point(100.0, 100.0, 100.0);
    MapManager manager(Config(type));
    EXPECT_TRUE(manager.supports_selected_backend());
    EXPECT_FALSE(manager.initialized());
    EXPECT_EQ(manager.size(), 0u);
    EXPECT_TRUE(Search(manager, query).empty());
    manager.initialize(MapPointList{});
    EXPECT_TRUE(manager.initialized());
    EXPECT_EQ(manager.size(), 0u);
    EXPECT_TRUE(manager.snapshot()->empty());
    EXPECT_TRUE(Search(manager, query).empty());

    manager.initialize(Plane());
    EXPECT_TRUE(manager.initialized());
    EXPECT_GT(manager.size(), 0u);
    EXPECT_FALSE(manager.snapshot()->empty());
    PlaneMatchList matches = Search(manager, query);
    ASSERT_EQ(matches.size(), 1u);
    EXPECT_EQ(matches.front().map_type, type);
    EXPECT_LT((matches.front().point - query.point_lidar).norm(), 1e-9);
    EXPECT_NEAR(std::abs(matches.front().normal.z()), 1.0, 1e-4);
    EXPECT_NEAR(std::abs(matches.front().distance), 0.01, 1e-4);
    EXPECT_TRUE(Search(manager, far_query).empty());

    manager.update(MapPointList{Point(1.15, 1.15, 1.0)}, 1);
    EXPECT_GT(manager.size(), 0u);
    EXPECT_EQ(Search(manager, query).size(), 1u);
    manager.clear();
    EXPECT_FALSE(manager.initialized());
    EXPECT_EQ(manager.size(), 0u);
    EXPECT_EQ(manager.point_count(), 0u);
    EXPECT_TRUE(manager.snapshot()->empty());
    EXPECT_TRUE(Search(manager, query).empty());
    manager.update(Plane(), 2);
    EXPECT_TRUE(manager.initialized());
    EXPECT_GT(manager.size(), 0u);
    EXPECT_EQ(Search(manager, query).size(), 1u);
}

INSTANTIATE_TEST_SUITE_P(SixSelectors, BackendLifecycle,
                         testing::Values(MapType::VoxelMap, MapType::VoxelMapPlus, MapType::IKDTree, MapType::IVox,
                                         MapType::C3PVoxelMap, MapType::OctVox),
                         [](const testing::TestParamInfo<MapType> &info) {
                             return pv_lio_plus::MapTypeName(info.param);
                         });

MapPointList EquidistantPlane()
{
    // Five distinct points, each exactly 5 m from (10, 10, 1), on z=1.
    return {Point(15, 10, 1), Point(5, 10, 1), Point(10, 15, 1), Point(10, 5, 1), Point(13, 14, 1)};
}

TEST(PointMapRadius, MetresAndNativeBoundaryForBothIvoxImplementations)
{
    const MapPoint query = Point(10, 10, 1);
    for (const MapType type : {MapType::IKDTree, MapType::IVox})
    {
        for (const bool optimized : {false, true})
        {
            if (type == MapType::IKDTree && optimized)
            {
                continue;
            }
            for (const double radius : {4.99, 5.0, 5.01})
            {
                SCOPED_TRACE(std::string(pv_lio_plus::MapTypeName(type)) +
                             " optimized=" + std::to_string(optimized) + " radius=" + std::to_string(radius));
                MapManagerConfig config = Config(type);
                config.ivox_resolution = 32.0;  // all five points are within the queried grid
                config.ivox_optimized = optimized;
                config.nearest_max_range = radius;
                MapManager manager(config);
                manager.initialize(EquidistantPlane());
                const bool expected = radius > 5.0 || (type == MapType::IKDTree && radius == 5.0);
                EXPECT_EQ(Search(manager, query).size(), expected ? 1u : 0u);
            }
        }
    }
}

TEST(PointMapRadius, IvoxSearchesConfiguredGridNeighborhoodRatherThanWholeRadius)
{
    const MapPoint query = Point(10, 10, 1);
    const MapPointList points = {Point(10, 10, 1), Point(11, 10, 1), Point(9, 10, 1),
                                 Point(10, 11, 1), Point(10, 9, 1)};
    for (const bool optimized : {false, true})
    {
        for (const int nearby_type : {0, 26})
        {
            SCOPED_TRACE(std::to_string(optimized) + "/" + std::to_string(nearby_type));
            MapManagerConfig config = Config(MapType::IVox);
            config.ivox_resolution = 1.0;
            config.ivox_optimized = optimized;
            config.ivox_nearby_type = nearby_type;
            config.nearest_max_range = 2.0;
            MapManager manager(config);
            manager.initialize(points);
            EXPECT_EQ(Search(manager, query).size(), nearby_type == 0 ? 0u : 1u);
        }
    }
    MapManagerConfig config = Config(MapType::IKDTree);
    config.nearest_max_range = 2.0;
    MapManager global_map(config);
    global_map.initialize(points);
    EXPECT_EQ(Search(global_map, query).size(), 1u);
}

TEST(PointMapRadius, RejectsInvalidConfigurationBeforeMutatingTheMap)
{
    const std::array<double, 7> invalid = {0.0, -1.0, std::numeric_limits<double>::infinity(),
                                           -std::numeric_limits<double>::infinity(),
                                           std::numeric_limits<double>::quiet_NaN(),
                                           std::numeric_limits<double>::max(),
                                           std::numeric_limits<double>::denorm_min()};
    for (const MapType type : {MapType::IKDTree, MapType::IVox})
    {
        SCOPED_TRACE(pv_lio_plus::MapTypeName(type));
        MapManagerConfig config = Config(type);
        MapManager manager(config);
        manager.initialize(EquidistantPlane());
        const std::size_t old_size = manager.size();
        for (double radius : invalid)
        {
            SCOPED_TRACE(radius);
            config.nearest_max_range = radius;
            EXPECT_THROW(manager.configure(config), std::invalid_argument);
            EXPECT_TRUE(manager.initialized());
            EXPECT_EQ(manager.size(), old_size);
            EXPECT_DOUBLE_EQ(manager.config().nearest_max_range, 5.0);
        }
    }
}

TEST(OctVoxWindow, RejectsUnsupportedConfigurationAndExplicitDeletion)
{
    MapManagerConfig config = Config(MapType::OctVox);
    MapManager manager(config);
    manager.initialize(Plane());
    const std::size_t old_size = manager.size();
    config.local_window_enabled = true;
    EXPECT_THROW(manager.configure(config), std::invalid_argument);
    EXPECT_EQ(manager.size(), old_size);
    EXPECT_FALSE(manager.config().local_window_enabled);
    pv_lio_plus::MapWindow window;
    window.initialized = true;
    window.min_bound = V3D::Zero();
    window.max_bound = V3D::Ones();
    EXPECT_THROW(manager.erase_outside_window(window), std::logic_error);
    EXPECT_EQ(manager.size(), old_size);
}
}  // namespace

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
