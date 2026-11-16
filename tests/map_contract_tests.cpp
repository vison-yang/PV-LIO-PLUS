/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/** @file map_contract_tests.cpp @brief 地图生命周期、重配置及原生匹配契约测试。 */

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cfenv>
#include <cmath>
#include <limits>
#include <tuple>
#include <vector>

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

/** 对比无序 XYZ 快照，避免把原生容器的遍历顺序作为公共契约。 */
void ExpectSameSnapshot(const PointCloudXYZI::Ptr &actual, const PointCloudXYZI::Ptr &expected)
{
    const auto coordinates = [](const PointCloudXYZI::Ptr &cloud) {
        std::vector<std::tuple<float, float, float>> result;
        result.reserve(cloud->size());
        for (const auto &point : *cloud)
        {
            result.emplace_back(point.x, point.y, point.z);
        }
        std::sort(result.begin(), result.end());
        return result;
    };
    EXPECT_EQ(coordinates(actual), coordinates(expected));
}

/** 已有地图切换后端后必须清除旧状态，新后端仍可独立建图和查询。 */
TEST(MapReconfigure, CrossBackendSwitchClearsOldState)
{
    auto config                 = Config(MapType::IOctree);
    config.local_window_enabled = true;
    MapManager manager(config);
    manager.initialize(Plane());
    manager.move_window(V3D::Ones(), V3D::Constant(10.0));
    ASSERT_TRUE(manager.window().initialized);
    const MapPoint query = Point(1.05, 1.05, 1.01);

    for (const MapType type : {MapType::VoxelMap, MapType::VoxelMapPlus, MapType::IKDTree, MapType::IVox,
                               MapType::C3PVoxelMap, MapType::OctVox, MapType::IOctree})
    {
        SCOPED_TRACE(pv_lio_plus::MapTypeName(type));
        ASSERT_TRUE(manager.initialized());
        ASSERT_EQ(Search(manager, query).size(), 1u);
        const auto oldRevision = manager.revision();
        manager.configure(Config(type));

        EXPECT_EQ(manager.type(), type);
        EXPECT_EQ(manager.config().type, type);
        EXPECT_FALSE(manager.initialized());
        EXPECT_FALSE(manager.window().initialized);
        EXPECT_EQ(manager.size(), 0u);
        EXPECT_EQ(manager.point_count(), 0u);
        EXPECT_TRUE(manager.snapshot()->empty());
        EXPECT_TRUE(Search(manager, query).empty());
        EXPECT_EQ(manager.revision(), oldRevision + 2);  // 清旧图和应用配置各递增一次。

        manager.initialize(Plane());
        EXPECT_TRUE(manager.initialized());
        EXPECT_GT(manager.size(), 0u);
        EXPECT_EQ(Search(manager, query).size(), 1u);
    }
}

/** i-Octree 参数校验失败时，连同跨后端切换在内的旧图可见状态必须保留。 */
TEST(MapReconfigure, InvalidIOctreeConfigPreservesExistingMap)
{
    std::vector<MapManagerConfig> invalidConfigs(5, Config(MapType::IOctree));
    invalidConfigs[0].i_octree_bucket_size = 0;
    invalidConfigs[1].nearest_point_count  = 0;
    invalidConfigs[2].plane_fit_threshold  = std::numeric_limits<double>::quiet_NaN();
    invalidConfigs[3].i_octree_min_extent  = 0.0;
    invalidConfigs[4].nearest_max_range    = std::numeric_limits<double>::infinity();

    for (const MapType type : {MapType::VoxelMap, MapType::IOctree})
    {
        SCOPED_TRACE(pv_lio_plus::MapTypeName(type));
        auto config                 = Config(type);
        config.local_window_enabled = true;
        MapManager manager(config);
        manager.initialize(Plane());
        manager.move_window(V3D::Ones(), V3D::Constant(10.0));
        const auto oldSnapshot   = manager.snapshot();
        const auto oldRevision   = manager.revision();
        const auto oldSize       = manager.size();
        const auto oldPointCount = manager.point_count();
        const auto oldWindow     = manager.window();
        const MapPoint query     = Point(1.05, 1.05, 1.01);
        const auto oldMatches    = Search(manager, query);
        ASSERT_EQ(oldMatches.size(), 1u);

        for (std::size_t i = 0; i < invalidConfigs.size(); ++i)
        {
            SCOPED_TRACE(i);
            EXPECT_THROW(manager.configure(invalidConfigs[i]), std::invalid_argument);
            EXPECT_EQ(manager.type(), type);
            EXPECT_TRUE(manager.initialized());
            EXPECT_EQ(manager.size(), oldSize);
            EXPECT_EQ(manager.point_count(), oldPointCount);
            EXPECT_EQ(manager.revision(), oldRevision);
            EXPECT_EQ(manager.config().i_octree_bucket_size, config.i_octree_bucket_size);
            EXPECT_EQ(manager.config().nearest_point_count, config.nearest_point_count);
            EXPECT_DOUBLE_EQ(manager.config().plane_fit_threshold, config.plane_fit_threshold);
            EXPECT_DOUBLE_EQ(manager.config().i_octree_min_extent, config.i_octree_min_extent);
            EXPECT_DOUBLE_EQ(manager.config().nearest_max_range, config.nearest_max_range);
            EXPECT_EQ(manager.window().initialized, oldWindow.initialized);
            EXPECT_TRUE(manager.window().min_bound.isApprox(oldWindow.min_bound));
            EXPECT_TRUE(manager.window().max_bound.isApprox(oldWindow.max_bound));
            ExpectSameSnapshot(manager.snapshot(), oldSnapshot);
            const auto matches = Search(manager, query);
            ASSERT_EQ(matches.size(), oldMatches.size());
            EXPECT_TRUE(matches.front().normal.isApprox(oldMatches.front().normal));
            EXPECT_TRUE(matches.front().center.isApprox(oldMatches.front().center));
            EXPECT_DOUBLE_EQ(matches.front().distance, oldMatches.front().distance);
        }
    }
}

/** 空树和空查询仍须验证浮点模式，不能由适配器提前返回而绕过。 */
TEST(MapReconfigure, EmptyIOctreeSearchChecksNumericEnvironment)
{
    MapManager manager(Config(MapType::IOctree));
    manager.clear();
    const auto oldRevision = manager.revision();
    const int oldRound     = std::fegetround();
    ASSERT_NE(oldRound, -1);
    pv_lio_plus::PlaneMatchList matches(1);
    std::vector<V3D> nonMatch(1, V3D::Ones());
    if (std::fesetround(FE_DOWNWARD) != 0)
    {
        GTEST_SKIP() << "当前平台不支持设置向下舍入模式";
    }
    EXPECT_THROW(manager.search(MapPointList{}, matches, nonMatch), std::runtime_error);
    // 不在恢复前使用致命断言，避免失败时污染后续测试的线程浮点环境。
    EXPECT_EQ(std::fesetround(oldRound), 0);
    EXPECT_TRUE(matches.empty());
    EXPECT_TRUE(nonMatch.empty());
    EXPECT_EQ(manager.revision(), oldRevision);
}

/** 将公共查询显式转换为原生接口输入，覆盖两个协方差坐标系。 */
template <typename NativePoint>
NativePoint NativeQuery(const MapPoint &point)
{
    NativePoint result;
    result.point_lidar = point.point_lidar;
    result.point_world = point.point_world;
    result.cov_lidar   = point.cov_lidar;
    result.cov_world   = point.cov_world;
    return result;
}

/** VoxelMap 原生搜索须保留公共滤波实际消费的字段，并拒绝错误后端。 */
TEST(NativeMapSearch, VoxelMapFieldsAndWrongBackend)
{
    MapPoint query = Point(1.05, 1.05, 1.01);
    query.cov_lidar *= 2.0;  // 两坐标系取不同协方差，以捕获字段误传。
    MapManager manager(Config(MapType::VoxelMap));
    manager.initialize(Plane());
    const std::vector<voxel_map_ns::pointWithCov> points{NativeQuery<voxel_map_ns::pointWithCov>(query)};
    std::vector<voxel_map_ns::ptpl> matches;
    std::vector<V3D> nonMatch;
    manager.search(points, matches, nonMatch);
    ASSERT_EQ(matches.size(), 1u);
    const auto &match = matches.front();
    EXPECT_TRUE(match.point.isApprox(query.point_lidar));
    EXPECT_TRUE(match.point_world.isApprox(query.point_world));
    EXPECT_TRUE(match.cov_lidar.isApprox(query.cov_lidar));
    EXPECT_TRUE(match.cov_world.isApprox(query.cov_world));
    EXPECT_TRUE(match.plane_cov.allFinite());
    EXPECT_NEAR(std::abs(match.normal.z()), 1.0, 1e-4);
    EXPECT_NEAR(match.center.z(), 1.0, 1e-4);
    EXPECT_NEAR(std::abs(match.normal.dot(query.point_world) + match.d), 0.01, 1e-4);

    manager.configure(Config(MapType::VoxelMapPlus));
    EXPECT_THROW(manager.search(points, matches, nonMatch), std::logic_error);
}

/** VoxelMap++ 保留非单位 omega 及原生距离，不经公共平面匹配转换。 */
TEST(NativeMapSearch, VoxelMapPlusFieldsAndWrongBackend)
{
    // 斜平面 z=1+0.2x+0.3y，避免只用单位 omega 的水平面掩盖归一化错误。
    auto plane = Plane();
    for (auto &point : plane)
    {
        point.point_world.z() += 0.2 * point.point_world.x() + 0.3 * point.point_world.y();
    }
    MapPoint query = Point(1.05, 1.05, 1.0 + 0.2 * 1.05 + 0.3 * 1.05 + 0.01);
    query.cov_lidar *= 2.0;
    const double normalNorm = std::sqrt(1.0 + 0.2 * 0.2 + 0.3 * 0.3);
    MapManager manager(Config(MapType::VoxelMapPlus));
    manager.initialize(plane);
    const std::vector<voxel_map_plus_ns::pointWithCov> points{NativeQuery<voxel_map_plus_ns::pointWithCov>(query)};
    std::vector<voxel_map_plus_ns::ptpl> matches;
    std::vector<V3D> nonMatch;
    manager.search(points, matches, nonMatch);
    ASSERT_EQ(matches.size(), 1u);
    const auto &match = matches.front();
    EXPECT_TRUE(match.point.isApprox(query.point_lidar));
    EXPECT_TRUE(match.point_world.isApprox(query.point_world));
    EXPECT_TRUE(match.point_cov.isApprox(query.cov_world));
    EXPECT_TRUE(match.plane_cov.allFinite());
    EXPECT_EQ(match.main_direction, 0);
    EXPECT_GT(match.omega_norm, 1.0);
    EXPECT_NEAR(match.omega_norm, match.omega.norm(), 1e-12);
    EXPECT_NEAR(match.omega.x(), -0.2, 1e-4);
    EXPECT_NEAR(match.omega.y(), -0.3, 1e-4);
    EXPECT_NEAR(match.omega.z(), 1.0, 1e-4);
    EXPECT_NEAR(match.omega_norm, normalNorm, 1e-4);
    EXPECT_NEAR(match.dist, 0.01 / normalNorm, 1e-4);

    manager.configure(Config(MapType::VoxelMap));
    EXPECT_THROW(manager.search(points, matches, nonMatch), std::logic_error);
}
}  // namespace

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
