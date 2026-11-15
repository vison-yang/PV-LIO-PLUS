/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <vector>

// Imported map headers define non-inline functions, so production and this
// integration test each include the manager implementation in one unit.
#include "map_manager/map_manager.cpp"
#include "map_manager/r_voxelmap_observation.h"
#include "utils.hpp"

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
    MapPoint p;
    p.point_world = V3D(x, y, z);
    p.point_lidar = V3D(x + 0.4, y - 0.3, z + 0.2);
    p.cov_lidar = M3D::Identity() * 0.002;
    p.cov_world = M3D::Identity() * 0.01;
    return p;
}

MapManagerConfig Config()
{
    MapManagerConfig c;
    c.type = MapType::RVoxelMap;
    c.voxel_size = 4.0;
    c.max_layer = 2;
    c.layer_point_size = {5, 5, 5};
    c.plane_threshold = 0.01;
    c.r_ransac_max_iter = 100;
    c.r_ransac_sample_num = 4;
    c.r_ransac_inlier_distance_threshold = 0.02;
    c.r_ransac_isplane_p_threshold = 0.65;
    c.r_update_inlier_distance_threshold = 0.02;
    c.r_valid_check_max_layer = -1;
    c.r_reinit_size_vec = {1000, 2000};
    c.r_max_root_points_size = 10000;
    return c;
}

MapPointList TwoPlanes()
{
    MapPointList out;
    for (int i = 0; i < 10; ++i)
        for (int j = 0; j < 10; ++j)
            out.push_back(Point(0.25 + 0.125 * i, 0.25 + 0.125 * j, 1.0));
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            out.push_back(Point(3.25, 0.25 + 0.125 * i, 2.25 + 0.125 * j));
    return out;
}

std::vector<r_voxelmap_ns::pointWithCov> NativePoints(const MapPointList &points)
{
    std::vector<r_voxelmap_ns::pointWithCov> result;
    for (const auto &p : points) result.push_back({p.point_world, p.cov_world});
    return result;
}

std::vector<std::tuple<double, double, double>> Coordinates(const MapPointList &points)
{
    std::vector<std::tuple<double, double, double>> result;
    for (const auto &p : points) result.emplace_back(p.point_world.x(), p.point_world.y(), p.point_world.z());
    std::sort(result.begin(), result.end());
    return result;
}

std::vector<std::tuple<double, double, double>> Coordinates(
    const std::vector<r_voxelmap_ns::pointWithCov> &points)
{
    std::vector<std::tuple<double, double, double>> result;
    for (const auto &p : points) result.emplace_back(p.point.x(), p.point.y(), p.point.z());
    std::sort(result.begin(), result.end());
    return result;
}

void ExpectNoPlaneAndFiniteCovariance(const r_voxelmap_ns::OctoTree *node)
{
    ASSERT_NE(node, nullptr);
    EXPECT_FALSE(node->plane_ptr_->is_plane);
    EXPECT_TRUE(node->plane_ptr_->plane_cov.allFinite());
    for (const auto *leaf : node->leaves_)
        if (leaf != nullptr) ExpectNoPlaneAndFiniteCovariance(leaf);
}

PlaneMatchList Search(MapManager &map, const MapPoint &point, std::vector<V3D> &unmatched)
{
    PlaneMatchList result;
    map.search(MapPointList{point}, result, unmatched);
    return result;
}

TEST(RVoxelMapStructure, InternalAndChildPlanesRetainEveryPointAcrossRebuild)
{
    MapManagerConfig c = Config();
    c.r_reinit_size_vec = {150, 160};
    r_voxelmap_ns::VoxelMap map;
    map.init(pv_lio_plus::MakeRVoxelConfig(c));
    MapPointList all = TwoPlanes();
    map.build(NativePoints(all));
    ASSERT_EQ(map.feat_map.size(), 1u);
    const auto &root = map.feat_map.begin()->second;
    ASSERT_TRUE(root->plane_ptr_->is_plane);
    ASSERT_EQ(root->octo_state_, 1);
    EXPECT_GT(std::abs(root->plane_ptr_->normal.z()), 0.99);
    ASSERT_NE(root->leaves_[5], nullptr);
    EXPECT_TRUE(root->leaves_[5]->plane_ptr_->is_plane);
    EXPECT_GT(std::abs(root->leaves_[5]->plane_ptr_->normal.x()), 0.99);
    EXPECT_EQ(root->point_count(), all.size());
    std::vector<r_voxelmap_ns::pointWithCov> snapshot;
    root->append_points(snapshot);
    EXPECT_EQ(Coordinates(snapshot), Coordinates(all));

    MapPointList added;
    for (int i = 0; i < 50; ++i) added.push_back(Point(0.3 + 0.01 * i, 1.5, 1.0));
    map.update(NativePoints(added));
    all.insert(all.end(), added.begin(), added.end());
    EXPECT_TRUE(root->stop_reinit_);
    EXPECT_EQ(root->reinit_state_, 2);
    EXPECT_EQ(root->point_count(), all.size());
    snapshot.clear();
    root->append_points(snapshot);
    EXPECT_EQ(Coordinates(snapshot), Coordinates(all));
    EXPECT_TRUE(root->plane_ptr_->is_plane);
    EXPECT_EQ(root->octo_state_, 1);
    map.update(NativePoints(MapPointList{Point(0.4, 1.6, 1.0)}));
    EXPECT_TRUE(root->stop_reinit_);
    EXPECT_EQ(root->reinit_state_, 2);
    EXPECT_EQ(root->point_count(), all.size() + 1);
}

TEST(RVoxelMapStructure, NativeConnectivityCheckAndNonfinitePlaneCovariance)
{
    MapManagerConfig c = Config();
    c.r_valid_check_max_layer = 0;
    c.r_valid_check_min_points_size = 10;
    c.r_valid_check_resolution = 5;
    r_voxelmap_ns::VoxelMap map;
    map.init(pv_lio_plus::MakeRVoxelConfig(c));
    std::vector<r_voxelmap_ns::ptpl> matches;
    std::vector<int> indices;
    std::vector<V3D> unmatched;
    const auto query = NativePoints(MapPointList{Point(0.8, 0.8, 1.01)});
    map.build_residual_omp(query, c.max_layer, matches, indices, unmatched);
    EXPECT_TRUE(matches.empty());
    EXPECT_EQ(unmatched.size(), 1u);
    map.build(NativePoints(TwoPlanes()));
    map.build_residual_omp(query, c.max_layer, matches, indices, unmatched);
    ASSERT_EQ(matches.size(), 1u);
    ASSERT_EQ(matches.front().layer, 0);
    ASSERT_EQ(map.feat_map.size(), 1u);
    map.feat_map.begin()->second->plane_ptr_->plane_cov(0, 0) = std::numeric_limits<double>::quiet_NaN();
    map.build_residual_omp(query, c.max_layer, matches, indices, unmatched);
    EXPECT_TRUE(matches.empty());
    EXPECT_EQ(unmatched.size(), 1u);
}

TEST(RVoxelMapDegenerate, DuplicateAndCollinearPointsAreRetainedWithoutPlanes)
{
    const MapManagerConfig c = Config();
    MapPointList duplicates(8, Point(0.5, 0.5, 1.0));
    MapPointList line;
    for (int i = 0; i < 12; ++i) line.push_back(Point(0.25 + 0.125 * i, 0.5, 1.0));
    MapPointList translated_line;
    for (int i = 0; i < 12; ++i)
    {
        const double t = static_cast<double>(i) / 11.0;
        translated_line.push_back(Point(100.1 + 0.3 * t, 100.2 + 0.7 * t, 100.3 + 1.1 * t));
    }
    for (const MapPointList *points : {&duplicates, &line, &translated_line})
    {
        r_voxelmap_ns::VoxelMap map;
        map.init(pv_lio_plus::MakeRVoxelConfig(c));
        map.build(NativePoints(*points));
        ASSERT_EQ(map.feat_map.size(), 1u);
        const auto *root = map.feat_map.begin()->second;
        ExpectNoPlaneAndFiniteCovariance(root);
        EXPECT_EQ(root->point_count(), points->size());
        std::vector<r_voxelmap_ns::pointWithCov> stored;
        root->append_points(stored);
        EXPECT_EQ(Coordinates(stored), Coordinates(*points));
        std::vector<r_voxelmap_ns::ptpl> matches;
        std::vector<int> indices;
        std::vector<V3D> unmatched;
        map.build_residual_omp(NativePoints(MapPointList{points->front()}), c.max_layer,
                               matches, indices, unmatched);
        EXPECT_TRUE(matches.empty());
        EXPECT_EQ(unmatched.size(), 1u);
    }
}

TEST(RVoxelMapDegenerate, RebuildAndIncrementalCovarianceRetainDegeneratePoints)
{
    MapManagerConfig c = Config();
    c.r_reinit_size_vec = {8, 9};
    MapPointList points(8, Point(0.5, 0.5, 1.0));
    r_voxelmap_ns::VoxelMap map;
    const auto native_config = pv_lio_plus::MakeRVoxelConfig(c);
    map.init(native_config);
    map.build(NativePoints(points));
    ASSERT_EQ(map.feat_map.size(), 1u);
    auto *root = map.feat_map.begin()->second;
    MapPointList added(3, Point(0.5, 0.5, 1.0));
    map.update(NativePoints(added));
    points.insert(points.end(), added.begin(), added.end());
    EXPECT_TRUE(root->stop_reinit_);
    EXPECT_EQ(root->reinit_state_, 2);
    ExpectNoPlaneAndFiniteCovariance(root);
    EXPECT_EQ(root->point_count(), points.size());
    std::vector<r_voxelmap_ns::pointWithCov> stored;
    root->append_points(stored);
    EXPECT_EQ(Coordinates(stored), Coordinates(points));

    // Exercise the incremental covariance guard directly with a deliberately
    // degenerate existing plane, including its reinitialization path.
    r_voxelmap_ns::OctoTree node(0, native_config);
    node.root = &node;
    node.voxel_center_[0] = node.voxel_center_[1] = node.voxel_center_[2] = 2.0;
    node.quater_length_ = 1.0f;
    node.init_octo_ = true;
    node.plane_ptr_->is_plane = true;
    node.plane_ptr_->is_init = true;
    node.plane_ptr_->points_size = 8;
    node.plane_ptr_->center = points.front().point_world;
    node.plane_ptr_->sum_ppt = 8.0 * node.plane_ptr_->center * node.plane_ptr_->center.transpose();
    node.plane_ptr_->covariance.setZero();
    for (const auto &p : NativePoints(MapPointList(8, points.front()))) node.plane_points_.push_back(p);
    EXPECT_FALSE(node.check_and_update(NativePoints(MapPointList{points.front()}).front()));
    node.update_plane_cov();
    ExpectNoPlaneAndFiniteCovariance(&node);
    EXPECT_EQ(node.point_count(), 8u);
}

TEST(RVoxelMapAdapter, SearchesParentAndChildAndRestoresSourceByIndex)
{
    MapManager map(Config());
    map.initialize(TwoPlanes());
    const MapPoint parent = Point(0.8, 0.8, 1.01);
    const MapPoint child = Point(3.26, 0.6, 2.6);
    const MapPoint far = Point(20, 20, 20);
    PlaneMatchList matches;
    std::vector<V3D> unmatched;
    map.search(MapPointList{far, parent, child}, matches, unmatched);
    ASSERT_EQ(matches.size(), 2u);
    ASSERT_EQ(unmatched.size(), 1u);
    EXPECT_EQ(matches[0].map_type, MapType::RVoxelMap);
    EXPECT_EQ(matches[0].layer, 0);
    EXPECT_EQ(matches[1].layer, 1);
    EXPECT_GT(std::abs(matches[0].normal.z()), 0.99);
    EXPECT_GT(std::abs(matches[1].normal.x()), 0.99);
    EXPECT_LT((matches[0].point - parent.point_lidar).norm(), 1e-12);
    EXPECT_LT((matches[1].point - child.point_lidar).norm(), 1e-12);
    EXPECT_LT((matches[1].point_world - child.point_world).norm(), 1e-12);
    EXPECT_LT((matches[1].lidar_cov - child.cov_lidar).norm(), 1e-12);
    EXPECT_LT((matches[1].point_cov - child.cov_world).norm(), 1e-12);
    EXPECT_LT((unmatched[0] - far.point_world).norm(), 1e-12);

    // The parent consists of a symmetric 10x10 z=1 grid with isotropic
    // covariance .01. The upstream plane fit gives this center and covariance
    // analytically (normal variance .01 / (N * var(x)), center variance .01/N).
    EXPECT_LT((matches[0].center - V3D(0.8125, 0.8125, 1.0)).norm(), 1e-12);
    Eigen::Matrix<double, 6, 6> expected_cov = Eigen::Matrix<double, 6, 6>::Zero();
    const double normal_variance = 0.01 / (100.0 * 0.12890625);
    expected_cov.diagonal() << normal_variance, normal_variance, 0.0, 0.0001, 0.0001, 0.0001;
    EXPECT_LT((matches[0].plane_cov - expected_cov).norm(), 1e-10);
    pv_lio_plus::RVoxelMapMeasurement measurement;
    ASSERT_TRUE(pv_lio_plus::MakeRVoxelMapMeasurement(
        matches[0].point, matches[0].point_world, matches[0].normal, matches[0].center, matches[0].d,
        matches[0].plane_cov, matches[0].lidar_cov, Eigen::Quaterniond::Identity(),
        Eigen::Quaterniond::Identity(), V3D::Zero(), true, measurement));
    const double expected_variance = 2.0 * normal_variance * 0.0125 * 0.0125 + 0.0001 + 0.002;
    EXPECT_NEAR(measurement.information, 1.0 / expected_variance, 1e-8);
    EXPECT_NEAR(measurement.residual, -matches[0].normal.z() * 0.01, 1e-8);
}

TEST(RVoxelMapAdapter, MatchGateUsesWorldCovarianceAndRejectsNonFiniteInputs)
{
    MapManager map(Config());
    map.initialize(TwoPlanes());
    MapPoint low = Point(0.8, 0.8, 1.06);
    low.cov_world = M3D::Identity() * 1e-8;
    MapPoint high = low;
    high.cov_world = M3D::Identity() * 0.01;
    std::vector<V3D> unmatched;
    EXPECT_TRUE(Search(map, low, unmatched).empty());
    EXPECT_EQ(unmatched.size(), 1u);
    const PlaneMatchList accepted = Search(map, high, unmatched);
    ASSERT_EQ(accepted.size(), 1u);
    MapPoint higher = high;
    higher.cov_world = M3D::Identity() * 0.02;
    const PlaneMatchList accepted_higher = Search(map, higher, unmatched);
    ASSERT_EQ(accepted_higher.size(), 1u);
    pv_lio_plus::RVoxelMapMeasurement measurement, measurement_higher;
    const auto calculate = [](const pv_lio_plus::PlaneMatch &m, pv_lio_plus::RVoxelMapMeasurement &out) {
        return pv_lio_plus::MakeRVoxelMapMeasurement(
            m.point, m.point_world, m.normal, m.center, m.d, m.plane_cov, m.lidar_cov,
            Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity(), V3D::Zero(), true, out);
    };
    ASSERT_TRUE(calculate(accepted[0], measurement));
    ASSERT_TRUE(calculate(accepted_higher[0], measurement_higher));
    EXPECT_NEAR(measurement.information, measurement_higher.information, 1e-12);
    high.cov_world(0, 0) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(Search(map, high, unmatched).empty());
    EXPECT_EQ(unmatched.size(), 1u);
    const std::size_t before = map.point_count();
    map.update(MapPointList{high});
    EXPECT_EQ(map.point_count(), before);
}

TEST(RVoxelMapLifecycle, LruEvictionSnapshotClearAndWindowRejection)
{
    MapManagerConfig c = Config();
    c.r_lru_capacity = 2;
    MapManager map(c);
    map.initialize(MapPointList{Point(0.25, 1.5, 1.0)});
    map.update(MapPointList{Point(4.25, 1.5, 1.0)});
    map.update(MapPointList{Point(8.25, 1.5, 1.0)});
    ASSERT_EQ(map.size(), 2u);
    ASSERT_EQ(map.point_count(), 2u);
    PointCloudXYZI::Ptr snapshot = map.snapshot();
    for (const auto &p : *snapshot) EXPECT_GE(p.x, 4.0f);
    map.update(MapPointList{Point(4.5, 1.5, 1.0)}); // touch root 1
    map.update(MapPointList{Point(12.25, 1.5, 1.0)}); // evict root 2
    ASSERT_EQ(map.size(), 2u);
    ASSERT_EQ(map.point_count(), 3u);
    snapshot = map.snapshot();
    for (const auto &p : *snapshot) EXPECT_FALSE(p.x >= 8.0f && p.x < 12.0f);
    MapManager initially_over_capacity(c);
    initially_over_capacity.initialize(MapPointList{Point(0.25, 1.5, 1.0), Point(4.25, 1.5, 1.0),
                                                     Point(8.25, 1.5, 1.0)});
    EXPECT_EQ(initially_over_capacity.size(), 2u);
    EXPECT_EQ(initially_over_capacity.point_count(), 2u);
    EXPECT_EQ(initially_over_capacity.snapshot()->size(), 2u);
    pv_lio_plus::MapWindow window;
    EXPECT_THROW(map.erase_outside_window(window), std::logic_error);
    c.local_window_enabled = true;
    EXPECT_THROW(map.configure(c), std::invalid_argument);
    EXPECT_EQ(map.size(), 2u);
    map.clear();
    EXPECT_FALSE(map.initialized());
    EXPECT_EQ(map.size(), 0u);
    EXPECT_TRUE(map.snapshot()->empty());
    map.update(MapPointList{Point(-4.0, 1.5, 1.0)});
    EXPECT_TRUE(map.initialized());
    EXPECT_EQ(map.size(), 1u);
    EXPECT_EQ(map.point_count(), 1u);
}

TEST(RVoxelMapQueries, NegativeIntegerBinsAndNeighborDistancesUseWorldMetres)
{
    r_voxelmap_ns::VOXEL_LOC loc;
    ASSERT_TRUE(r_voxelmap_ns::voxel_location(V3D(-4.0, -0.0, 4.0), 4.0, loc));
    EXPECT_EQ(loc.x, -1);
    EXPECT_EQ(loc.y, 0);
    EXPECT_EQ(loc.z, 1);
    ASSERT_TRUE(r_voxelmap_ns::voxel_location(V3D(-4.00001, -0.00001, 0), 4.0, loc));
    EXPECT_EQ(loc.x, -2);
    EXPECT_EQ(loc.y, -1);
    ASSERT_TRUE(r_voxelmap_ns::voxel_location(V3D(-3.99999, 0, 0), 4.0, loc));
    EXPECT_EQ(loc.x, -1);

    MapPointList points;
    points.push_back(Point(11.5, 1.8, 1.2)); // occupied query root without a plane
    for (int i = 0; i < 10; ++i)
        for (int j = 0; j < 10; ++j)
            points.push_back(Point(12.1 + 0.1 * i, 1.3 + 0.1 * j, 1.0));
    MapManager map(Config());
    map.initialize(points);
    std::vector<V3D> unmatched;
    const auto matches = Search(map, Point(11.95, 1.75, 1.01), unmatched);
    ASSERT_EQ(matches.size(), 1u);
    EXPECT_GT(std::abs(matches.front().normal.z()), 0.99);
}

TEST(RVoxelMapConfiguration, InvalidValuesDoNotClearExistingMap)
{
    MapManagerConfig c = Config();
    MapManager map(c);
    map.initialize(TwoPlanes());
    const std::size_t before = map.point_count();
    for (const std::vector<int> &thresholds : {std::vector<int>{}, std::vector<int>{2, 2}, std::vector<int>{5, 1}})
    {
        c.r_reinit_size_vec = thresholds;
        EXPECT_THROW(map.configure(c), std::invalid_argument);
        EXPECT_EQ(map.point_count(), before);
    }
    c = Config();
    c.r_valid_check_resolution = 0;
    EXPECT_THROW(map.configure(c), std::invalid_argument);
    c = Config();
    c.r_ransac_inlier_distance_threshold = std::numeric_limits<double>::infinity();
    EXPECT_THROW(map.configure(c), std::invalid_argument);
    EXPECT_EQ(map.point_count(), before);
}

TEST(RVoxelMapConfiguration, SelectorAndOutputStemAreDistinct)
{
    EXPECT_EQ(pv_lio_plus::ParseMapType("r_voxelmap"), MapType::RVoxelMap);
    EXPECT_EQ(pv_lio_plus::ParseMapType("r-voxelmap"), MapType::RVoxelMap);
    EXPECT_EQ(pv_lio_plus::ParseMapType("r_voxel_map"), MapType::RVoxelMap);
    const std::string stem = result_file_stem(pv_lio_plus::MapTypeName(MapType::RVoxelMap));
    EXPECT_EQ(stem, "pv_lio_r_voxelmap");
    for (const MapType type : {MapType::VoxelMap, MapType::VoxelMapPlus, MapType::IKDTree,
                               MapType::IVox, MapType::C3PVoxelMap, MapType::OctVox})
        EXPECT_NE(stem, result_file_stem(pv_lio_plus::MapTypeName(type)));
}

TEST(RVoxelMapObservation, MatchesAuthorSensorAndPlaneCovarianceWeight)
{
    const Eigen::Vector3d pl(1.4, -2.0, 0.7), pw(1, 2, 3), normal(1, 0, 0), center(-1, 3, 2.5);
    Eigen::Matrix<double, 6, 6> plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
    plane_cov.diagonal() << 0.01, 0.02, 0.03, 0.04, 0.05, 0.06;
    Eigen::Matrix3d lidar_cov = Eigen::Matrix3d::Zero();
    lidar_cov.diagonal() << 0.1, 0.2, 0.3;
    const Eigen::Quaterniond rwi(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond ril(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));
    pv_lio_plus::RVoxelMapMeasurement measurement;
    ASSERT_TRUE(pv_lio_plus::MakeRVoxelMapMeasurement(pl, pw, normal, center, -0.8, plane_cov, lidar_cov,
                                                       rwi, ril, V3D(0.2, -0.3, 0.4), true, measurement));
    // Plane contribution is 4*.01 + 1*.02 + .25*.03 + 1*.04 = .1075.
    // R_IL^T R_WI^T n = (0,0,1), so the sensor contribution is .3.
    EXPECT_NEAR(measurement.information, 1.0 / 0.4075, 1e-12);
    EXPECT_NEAR(measurement.residual, -0.2, 1e-12);
    EXPECT_GT(measurement.jacobian.tail<6>().norm(), 0.1);
    ASSERT_TRUE(pv_lio_plus::MakeRVoxelMapMeasurement(pl, pw, normal, center, -0.8, plane_cov, lidar_cov,
                                                       rwi, ril, V3D(0.2, -0.3, 0.4), false, measurement));
    EXPECT_DOUBLE_EQ(measurement.jacobian.tail<6>().norm(), 0.0);
    lidar_cov(0, 0) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(pv_lio_plus::MakeRVoxelMapMeasurement(pl, pw, normal, center, -0.8, plane_cov, lidar_cov,
                                                        rwi, ril, V3D::Zero(), false, measurement));
}
}  // namespace

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
