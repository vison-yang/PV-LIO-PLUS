/* Copyright (C) 2026 PV-LIO-PLUS contributors. SPDX-License-Identifier: GPL-2.0-or-later */
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "map_manager/hvox_observation.h"
#include "map_manager/map_manager.cpp"

namespace {
using namespace pv_lio_plus;
MapPoint Point(double x, double y, double z) {
    MapPoint p;
    p.point_lidar = V3D(x, y, z);
    p.point_world = V3D(x, y, z);
    return p;
}
MapUpdateContext Context(double x = 0, double y = 0, double z = 0) {
    MapUpdateContext c;
    c.imu_position_world = V3D(x, y, z);
    return c;
}
MapManagerConfig Config() {
    MapManagerConfig c;
    c.type = MapType::HVox;
    c.hvox_voxel_size = 1.0;
    c.hvox_hierarchy_factor = 3;
    c.hvox_max_distance = 100.0;
    return c;
}
PlaneMatchList Search(MapManager &map, const MapPoint &p) {
    PlaneMatchList matches;
    std::vector<V3D> nonmatch;
    map.search(MapPointList{p}, matches, nonmatch);
    return matches;
}
bool HasCenter(const PointCloudXYZI::Ptr &snapshot, double x, double y, double z) {
    for (const auto &p : *snapshot)
        if (std::abs(p.x-x)<1e-5 && std::abs(p.y-y)<1e-5 && std::abs(p.z-z)<1e-5) return true;
    return false;
}

TEST(HVoxMap, EmptyMapAndNoMatchRemainEmpty) {
    MapManager map(Config());
    EXPECT_TRUE(Search(map, Point(.2,.2,.2)).empty());
    map.initialize({}, Context());
    EXPECT_TRUE(map.initialized());
    EXPECT_EQ(map.size(), 0u);
    EXPECT_TRUE(map.snapshot()->empty());
    map.update({}, 1, Context());
    EXPECT_TRUE(Search(map, Point(.2,.2,.2)).empty());
    EXPECT_EQ(map.size(), 0u);
}

TEST(HVoxMap, L0MeansL1EqualWeightAndCachedSurfel) {
    MapManager map(Config());
    EXPECT_EQ(ParseMapType("h-vox"), MapType::HVox);
    EXPECT_STREQ(MapTypeName(MapType::HVox), "hvox");
    map.initialize({Point(.2,.2,1.2), Point(.8,.2,1.2), Point(1.2,.2,1.2),
                    Point(2.2,.2,1.2), Point(.2,1.2,1.2)}, Context());
    EXPECT_EQ(map.size(), 4u);
    EXPECT_TRUE(Search(map, Point(1.1, .3, 1.21)).empty());  // native minimum is five children
    EXPECT_TRUE(HasCenter(map.snapshot(), .5, .2, 1.2));
    map.update({Point(1.2,1.2,1.2), Point(2.2,1.2,1.2)}, 1, Context());
    ASSERT_EQ(map.size(), 6u);
    ASSERT_EQ(map.point_count(), 7u);
    auto matches = Search(map, Point(1.1,.3,1.21));
    ASSERT_EQ(matches.size(), 1u);
    // Each L0 centroid has equal influence, irrespective of its point count.
    EXPECT_NEAR(matches[0].center.x(), 1.25, 1e-6);
    EXPECT_NEAR(matches[0].center.y(), .7, 1e-6);
    EXPECT_NEAR(matches[0].center.z(), 1.2, 1e-6);
    EXPECT_NEAR(std::abs(matches[0].normal.z()), 1., 1e-6);
    // Compare the signed normal directly with the pinned native consumer API.
    lio::VoxelMap author(1.0f);
    auto cloud = std::make_shared<lio::PointCloud>();
    for (const auto &p : {Point(.2,.2,1.2), Point(.8,.2,1.2), Point(1.2,.2,1.2),
                          Point(2.2,.2,1.2), Point(.2,1.2,1.2), Point(1.2,1.2,1.2),
                          Point(2.2,1.2,1.2)})
        cloud->push_back(lio::Point3D::from_eigen(p.point_world));
    author.UpdateVoxelMap(cloud, V3D::Zero(), 100.0, true);
    Eigen::Vector3f author_normal, author_center;
    float author_planarity;
    ASSERT_TRUE(author.GetSurfelAtPoint(lio::Point3D(1.1f,.3f,1.21f), author_normal, author_center,
                                        author_planarity));
    EXPECT_LT((matches[0].normal-author_normal.cast<double>()).norm(), 1e-7);
    const V3D native_normal = matches[0].normal;
    const V3D native_center = matches[0].center;
    map.update({Point(.9,.2,1.2)}, 2, Context());
    EXPECT_EQ(map.size(), 6u);
    EXPECT_TRUE(HasCenter(map.snapshot(), (0.2+0.8+0.9)/3., .2, 1.2));
    matches = Search(map, Point(1.1,.3,1.21));
    ASSERT_EQ(matches.size(), 1u);
    EXPECT_EQ((matches[0].center-native_center).norm(), 0.0);  // same child count: no refresh
    EXPECT_EQ((matches[0].normal-native_normal).norm(), 0.0);
}

TEST(HVoxMap, NegativeCoordinatesUseFloorParentAndContainingL1Only) {
    MapManager map(Config());
    MapPointList points;
    for (double x : {-2.8, -1.8, -.8})
        for (double y : {.2, 1.2}) points.push_back(Point(x,y,1.2));
    map.initialize(points, Context());
    EXPECT_EQ(map.size(), 6u);
    auto inside = Search(map, Point(-.01,.3,1.2));
    ASSERT_EQ(inside.size(), 1u);
    EXPECT_NEAR(inside[0].center.x(), -1.8, 1e-6);
    EXPECT_TRUE(Search(map, Point(.01,.3,1.2)).empty());
    EXPECT_TRUE(Search(map, Point(-3.01,.3,1.2)).empty());
    EXPECT_EQ(Search(map, Point(-3.0,.3,1.2)).size(), 1u);
}

TEST(HVoxMap, WindowStrictThresholdAndClearReconstructsCenter) {
    auto config = Config();
    config.hvox_max_distance = 2.0;
    config.hvox_box_multiplier = 2.0;
    MapManager map(config);
    // Box half width = 4; first center is the IMU position, not the cloud mean.
    map.initialize({Point(-3.5,.2,.2), Point(.5,.2,.2)}, Context());
    EXPECT_EQ(map.size(), 2u);
    map.update({Point(.5,.2,.2)}, 1, Context(1.0));
    EXPECT_EQ(map.size(), 2u);  // displacement == D / multiplier does not recenter
    map.update({Point(.5,.2,.2)}, 2, Context(1.001));
    EXPECT_EQ(map.size(), 1u);
    EXPECT_FALSE(HasCenter(map.snapshot(), -3.5,.2,.2));
    const auto old_revision = map.revision();
    map.clear();
    EXPECT_GT(map.revision(), old_revision);
    EXPECT_EQ(map.size(), 0u);
    map.initialize({Point(10.5,.2,.2), Point(14.5,.2,.2)}, Context(10.0));
    map.update({Point(10.5,.2,.2)}, 3, Context(11.0));
    EXPECT_EQ(map.size(), 2u);  // fresh center=10, equality again does not recenter
    map.update({Point(10.5,.2,.2)}, 4, Context(8.999));
    EXPECT_EQ(map.size(), 1u);  // center moved left, positive-side point is evicted
}

TEST(HVoxMap, ContextAndCoordinatesRejectedBeforeMutation) {
    MapManager map(Config());
    map.initialize({Point(.2,.2,.2)}, Context());
    const auto revision = map.revision();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(map.initialize({Point(10,0,0)}), std::invalid_argument);
    EXPECT_THROW(map.update({Point(10,0,0)}, 1, Context(nan)), std::invalid_argument);
    EXPECT_THROW(map.update({Point(nan,0,0)}, 1, Context()), std::invalid_argument);
    EXPECT_THROW(map.update({Point(1e100,0,0)}, 1, Context()), std::invalid_argument);
    EXPECT_THROW(map.update({Point(-2147483648.0,0,0)}, 1, Context()), std::invalid_argument);
    EXPECT_THROW(map.update({Point(2147483647.0,0,0)}, 1, Context()), std::invalid_argument);
    EXPECT_THROW(map.update({Point(.3,0,0), Point(nan,0,0)}, 1, Context()), std::invalid_argument);
    EXPECT_EQ(map.revision(), revision);
    EXPECT_EQ(map.size(), 1u);
    EXPECT_TRUE(HasCenter(map.snapshot(), .2,.2,.2));
    MapWindow window;
    EXPECT_THROW(map.erase_outside_window(window), std::logic_error);
    auto invalid = Config();
    invalid.local_window_enabled = true;
    EXPECT_THROW(map.configure(invalid), std::invalid_argument);
    invalid = Config(); invalid.hvox_hierarchy_factor = 2;
    EXPECT_THROW(map.configure(invalid), std::invalid_argument);
    EXPECT_EQ(map.revision(), revision);
    EXPECT_EQ(map.size(), 1u);
    EXPECT_THROW(Search(map, Point(nan,0,0)), std::invalid_argument);
    map.configure(Config());
    EXPECT_GT(map.revision(), revision);
    EXPECT_EQ(map.size(), 0u);
}

TEST(HVoxMap, RealLifecycleGenerationInvalidatesObservationGroup) {
    MapManager map(Config());
    map.initialize({Point(.2,.2,.2)}, Context());
    HVoxCorrespondence c;
    c.normal_world = V3D::UnitZ();
    HVoxObservationGroup group;
    auto refresh = [&]() {
        return group.Refresh({c}, Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity(),
                             V3D::Zero(), V3D::Zero(), map.revision());
    };
    ASSERT_TRUE(refresh());
    EXPECT_FALSE(group.NeedsRefresh(false, map.revision()));
    map.clear();
    EXPECT_TRUE(group.NeedsRefresh(false, map.revision()));
    ASSERT_TRUE(refresh());
    map.configure(Config());
    EXPECT_TRUE(group.NeedsRefresh(false, map.revision()));
    ASSERT_TRUE(refresh());
    map.initialize({Point(.2,.2,.2)}, Context());
    EXPECT_TRUE(group.NeedsRefresh(false, map.revision()));
    ASSERT_TRUE(refresh());
    map.update({Point(.3,.2,.2)}, 1, Context());
    EXPECT_TRUE(group.NeedsRefresh(false, map.revision()));
}
}  // namespace

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
