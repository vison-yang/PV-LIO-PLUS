/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <gtest/gtest.h>
#include <algorithm>
#include <limits>
#include <tuple>
#include <unordered_set>
#include <vector>

#include "map_manager/native/r_voxelmap/r_voxelmap.hpp"

namespace
{
namespace rv = r_voxelmap_ns;

rv::voxel_map_config Config()
{
    rv::voxel_map_config c{};
    c.voxel_size = 4.0f;
    c.max_layer = 2;
    c.layer_point_size = {5, 5, 5};
    c.planer_threshold = 0.01f;
    c.max_points_size = 1000;
    c.max_cov_points_size = 1000;
    c.max_root_points_size = 10000;
    c.update_size_threshold = 1;
    c.update_inlier_distance_threshold = 0.02;
    c.reinit_size_vec = {10000};
    c.lru_capacity = 100;
    c.ransac_max_iter = 100;
    c.ransac_sample_num = 4;
    c.ransac_inlier_distance_threshold = 0.02;
    c.ransac_isplane_p_threshold = 0.65;
    c.sample_seed = 42;
    c.valid_check_max_layer = -1;
    c.valid_check_min_points_size = 10;
    c.valid_check_resolution = 5;
    return c;
}

rv::pointWithCov Point(double x, double y, double z)
{
    rv::pointWithCov p;
    p.point = Eigen::Vector3d(x, y, z);
    p.cov = Eigen::Matrix3d::Identity() * 0.01;
    return p;
}

std::vector<rv::pointWithCov> TwoPlanes()
{
    std::vector<rv::pointWithCov> points;
    for (int i = 0; i < 10; ++i)
        for (int j = 0; j < 10; ++j)
            points.push_back(Point(0.25 + 0.125 * i, 0.25 + 0.125 * j, 1.0));
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            points.push_back(Point(3.25, 0.25 + 0.125 * i, 2.25 + 0.125 * j));
    return points;
}

using Coordinates = std::vector<std::tuple<double, double, double>>;
Coordinates Positions(const std::vector<rv::pointWithCov> &points)
{
    Coordinates result;
    for (const auto &p : points) result.emplace_back(p.point.x(), p.point.y(), p.point.z());
    std::sort(result.begin(), result.end());
    return result;
}

TEST(RVoxelMapAsan, ParentRebuildAfterSameBatchChildUpdateKeepsAllPoints)
{
    rv::VoxelMap map;
    map.init(Config());
    auto original = TwoPlanes();
    map.build(original);
    ASSERT_EQ(map.feat_map.size(), 1u);
    rv::OctoTree *root = map.feat_map.begin()->second;
    ASSERT_TRUE(root->plane_ptr_->is_plane);
    ASSERT_EQ(root->octo_state_, 1);
    rv::OctoTree *child = root->leaves_[5];
    ASSERT_NE(child, nullptr);
    ASSERT_TRUE(child->plane_ptr_->is_plane);
    ASSERT_FALSE(root->plane_points_.empty());

    // Both planes are exactly planar in this fixture. A positive native
    // residual gate allows one new point to enter each plane's update cache.
    root->plane_ptr_->min_eigen_value = 0.01f;
    child->plane_ptr_->min_eigen_value = 0.01f;
    // Force the parent's covariance recomputation to fail after both nodes
    // have been cached, making it rebuild and delete the old child subtree.
    root->plane_points_.front().cov(0, 0) = std::numeric_limits<double>::quiet_NaN();
    const rv::pointWithCov parent_point = Point(0.8, 0.8, 1.001);
    const rv::pointWithCov child_point = Point(3.251, 0.6, 2.6);

    // Select input order whose original unordered cache would visit the parent
    // first. This makes the fixture exercise the former use-after-free rather
    // than depending on allocator addresses or hash bucket traversal order.
    std::unordered_set<rv::OctoTree *> order;
    order.insert(root);
    order.insert(child);
    std::vector<rv::pointWithCov> batch;
    if (*order.begin() == root) {
        batch = {parent_point, child_point};
    } else {
        order.clear();
        order.insert(child);
        order.insert(root);
        ASSERT_EQ(*order.begin(), root);
        batch = {child_point, parent_point};
    }
    original.insert(original.end(), batch.begin(), batch.end());
    map.update(batch);

    // Reinitialization rejects the corrupted parent plane and redistributes
    // its retained points. A false parent plane proves the rebuild ran.
    EXPECT_FALSE(root->plane_ptr_->is_plane);
    EXPECT_EQ(root->all_points_num_root_, static_cast<int>(original.size()));
    EXPECT_EQ(root->point_count(), original.size());
    std::vector<rv::pointWithCov> stored;
    root->append_points(stored);
    EXPECT_EQ(Positions(stored), Positions(original));

    std::vector<rv::ptpl> matches;
    std::vector<int> indices;
    std::vector<Eigen::Vector3d> unmatched;
    map.build_residual_omp({Point(3.251, 0.6, 2.6)}, 2, matches, indices, unmatched);
    EXPECT_EQ(matches.size(), 1u);
    const auto next_point = Point(3.249, 0.625, 2.625);
    map.update({next_point});
    EXPECT_EQ(root->all_points_num_root_, static_cast<int>(original.size() + 1));
    EXPECT_EQ(root->point_count(), original.size() + 1);
}
}  // namespace

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
