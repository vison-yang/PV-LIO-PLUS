/**
 * @file bievr_map_tests.cpp
 * @brief BIEVR 原生地图的契约测试。
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "map_manager/native/bievr/voxel_map.h"

namespace
{
using pv_lio_plus::bievr_native::Config;
using pv_lio_plus::bievr_native::Map;
using pv_lio_plus::bievr_native::Voxel;

/** @brief 建立便于合成几何测试的最小 BIEVR 配置。 */
Config TestConfig()
{
    Config config;
    config.voxelSize_      = 10.0;
    config.pixelSize_      = 0.1;
    config.capacity_       = 100;
    config.maxImagePixels_ = 100000;
    return config;
}

/** @brief 返回同一水平面内四角点，足以让样本协方差成熟。 */
std::vector<Eigen::Vector3d> HorizontalPlane()
{
    return {{4.0, 4.0, 5.0}, {6.0, 4.0, 5.0}, {4.0, 6.0, 5.0}, {6.0, 6.0, 5.0}};
}

/** @brief 为点集建立一一对应的正单位量程。 */
std::vector<double> UnitRanges(const std::size_t count)
{
    return std::vector<double>(count, 1.0);
}

/** @brief 返回有效像素数量。 */
std::size_t ValidPixelCount(const Voxel &voxel)
{
    return static_cast<std::size_t>((voxel.weights_.array() > 0.0f).count());
}
}  // namespace

/** @brief 提供 catkin gtest 目标所需的显式入口。 */
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(BievrNativeMap, SingleVoxelBatchIsGroupedAndUsesSampleNMinusOne)
{
    Map map(TestConfig());
    const auto points = HorizontalPlane();

    ASSERT_TRUE(map.Integrate(points, UnitRanges(points.size())));

    ASSERT_EQ(map.Size(), 1U);
    ASSERT_EQ(map.PointCount(), 4U);
    const Voxel *voxel = map.GetVoxel(points.front());
    ASSERT_NE(voxel, nullptr);
    ASSERT_TRUE(voxel->observed_);
    EXPECT_EQ(voxel->numPoints_, 4U);
    EXPECT_TRUE(voxel->pendingPoints_.empty());

    const Eigen::Vector3d mean       = voxel->pointSum_ / 4.0;
    const Eigen::Matrix3d covariance = (voxel->outerSum_ - voxel->pointSum_ * mean.transpose()) / (4.0 - 1.0);
    EXPECT_NEAR(covariance(0, 0), 4.0 / 3.0, 1e-12);
    EXPECT_NEAR(covariance(1, 1), 4.0 / 3.0, 1e-12);
    EXPECT_NEAR(covariance(2, 2), 0.0, 1e-12);
}

TEST(BievrNativeMap, PendingPointsAreProjectedOnceWhenVoxelMatures)
{
    Map map(TestConfig());
    const auto points = HorizontalPlane();

    ASSERT_TRUE(map.Integrate({points[0], points[1], points[2]}, {1.0, 1.0, 1.0}));
    const Voxel *pending = map.GetVoxel(points[0]);
    ASSERT_NE(pending, nullptr);
    EXPECT_FALSE(pending->observed_);
    EXPECT_EQ(pending->numPoints_, 3U);
    ASSERT_EQ(pending->pendingPoints_.size(), 3U);
    EXPECT_EQ(ValidPixelCount(*pending), 0U);

    ASSERT_TRUE(map.Integrate({points[3]}, {1.0}));
    const Voxel *mature = map.GetVoxel(points[0]);
    ASSERT_NE(mature, nullptr);
    EXPECT_TRUE(mature->observed_);
    EXPECT_EQ(mature->numPoints_, 4U);
    EXPECT_TRUE(mature->pendingPoints_.empty());
    EXPECT_EQ(ValidPixelCount(*mature), 4U);
}

TEST(BievrNativeMap, WeightedFusionAndDisabledSmoothingKeepRawAndSmoothedImages)
{
    Config config     = TestConfig();
    config.pixelSize_ = 3.0;
    config.weighted_  = true;
    config.smooth_    = false;
    Map map(config);
    const std::vector<Eigen::Vector3d> points = {
        {4.95, 4.95, 5.0}, {5.05, 4.95, 5.0}, {4.95, 5.05, 5.0}, {5.05, 5.05, 5.0}};
    const std::vector<double> ranges = {1.0, 2.0, 4.0, 5.0};

    ASSERT_TRUE(map.Integrate(points, ranges));
    const Voxel *voxel = map.GetVoxel(points.front());
    ASSERT_NE(voxel, nullptr);
    EXPECT_EQ(ValidPixelCount(*voxel), 1U);
    EXPECT_NEAR(static_cast<double>(voxel->weights_.maxCoeff()), 1.45, 1e-6);
    ASSERT_EQ(voxel->image_.rows(), voxel->smoothed_.rows());
    ASSERT_EQ(voxel->image_.cols(), voxel->smoothed_.cols());
    EXPECT_TRUE((voxel->image_.array() == voxel->smoothed_.array()).all());
}

TEST(BievrNativeMap, OptionalMaskedSmoothingChangesTheBumpNeighborhood)
{
    Config config     = TestConfig();
    config.pixelSize_ = 0.25;
    config.smooth_    = true;
    Map map(config);
    std::vector<Eigen::Vector3d> points;
    for (int y = 0; y < 5; ++y)
    {
        for (int x = 0; x < 5; ++x)
        {
            const double bump = (x == 2 && y == 2) ? 0.15 : 0.0;
            points.emplace_back(4.0 + 0.15 * x, 4.0 + 0.15 * y, 5.0 + bump);
        }
    }

    ASSERT_TRUE(map.Integrate(points, UnitRanges(points.size())));
    const Voxel *voxel = map.GetVoxel(points.front());
    ASSERT_NE(voxel, nullptr);
    ASSERT_TRUE(voxel->observed_);
    ASSERT_EQ(voxel->image_.rows(), voxel->smoothed_.rows());
    ASSERT_EQ(voxel->image_.cols(), voxel->smoothed_.cols());
    EXPECT_TRUE(voxel->image_.allFinite());
    EXPECT_TRUE(voxel->smoothed_.allFinite());
    bool changed = false;
    for (int y = 0; y < voxel->image_.rows(); ++y)
    {
        for (int x = 0; x < voxel->image_.cols(); ++x)
        {
            if (voxel->weights_(y, x) > 0.0f && std::abs(voxel->image_(y, x) - voxel->smoothed_(y, x)) > 1e-5f)
            {
                changed = true;
            }
        }
    }
    EXPECT_TRUE(changed);
}

TEST(BievrNativeMap, NormalChangeReprojectsImageIntoTheNewFrame)
{
    Config config  = TestConfig();
    config.smooth_ = true;
    Map map(config);
    const auto oldPlane = HorizontalPlane();
    ASSERT_TRUE(map.Integrate(oldPlane, UnitRanges(oldPlane.size())));
    const Voxel *oldVoxel = map.GetVoxel(oldPlane.front());
    ASSERT_NE(oldVoxel, nullptr);
    ASSERT_TRUE(oldVoxel->observed_);
    const Eigen::Vector3d oldNormal = oldVoxel->objectFromWorld_.linear().row(2).transpose();
    EXPECT_GT(std::abs(oldNormal.dot(Eigen::Vector3d::UnitZ())), 0.99);

    const std::vector<Eigen::Vector3d> newPlane = {{5.0, 1.0, 1.0}, {5.0, 9.0, 1.0}, {5.0, 1.0, 9.0}, {5.0, 9.0, 9.0}};
    ASSERT_TRUE(map.Integrate(newPlane, UnitRanges(newPlane.size())));
    const Voxel *newVoxel = map.GetVoxel(oldPlane.front());
    ASSERT_NE(newVoxel, nullptr);
    const Eigen::Vector3d newNormal = newVoxel->objectFromWorld_.linear().row(2).transpose();
    EXPECT_GT(std::abs(newNormal.dot(Eigen::Vector3d::UnitX())), 0.99);
    EXPECT_EQ(newVoxel->numPoints_, 8U);
    EXPECT_GE(ValidPixelCount(*newVoxel), 4U);
    EXPECT_TRUE(newVoxel->pendingPoints_.empty());
    bool foundReprojectedHeight = false;
    for (int y = 0; y < newVoxel->image_.rows(); ++y)
    {
        for (int x = 0; x < newVoxel->image_.cols(); ++x)
        {
            if (newVoxel->weights_(y, x) > 0.0f && std::abs(newVoxel->image_(y, x)) > 0.5f &&
                std::abs(newVoxel->smoothed_(y, x) - newVoxel->image_(y, x)) < 1e-5f)
            {
                foundReprojectedHeight = true;
            }
        }
    }
    EXPECT_TRUE(foundReprojectedHeight);
}

TEST(BievrNativeMap, SnapshotLiftsRawPixelsBackToWorldGeometry)
{
    Map map(TestConfig());
    const auto points = HorizontalPlane();
    ASSERT_TRUE(map.Integrate(points, UnitRanges(points.size())));

    const auto snapshot = map.Snapshot();
    ASSERT_EQ(snapshot.size(), 4U);
    for (const Eigen::Vector3d &sourcePoint : points)
    {
        double nearestDistance = std::numeric_limits<double>::infinity();
        for (const Eigen::Vector3d &snapshotPoint : snapshot)
        {
            nearestDistance = std::min(nearestDistance, (sourcePoint - snapshotPoint).norm());
        }
        EXPECT_LT(nearestDistance, 0.1);
    }
}

TEST(BievrNativeMap, QueryUsesLaterNeighborOnEqualCentroidDistance)
{
    Config config     = TestConfig();
    config.voxelSize_ = 1.0;
    config.pixelSize_ = 0.1;
    Map map(config);
    const std::vector<Eigen::Vector3d> points = {{1.2, 0.2, 0.5},  {1.8, 0.2, 0.5},  {1.2, 0.8, 0.5},
                                                 {1.8, 0.8, 0.5},  {-0.8, 0.2, 0.5}, {-0.2, 0.2, 0.5},
                                                 {-0.8, 0.8, 0.5}, {-0.2, 0.8, 0.5}};

    ASSERT_TRUE(map.Integrate(points, UnitRanges(points.size())));
    EXPECT_EQ(map.GetVoxel(Eigen::Vector3d(0.5, 0.5, 0.5)), nullptr);
    const Voxel *nearest          = map.FindVoxel(Eigen::Vector3d(0.5, 0.5, 0.5));
    const Voxel *negativeNeighbor = map.GetVoxel(Eigen::Vector3d(-0.5, 0.5, 0.5));
    ASSERT_NE(nearest, nullptr);
    ASSERT_NE(negativeNeighbor, nullptr);
    EXPECT_EQ(nearest, negativeNeighbor);
}

TEST(BievrNativeMap, NegativeAndBoundaryMortonCellsDoNotAlias)
{
    Config config     = TestConfig();
    config.voxelSize_ = 1.0;
    Map map(config);
    const double minimum                      = -1048576.0 + 0.25;
    const double maximum                      = 1048575.0 + 0.25;
    const std::vector<Eigen::Vector3d> points = {{-0.1, 0.1, 0.1}, {minimum, 0.1, 0.1}, {maximum, 0.1, 0.1}};

    ASSERT_TRUE(map.Integrate(points, UnitRanges(points.size())));
    EXPECT_NE(map.GetVoxel(points[0]), nullptr);
    EXPECT_NE(map.GetVoxel(points[1]), nullptr);
    EXPECT_NE(map.GetVoxel(points[2]), nullptr);
    EXPECT_EQ(map.Size(), 3U);

    const std::size_t oldSize = map.Size();
    EXPECT_THROW(map.Integrate({Eigen::Vector3d(1048576.0, 0.1, 0.1)}, {1.0}), std::invalid_argument);
    EXPECT_EQ(map.Size(), oldSize);
}

TEST(BievrNativeMap, InvalidBatchIsRejectedBeforeAnyMapMutation)
{
    Map map(TestConfig());
    ASSERT_TRUE(map.Integrate({Eigen::Vector3d(1.0, 1.0, 1.0)}, {1.0}));
    const std::size_t oldSize       = map.Size();
    const std::size_t oldPointCount = map.PointCount();
    const double nan                = std::numeric_limits<double>::quiet_NaN();

    EXPECT_THROW(map.Integrate({Eigen::Vector3d(20.0, 20.0, 20.0), Eigen::Vector3d(nan, 0.0, 0.0)}, {1.0, 1.0}),
                 std::invalid_argument);
    EXPECT_THROW(map.Integrate({Eigen::Vector3d(20.0, 20.0, 20.0)}, {}), std::invalid_argument);
    EXPECT_EQ(map.Size(), oldSize);
    EXPECT_EQ(map.PointCount(), oldPointCount);
    EXPECT_EQ(map.GetVoxel(Eigen::Vector3d(20.0, 20.0, 20.0)), nullptr);
}

TEST(BievrNativeMap, QueriesDoNotTouchLruBeforeCapacityEviction)
{
    Config config     = TestConfig();
    config.voxelSize_ = 1.0;
    config.capacity_  = 2;
    Map map(config);
    ASSERT_TRUE(
        map.Integrate({Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector3d(1.1, 0.1, 0.1), Eigen::Vector3d(2.1, 0.1, 0.1)},
                      {1.0, 1.0, 1.0}));

    EXPECT_EQ(map.Size(), 2U);
    EXPECT_EQ(map.GetVoxel(Eigen::Vector3d(0.1, 0.1, 0.1)), nullptr);
    EXPECT_NE(map.GetVoxel(Eigen::Vector3d(1.1, 0.1, 0.1)), nullptr);
    ASSERT_TRUE(map.Integrate({Eigen::Vector3d(3.1, 0.1, 0.1)}, {1.0}));

    EXPECT_EQ(map.GetVoxel(Eigen::Vector3d(1.1, 0.1, 0.1)), nullptr);
    EXPECT_NE(map.GetVoxel(Eigen::Vector3d(2.1, 0.1, 0.1)), nullptr);
    EXPECT_NE(map.GetVoxel(Eigen::Vector3d(3.1, 0.1, 0.1)), nullptr);
    EXPECT_EQ(map.PointCount(), 2U);
}

TEST(BievrNativeMap, ConfigureRejectsInvalidParametersWithoutClearingTheMap)
{
    Map map(TestConfig());
    ASSERT_TRUE(map.Integrate({Eigen::Vector3d(1.0, 1.0, 1.0)}, {1.0}));
    const std::size_t oldSize = map.Size();
    Config invalid            = TestConfig();
    invalid.pixelSize_        = 0.0;

    EXPECT_THROW(map.Configure(invalid), std::invalid_argument);
    EXPECT_EQ(map.Size(), oldSize);
    EXPECT_NE(map.GetVoxel(Eigen::Vector3d(1.0, 1.0, 1.0)), nullptr);
}
