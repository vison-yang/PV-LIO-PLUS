/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/**
 * @file ig_lio_tests.cpp
 * @brief iG-LIO 原生统计、邻域和 facade 契约测试。
 */

#include <gtest/gtest.h>

#include <Eigen/SVD>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "map_manager/map_manager.cpp"
#include "map_manager/native/ig_lio/voxel_map.h"

namespace
{
namespace ig  = pv_lio_plus::ig_lio_native;
using CellKey = ig::CellKey;
using Vector3 = Eigen::Vector3d;

/** @brief 独立计算指定分辨率下的 floor 格键。 */
CellKey ModelGridKey(const Vector3 &point, const double inverseResolution)
{
    return {static_cast<std::int32_t>(std::floor(point.x() * inverseResolution)),
            static_cast<std::int32_t>(std::floor(point.y() * inverseResolution)),
            static_cast<std::int32_t>(std::floor(point.z() * inverseResolution))};
}

/** @brief 独立复现定义后的 uint32 legacy 顺序值，仅用于测试模型。 */
std::size_t ModelLegacyHashIndex(const CellKey &key, const std::size_t capacity)
{
    const std::uint32_t x     = static_cast<std::uint32_t>(key[0]);
    const std::uint32_t y     = static_cast<std::uint32_t>(key[1]);
    const std::uint32_t z     = static_cast<std::uint32_t>(key[2]);
    const std::uint32_t hash  = (x * 73856093U) ^ (y * 471943U) ^ (z * 83492791U);
    std::uint64_t signedValue = hash;
    if ((hash & 0x80000000U) != 0U) signedValue |= 0xffffffff00000000ULL;
    return static_cast<std::size_t>(signedValue) % capacity;
}

/** @brief 保存标量参考状态；不调用 native 分组、更新或淘汰函数。 */
struct ModelGrid
{
    std::size_t count_        = 0;
    Vector3 pointSum_         = Vector3::Zero();
    Eigen::Matrix3d outerSum_ = Eigen::Matrix3d::Zero();
    std::vector<Vector3> storedPoints_;
};

/**
 * @brief 独立标量状态机，用 map 和线性 LRU 列表核对每格 N/S/Q/存点。
 */
class ScalarModel
{
    /** @brief 保存参考模型中已量化点的格键和确定性顺序。 */
    struct Entry
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Vector3 point_ = Vector3::Zero();
        CellKey key_{};
        std::size_t hashIndex_  = 0;
        std::size_t inputIndex_ = 0;
    };

   public:
    explicit ScalarModel(const ig::Config &config) : config_(config)
    {
    }

    /** @brief 按输入批次更新参考统计和 LRU。 */
    void AddBatch(const std::vector<Vector3> &points)
    {
        std::vector<Entry, Eigen::aligned_allocator<Entry>> entries;
        entries.reserve(points.size());
        for (std::size_t index = 0; index < points.size(); ++index)
        {
            Entry entry;
            for (int axis = 0; axis < 3; ++axis)
            {
                entry.point_[axis] = static_cast<double>(static_cast<float>(points[index][axis]));
            }
            entry.key_        = ModelGridKey(entry.point_, 1.0 / config_.resolution_);
            entry.hashIndex_  = ModelLegacyHashIndex(entry.key_, config_.capacity_);
            entry.inputIndex_ = index;
            entries.push_back(entry);
        }
        std::stable_sort(entries.begin(), entries.end(), [](const Entry &left, const Entry &right) {
            if (left.hashIndex_ != right.hashIndex_) return left.hashIndex_ < right.hashIndex_;
            if (left.key_ != right.key_) return left.key_ < right.key_;
            return left.inputIndex_ < right.inputIndex_;
        });

        for (std::size_t first = 0; first < entries.size();)
        {
            std::size_t last = first + 1;
            while (last < entries.size() && entries[last].key_ == entries[first].key_) ++last;
            Apply(entries[first].key_, entries.data() + first, entries.data() + last);
            first = last;
        }
    }

    /** @brief 返回参考格数。 */
    std::size_t Size() const
    {
        return grids_.size();
    }

    /** @brief 返回参考存点总数。 */
    std::size_t PointCount() const
    {
        std::size_t count = 0;
        for (const auto &entry : grids_) count += entry.second.storedPoints_.size();
        return count;
    }

    /** @brief 依参考 LRU 顺序展开 XYZ。 */
    std::vector<Vector3> Snapshot() const
    {
        std::vector<Vector3> result;
        result.reserve(PointCount());
        for (const auto &key : lru_)
        {
            const auto grid = grids_.find(key);
            if (grid != grids_.end())
                result.insert(result.end(), grid->second.storedPoints_.begin(), grid->second.storedPoints_.end());
        }
        return result;
    }

    /** @brief 暴露参考格表和 LRU，仅供断言读取。 */
    const std::map<CellKey, ModelGrid> &Grids() const
    {
        return grids_;
    }

    /** @brief 暴露参考 LRU，仅供断言读取。 */
    const std::vector<CellKey> &Lru() const
    {
        return lru_;
    }

   private:
    /** @brief 更新一个参考格并独立执行 capacity 尾部淘汰规则。 */
    void Apply(const CellKey &key, const Entry *first, const Entry *last)
    {
        const std::size_t batchCount = static_cast<std::size_t>(last - first);
        Vector3 batchSum             = Vector3::Zero();
        Eigen::Matrix3d batchOuter   = Eigen::Matrix3d::Zero();
        for (const Entry *entry = first; entry != last; ++entry)
        {
            batchSum += entry->point_;
            batchOuter += entry->point_ * entry->point_.transpose();
        }

        auto grid = grids_.find(key);
        if (grid == grids_.end())
        {
            ModelGrid newGrid;
            newGrid.count_    = batchCount;
            newGrid.pointSum_ = batchSum;
            newGrid.outerSum_ = batchOuter;
            for (const Entry *entry = first; entry != last; ++entry) newGrid.storedPoints_.push_back(entry->point_);
            grids_.emplace(key, std::move(newGrid));
            Touch(key);
            if (grids_.size() >= config_.capacity_)
            {
                grids_.erase(lru_.back());
                lru_.pop_back();
            }
            return;
        }

        if (grid->second.count_ < 50)
        {
            grid->second.pointSum_ += batchSum;
            grid->second.outerSum_ += batchOuter;
            grid->second.count_ += batchCount;
            if (grid->second.count_ < config_.gridMaxPoints_)
            {
                for (const Entry *entry = first; entry != last; ++entry)
                    grid->second.storedPoints_.push_back(entry->point_);
            }
        }
        Touch(key);
    }

    /** @brief 将键移到线性 LRU 表首位。 */
    void Touch(const CellKey &key)
    {
        const auto position = std::find(lru_.begin(), lru_.end(), key);
        if (position != lru_.end()) lru_.erase(position);
        lru_.insert(lru_.begin(), key);
    }

    ig::Config config_;
    std::map<CellKey, ModelGrid> grids_;
    std::vector<CellKey> lru_;
};

/** @brief 对比 native 快照与独立标量状态机的键、矩和存点。 */
void ExpectSameAsModel(const ig::VoxelMap &map, const ScalarModel &model)
{
    EXPECT_EQ(map.Size(), model.Size());
    EXPECT_EQ(map.PointCount(), model.PointCount());
    const auto actualPoints   = map.Snapshot();
    const auto expectedPoints = model.Snapshot();
    ASSERT_EQ(actualPoints.size(), expectedPoints.size());
    for (std::size_t index = 0; index < actualPoints.size(); ++index)
    {
        EXPECT_EQ((actualPoints[index] - expectedPoints[index]).norm(), 0.0) << index;
    }

    const auto actualStats = map.GridStatsSnapshot();
    ASSERT_EQ(actualStats.size(), model.Lru().size());
    for (std::size_t index = 0; index < actualStats.size(); ++index)
    {
        const CellKey &key = model.Lru()[index];
        EXPECT_EQ(actualStats[index].cellKey_, key);
        const auto expected = model.Grids().find(key);
        ASSERT_NE(expected, model.Grids().end());
        EXPECT_EQ(actualStats[index].count_, expected->second.count_);
        EXPECT_TRUE((actualStats[index].pointSum_ - expected->second.pointSum_).isZero(0.0));
        EXPECT_TRUE((actualStats[index].outerSum_ - expected->second.outerSum_).isZero(0.0));
        EXPECT_EQ(actualStats[index].retainedPointCount_, expected->second.storedPoints_.size());
    }
}

/** @brief 在统计快照中按键查找单格。 */
ig::GridStats FindStats(const ig::VoxelMap &map, const CellKey &key)
{
    ig::GridStats stats;
    EXPECT_TRUE(map.GetGridStats(key, stats));
    return stats;
}

/** @brief 生成多个不共线且共面的输入点。 */
pv_lio_plus::MapPointList MakePlanePoints()
{
    pv_lio_plus::MapPointList points;
    const std::array<Vector3, 6> world{
        {{0.0, 0.0, 1.0}, {0.5, 0.0, 1.0}, {0.0, 0.5, 1.0}, {0.5, 0.5, 1.0}, {1.0, 0.0, 1.0}, {0.0, 1.0, 1.0}}};
    for (const auto &value : world)
    {
        pv_lio_plus::MapPoint point;
        point.point_world = value;
        point.point_lidar = value;
        point.cov_world   = Eigen::Matrix3d::Identity() * 0.01;
        point.cov_lidar   = Eigen::Matrix3d::Identity() * 0.01;
        points.push_back(point);
    }
    return points;
}

/** @brief 生成世界系和 LiDAR 系一致的有限查询点。 */
pv_lio_plus::MapPoint MakeQuery(const Vector3 &world)
{
    pv_lio_plus::MapPoint point;
    point.point_world = world;
    point.point_lidar = world;
    point.cov_world   = Eigen::Matrix3d::Identity() * 0.01;
    point.cov_lidar   = Eigen::Matrix3d::Identity() * 0.01;
    return point;
}

/** @brief 按搜索模式独立生成实际格偏移集合。 */
std::set<CellKey> ModelOffsets(const int searchMethod)
{
    std::set<CellKey> offsets;
    for (std::int32_t x = -1; x <= 1; ++x)
    {
        for (std::int32_t y = -1; y <= 1; ++y)
        {
            for (std::int32_t z = -1; z <= 1; ++z)
            {
                const int nonzero = (x != 0) + (y != 0) + (z != 0);
                if ((searchMethod == 1 && nonzero == 0) || (searchMethod == 7 && nonzero <= 1) ||
                    (searchMethod == 19 && nonzero <= 2) || searchMethod == 26)
                {
                    offsets.insert({x, y, z});
                }
            }
        }
    }
    return offsets;
}

/** @brief 独立暴力计算邻域、strict 半径和稳定 top-K 预期结果。 */
std::vector<ig::Neighbor> BruteForceNeighbors(const std::vector<Vector3> &points, const Vector3 &query,
                                              const double resolution, const int searchMethod, const std::size_t count,
                                              const double range)
{
    const double inverseResolution  = 1.0 / resolution;
    const CellKey baseKey           = ModelGridKey(query, inverseResolution);
    const std::set<CellKey> offsets = ModelOffsets(searchMethod);
    const double rangeSquared       = range * range;
    std::vector<ig::Neighbor> expected;
    std::map<CellKey, std::vector<Vector3>> pointsByGrid;
    for (const auto &point : points)
    {
        Vector3 storedPoint;
        for (int axis = 0; axis < 3; ++axis) storedPoint[axis] = static_cast<double>(static_cast<float>(point[axis]));
        pointsByGrid[ModelGridKey(storedPoint, inverseResolution)].push_back(storedPoint);
    }
    for (const auto &grid : pointsByGrid)
    {
        const CellKey offset{grid.first[0] - baseKey[0], grid.first[1] - baseKey[1], grid.first[2] - baseKey[2]};
        if (offsets.find(offset) == offsets.end()) continue;
        for (std::size_t index = 0; index < grid.second.size(); ++index)
        {
            const double distanceSquared = (query - grid.second[index]).squaredNorm();
            if (distanceSquared < rangeSquared)
            {
                ig::Neighbor neighbor;
                neighbor.point_           = grid.second[index];
                neighbor.distanceSquared_ = distanceSquared;
                neighbor.cellKey_         = grid.first;
                neighbor.pointIndex_      = index;
                expected.push_back(neighbor);
            }
        }
    }
    std::stable_sort(expected.begin(), expected.end(), [](const ig::Neighbor &left, const ig::Neighbor &right) {
        if (left.distanceSquared_ != right.distanceSquared_) return left.distanceSquared_ < right.distanceSquared_;
        for (int axis = 0; axis < 3; ++axis)
            if (left.point_[axis] != right.point_[axis]) return left.point_[axis] < right.point_[axis];
        if (left.cellKey_ != right.cellKey_) return left.cellKey_ < right.cellKey_;
        return left.pointIndex_ < right.pointIndex_;
    });
    if (expected.size() > count) expected.resize(count);
    return expected;
}

/** @brief 对比 native KNN 与输入 XYZ 上独立计算的暴力结果。 */
void ExpectSameNeighbors(const ig::VoxelMap &map, const std::vector<Vector3> &sourcePoints, const Vector3 &query,
                         const ig::Config &config, const std::size_t count, const double range)
{
    ig::NeighborList actual;
    const bool isFound = map.KNNByCondition(query, count, range, actual);
    const auto expected =
        BruteForceNeighbors(sourcePoints, query, config.resolution_, config.searchMethod_, count, range);
    EXPECT_EQ(isFound, !expected.empty());
    ASSERT_EQ(actual.size(), expected.size());
    for (std::size_t index = 0; index < actual.size(); ++index)
    {
        EXPECT_EQ((actual[index].point_ - expected[index].point_).norm(), 0.0) << index;
        EXPECT_DOUBLE_EQ(actual[index].distanceSquared_, expected[index].distanceSquared_) << index;
        EXPECT_EQ(actual[index].cellKey_, expected[index].cellKey_) << index;
        EXPECT_EQ(actual[index].pointIndex_, expected[index].pointIndex_) << index;
    }
}

/** @brief 配置一张单格 6 点地图用于 manager 搜索契约。 */
pv_lio_plus::MapManagerConfig MakeManagerConfig()
{
    pv_lio_plus::MapManagerConfig config;
    config.type                = pv_lio_plus::MapType::IGLIOPlane;
    config.nearest_point_count = 5;
    config.nearest_max_range   = 5.0;
    config.plane_fit_threshold = 0.1;
    config.igResolution_       = 10.0;
    config.igSearchMethod_     = 26;
    config.igCapacity_         = 32;
    config.igGridMaxPoints_    = 20;
    return config;
}

/** @brief 校验单个 native 快照点 XYZ。 */
void ExpectPoint(const Vector3 &actual, const Vector3 &expected)
{
    EXPECT_EQ(actual.x(), expected.x());
    EXPECT_EQ(actual.y(), expected.y());
    EXPECT_EQ(actual.z(), expected.z());
}

/** @brief 在五点后端拟合前为 manager helper 建立足量几何样本。 */
TEST(IGLioNative, StatisticsMatchIndependentSixPointSvdOracle)
{
    ig::Config config;
    config.resolution_    = 10.0;
    config.capacity_      = 8;
    config.gridMaxPoints_ = 32;
    ig::VoxelMap map(config);

    const std::vector<Vector3> points{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0},
                                      {0.0, 0.0, 1.0}, {1.0, 1.0, 0.0}, {1.0, 0.0, 1.0}};
    ASSERT_TRUE(map.AddCloud(std::vector<Vector3>(points.begin(), points.begin() + 5)));
    ig::GridStats stats = FindStats(map, {0, 0, 0});
    EXPECT_EQ(stats.count_, 5u);
    EXPECT_FALSE(stats.isValid_);

    ASSERT_TRUE(map.AddCloud(std::vector<Vector3>(points.begin() + 5, points.end())));
    stats = FindStats(map, {0, 0, 0});
    EXPECT_EQ(stats.count_, 6u);
    EXPECT_TRUE(stats.isValid_);

    Vector3 expectedSum           = Vector3::Zero();
    Eigen::Matrix3d expectedOuter = Eigen::Matrix3d::Zero();
    for (const auto &input : points)
    {
        Vector3 quantized;
        for (int axis = 0; axis < 3; ++axis) quantized[axis] = static_cast<double>(static_cast<float>(input[axis]));
        expectedSum += quantized;
        expectedOuter += quantized * quantized.transpose();
    }
    const Vector3 expectedMean           = expectedSum / 6.0;
    const Eigen::Matrix3d expectedSample = (expectedOuter - expectedSum * expectedMean.transpose()) / 5.0;
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(expectedSample, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Vector3d replacedSpectrum(1.0, 1.0, 1e-3);
    const Eigen::Matrix3d expectedShape = svd.matrixU() * replacedSpectrum.asDiagonal() * svd.matrixV().transpose();

    EXPECT_TRUE((stats.pointSum_ - expectedSum).isZero(0.0));
    EXPECT_TRUE((stats.outerSum_ - expectedOuter).isZero(0.0));
    EXPECT_TRUE((stats.centroid_ - expectedMean).isZero(0.0));
    EXPECT_NEAR((stats.sampleCovariance_ - expectedSample).norm(), 0.0, 1e-14);
    EXPECT_NEAR((stats.regularizedShape_ - expectedShape).norm(), 0.0, 1e-14);
    EXPECT_NEAR((stats.inverseRegularizedShape_ - expectedShape.inverse()).norm(), 0.0, 1e-12);
}

/** @brief 用轴对齐六点平面核对解析样本矩、SVD 改形和逆矩阵。 */
TEST(IGLioNative, AxisAlignedSixPointMomentsMatchAnalyticOracle)
{
    ig::Config config;
    config.resolution_    = 10.0;
    config.capacity_      = 8;
    config.gridMaxPoints_ = 20;
    ig::VoxelMap map(config);
    const std::vector<Vector3> points{{3.0, 2.0, 1.0}, {2.0, 3.0, 1.0}, {1.0, 2.0, 1.0},
                                      {2.0, 1.0, 1.0}, {3.0, 3.0, 1.0}, {1.0, 1.0, 1.0}};
    ASSERT_TRUE(map.AddCloud(points));

    const ig::GridStats stats = FindStats(map, {0, 0, 0});
    Eigen::Matrix3d expectedOuter;
    expectedOuter << 28.0, 26.0, 12.0, 26.0, 28.0, 12.0, 12.0, 12.0, 6.0;
    Eigen::Matrix3d expectedSample;
    expectedSample << 0.8, 0.4, 0.0, 0.4, 0.8, 0.0, 0.0, 0.0, 0.0;
    Eigen::Matrix3d expectedShape   = Eigen::Matrix3d::Identity();
    expectedShape(2, 2)             = 1e-3;
    Eigen::Matrix3d expectedInverse = Eigen::Matrix3d::Identity();
    expectedInverse(2, 2)           = 1000.0;

    EXPECT_EQ(stats.count_, 6u);
    EXPECT_TRUE((stats.pointSum_ - Vector3(12.0, 12.0, 6.0)).isZero(0.0));
    EXPECT_TRUE((stats.centroid_ - Vector3(2.0, 2.0, 1.0)).isZero(0.0));
    EXPECT_TRUE((stats.outerSum_ - expectedOuter).isZero(0.0));
    EXPECT_TRUE((stats.sampleCovariance_ - expectedSample).isZero(1e-14));
    EXPECT_TRUE((stats.regularizedShape_ - expectedShape).isZero(1e-12));
    EXPECT_TRUE((stats.inverseRegularizedShape_ - expectedInverse).isZero(1e-9));
}

/** @brief 比较多批更新后的 N/S/Q、快照和 LRU 标量模型。 */
TEST(IGLioNative, BatchStateMatchesIndependentScalarModel)
{
    ig::Config config;
    config.resolution_    = 1.0;
    config.searchMethod_  = 19;
    config.capacity_      = 5;
    config.gridMaxPoints_ = 9;
    ig::VoxelMap map(config);
    ScalarModel model(config);

    const std::vector<std::vector<Vector3>> batches{
        {{0.1, 0.1, 0.1}, {1.1, 0.1, 0.1}, {0.2, 0.1, 0.1}, {2.1, 0.1, 0.1}},
        {{-0.1, 0.1, 0.1}, {3.1, 0.1, 0.1}, {1.2, 0.2, 0.1}, {-1.2, 0.3, 0.1}},
        {{4.1, 0.1, 0.1}, {0.3, 0.3, 0.3}, {2.2, 0.2, 0.1}, {5.1, 0.1, 0.1}},
        {{1.3, 0.3, 0.1}, {-0.4, 0.2, 0.2}, {3.2, 0.2, 0.2}},
        {{6.1, 0.1, 0.1}, {2.3, 0.2, 0.2}, {4.2, 0.2, 0.2}, {-2.1, 0.2, 0.1}},
    };
    for (const auto &batch : batches)
    {
        EXPECT_TRUE(map.AddCloud(batch));
        model.AddBatch(batch);
        ExpectSameAsModel(map, model);
    }
}

/** @brief 覆盖旧 N=49 整批越过冻结线及随后不再更新的规则。 */
TEST(IGLioNative, StatisticsCanCrossFiftyThenFreeze)
{
    ig::Config config;
    config.resolution_    = 100.0;
    config.capacity_      = 8;
    config.gridMaxPoints_ = 100;
    ig::VoxelMap map(config);

    std::vector<Vector3> firstBatch;
    for (int index = 0; index < 49; ++index) firstBatch.emplace_back(0.1 * index, 0.1, 0.2);
    ASSERT_TRUE(map.AddCloud(firstBatch));
    ig::GridStats stats = FindStats(map, {0, 0, 0});
    ASSERT_EQ(stats.count_, 49u);
    ASSERT_EQ(stats.retainedPointCount_, 49u);

    const std::vector<Vector3> crossingBatch{{5.0, 1.0, 2.0}, {5.1, 1.0, 2.0}, {5.2, 1.0, 2.0}, {5.3, 1.0, 2.0}};
    ASSERT_TRUE(map.AddCloud(crossingBatch));
    stats = FindStats(map, {0, 0, 0});
    EXPECT_EQ(stats.count_, 53u);
    EXPECT_EQ(stats.retainedPointCount_, 53u);

    ASSERT_TRUE(map.AddCloud({{6.0, 2.0, 2.0}}));
    stats = FindStats(map, {0, 0, 0});
    EXPECT_EQ(stats.count_, 53u);
    EXPECT_EQ(stats.retainedPointCount_, 53u);
    EXPECT_EQ(map.PointCount(), 53u);
}

/** @brief 新格存完首批超门限点，既有格只在新 N 严格小于门限时追加。 */
TEST(IGLioNative, FirstBatchIsNotHardCappedAndNineteenToTwentyDoesNotAppend)
{
    ig::Config config;
    config.resolution_    = 10.0;
    config.capacity_      = 8;
    config.gridMaxPoints_ = 20;
    ig::VoxelMap map(config);

    std::vector<Vector3> firstBatch;
    for (int index = 0; index < 25; ++index) firstBatch.emplace_back(0.1 * index, 0.1, 0.1);
    ASSERT_TRUE(map.AddCloud(firstBatch));
    ig::GridStats stats = FindStats(map, {0, 0, 0});
    EXPECT_EQ(stats.count_, 25u);
    EXPECT_EQ(stats.retainedPointCount_, 25u);

    ig::VoxelMap thresholdMap(config);
    std::vector<Vector3> nineteen;
    for (int index = 0; index < 19; ++index) nineteen.emplace_back(0.1 * index, 1.0, 0.1);
    ASSERT_TRUE(thresholdMap.AddCloud(nineteen));
    ASSERT_TRUE(thresholdMap.AddCloud({{2.0, 1.0, 0.1}}));
    stats = FindStats(thresholdMap, {0, 0, 0});
    EXPECT_EQ(stats.count_, 20u);
    EXPECT_EQ(stats.retainedPointCount_, 19u);
    EXPECT_EQ(thresholdMap.PointCount(), 19u);
}

/** @brief 确认查询不触碰 LRU、update 触碰且 cap=2/3 保留 capacity−1 格。 */
TEST(IGLioNative, QueryDoesNotTouchUpdateTouchesAndCapacityEvictsTail)
{
    ig::Config config;
    config.resolution_    = 1.0;
    config.searchMethod_  = 1;
    config.capacity_      = 3;
    config.gridMaxPoints_ = 100;
    ig::VoxelMap queryMap(config);
    ASSERT_TRUE(queryMap.AddCloud({{0.2, 0.2, 0.2}}));
    ASSERT_TRUE(queryMap.AddCloud({{2.2, 0.2, 0.2}}));
    ASSERT_EQ(queryMap.GridStatsSnapshot().size(), 2u);
    EXPECT_EQ(queryMap.GridStatsSnapshot()[0].cellKey_, (CellKey{2, 0, 0}));
    ig::NeighborList found;
    ASSERT_TRUE(queryMap.KNNByCondition({0.2, 0.2, 0.2}, 1, 0.5, found));
    EXPECT_EQ(queryMap.GridStatsSnapshot()[0].cellKey_, (CellKey{2, 0, 0}));
    ASSERT_TRUE(queryMap.AddCloud({{4.2, 0.2, 0.2}}));
    ig::GridStats queryStats;
    EXPECT_FALSE(queryMap.GetGridStats({0, 0, 0}, queryStats));
    EXPECT_TRUE(queryMap.GetGridStats({2, 0, 0}, queryStats));
    EXPECT_EQ(queryMap.Size(), 2u);

    ig::VoxelMap updateMap(config);
    ASSERT_TRUE(updateMap.AddCloud({{0.2, 0.2, 0.2}}));
    ASSERT_TRUE(updateMap.AddCloud({{2.2, 0.2, 0.2}}));
    ASSERT_TRUE(updateMap.AddCloud({{0.3, 0.2, 0.2}}));
    EXPECT_EQ(updateMap.GridStatsSnapshot()[0].cellKey_, (CellKey{0, 0, 0}));
    ASSERT_TRUE(updateMap.AddCloud({{4.2, 0.2, 0.2}}));
    ig::GridStats stats;
    EXPECT_TRUE(updateMap.GetGridStats({0, 0, 0}, stats));
    EXPECT_FALSE(updateMap.GetGridStats({2, 0, 0}, stats));

    config.capacity_ = 2;
    ig::VoxelMap oneGridMap(config);
    ASSERT_TRUE(oneGridMap.AddCloud({{0.2, 0.2, 0.2}}));
    ASSERT_TRUE(oneGridMap.AddCloud({{2.2, 0.2, 0.2}}));
    EXPECT_EQ(oneGridMap.Size(), 1u);
    EXPECT_EQ(oneGridMap.PointCount(), 1u);
}

/** @brief 找到 legacy reduced-hash 冲突键，确认完整三元组仍分格保存。 */
TEST(IGLioNative, HashCollisionsKeepDistinctFullKeysAndNegativeFloorKeys)
{
    ig::Config config;
    config.resolution_    = 1.0;
    config.searchMethod_  = 1;
    config.capacity_      = 101;
    config.gridMaxPoints_ = 100;

    std::unordered_map<std::size_t, CellKey> firstKeyForHash;
    CellKey firstKey{};
    CellKey secondKey{};
    bool isCollisionFound = false;
    for (std::int32_t x = 0; x < 1000 && !isCollisionFound; ++x)
    {
        const CellKey key{x, 0, 0};
        const std::size_t hash = ModelLegacyHashIndex(key, config.capacity_);
        const auto iter        = firstKeyForHash.find(hash);
        if (iter != firstKeyForHash.end() && iter->second != key)
        {
            firstKey         = iter->second;
            secondKey        = key;
            isCollisionFound = true;
        }
        else
        {
            firstKeyForHash.emplace(hash, key);
        }
    }
    ASSERT_TRUE(isCollisionFound);

    ig::VoxelMap map(config);
    ASSERT_TRUE(map.AddCloud(
        {{double(firstKey[0]) + 0.25, 0.25, 0.25}, {double(secondKey[0]) + 0.25, 0.25, 0.25}, {-0.1, 0.1, 0.1}}));
    EXPECT_EQ(map.Size(), 3u);
    ig::GridStats stats;
    EXPECT_TRUE(map.GetGridStats(firstKey, stats));
    EXPECT_TRUE(map.GetGridStats(secondKey, stats));
    EXPECT_TRUE(map.GetGridStats({-1, 0, 0}, stats));
}

/** @brief 非有限、float 越界或 int32 越界批次必须在任何格修改前拒绝。 */
TEST(IGLioNative, InvalidBatchesAreRejectedAtomically)
{
    ig::Config config;
    config.resolution_    = 1.0;
    config.capacity_      = 8;
    config.gridMaxPoints_ = 20;
    ig::VoxelMap map(config);
    ASSERT_TRUE(map.AddCloud({{0.1, 0.1, 0.1}, {0.2, 0.1, 0.1}}));
    const auto beforePoints = map.Snapshot();
    const auto beforeStats  = map.GridStatsSnapshot();

    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(map.AddCloud({{5.1, 0.1, 0.1}, {nan, 0.0, 0.0}}), std::invalid_argument);
    EXPECT_THROW(map.AddCloud({{5.1, 0.1, 0.1}, {std::numeric_limits<double>::max(), 0.0, 0.0}}),
                 std::invalid_argument);
    EXPECT_THROW(map.AddCloud({{5.1, 0.1, 0.1}, {2147483650.0, 0.0, 0.0}}), std::invalid_argument);

    const auto afterPoints = map.Snapshot();
    const auto afterStats  = map.GridStatsSnapshot();
    ASSERT_EQ(afterPoints.size(), beforePoints.size());
    ASSERT_EQ(afterStats.size(), beforeStats.size());
    EXPECT_EQ((afterPoints[0] - beforePoints[0]).norm(), 0.0);
    EXPECT_EQ((afterPoints[1] - beforePoints[1]).norm(), 0.0);
    EXPECT_EQ(afterStats[0].cellKey_, beforeStats[0].cellKey_);
    EXPECT_EQ(afterStats[0].count_, beforeStats[0].count_);
    EXPECT_TRUE((afterStats[0].pointSum_ - beforeStats[0].pointSum_).isZero(0.0));
}

/** @brief 与实际 1/7/19/26 邻域暴力 top-K 对照并检查 strict 半径及 ties。 */
TEST(IGLioNative, SearchMatchesBruteForceNeighborhoodStrictRangeAndTies)
{
    const Vector3 query{0.25, 0.25, 0.25};
    std::vector<Vector3> sourcePoints;
    for (int x = -1; x <= 1; ++x)
    {
        for (int y = -1; y <= 1; ++y)
        {
            for (int z = -1; z <= 1; ++z)
            {
                sourcePoints.emplace_back(x + 0.1, y + 0.1, z + 0.1);
            }
        }
    }
    sourcePoints.push_back({0.1, 0.1, 0.1});
    sourcePoints.push_back({0.1, 0.1, 0.1});

    for (const int searchMethod : {1, 7, 19, 26})
    {
        ig::Config config;
        config.resolution_    = 1.0;
        config.searchMethod_  = searchMethod;
        config.capacity_      = 100;
        config.gridMaxPoints_ = 100;
        ig::VoxelMap map(config);
        ASSERT_TRUE(map.AddCloud(sourcePoints));
        ExpectSameNeighbors(map, sourcePoints, query, config, 50, 2.5);
        const std::size_t expectedNeighborhoodCount = ModelOffsets(searchMethod).size();
        ig::NeighborList allFound;
        ASSERT_TRUE(map.KNNByCondition(query, 100, 2.5, allFound));
        EXPECT_EQ(allFound.size(), expectedNeighborhoodCount + 2u);
    }

    ig::Config config;
    config.resolution_    = 1.0;
    config.searchMethod_  = 7;
    config.capacity_      = 8;
    config.gridMaxPoints_ = 20;
    ig::VoxelMap strictMap(config);
    ASSERT_TRUE(strictMap.AddCloud({{1.25, 0.25, 0.25}}));
    ig::NeighborList found;
    EXPECT_FALSE(strictMap.KNNByCondition(query, 5, 1.0, found));
    EXPECT_TRUE(found.empty());

    ig::VoxelMap tieMap(config);
    const std::vector<Vector3> tiePoints{
        {1.25, 0.25, 0.25}, {-0.75, 0.25, 0.25}, {0.25, 1.25, 0.25}, {-0.75, 0.25, 0.25}};
    ASSERT_TRUE(tieMap.AddCloud(tiePoints));
    ASSERT_TRUE(tieMap.KNNByCondition(query, 4, 1.01, found));
    ASSERT_EQ(found.size(), 4u);
    ExpectPoint(found[0].point_, {-0.75, 0.25, 0.25});
    ExpectPoint(found[1].point_, {-0.75, 0.25, 0.25});
    ExpectPoint(found[2].point_, {0.25, 1.25, 0.25});
    ExpectPoint(found[3].point_, {1.25, 0.25, 0.25});
}

/** @brief 确认插入和查询都沿用 point×倒数分辨率的浮点边界。 */
TEST(IGLioNative, QueryUsesTheSameReciprocalMultiplicationAsInsertion)
{
    ig::Config config;
    config.resolution_    = 0.1;
    config.searchMethod_  = 1;
    config.capacity_      = 8;
    config.gridMaxPoints_ = 20;
    ig::VoxelMap map(config);
    ASSERT_TRUE(map.AddCloud({{0.31, 0.0, 0.0}}));

    ig::NeighborList found;
    EXPECT_TRUE(map.KNNByCondition({0.3, 0.0, 0.0}, 1, 0.02, found));
    ASSERT_EQ(found.size(), 1u);
    EXPECT_EQ(found.front().cellKey_, (CellKey{3, 0, 0}));
}

/** @brief 检查 selector、生命周期、快照计数以及失败配置/初始化保图。 */
TEST(IGLioManager, SelectorLifecycleSnapshotAndRejectionPreserveOldMap)
{
    using pv_lio_plus::MapManager;
    using pv_lio_plus::MapManagerConfig;
    using pv_lio_plus::MapType;

    EXPECT_EQ(pv_lio_plus::ParseMapType("ig_lio_plane"), MapType::IGLIOPlane);
    EXPECT_EQ(pv_lio_plus::ParseMapType("ig-lio-plane"), MapType::IGLIOPlane);
    EXPECT_STREQ(pv_lio_plus::MapTypeName(MapType::IGLIOPlane), "ig_lio_plane");

    MapManagerConfig config = MakeManagerConfig();
    MapManager manager(config);
    const auto planePoints = MakePlanePoints();
    manager.initialize(planePoints);
    ASSERT_TRUE(manager.initialized());
    EXPECT_EQ(manager.type(), MapType::IGLIOPlane);
    EXPECT_EQ(manager.size(), 1u);
    EXPECT_EQ(manager.point_count(), planePoints.size());
    EXPECT_EQ(manager.snapshot()->size(), planePoints.size());

    const auto query = MakeQuery({0.3, 0.3, 1.01});
    pv_lio_plus::PlaneMatchList matches;
    std::vector<Vector3> nonMatch;
    manager.search(pv_lio_plus::MapPointList{query}, matches, nonMatch);
    ASSERT_EQ(matches.size(), 1u);
    EXPECT_EQ(matches.front().map_type, MapType::IGLIOPlane);
    EXPECT_NEAR(std::abs(matches.front().normal.z()), 1.0, 1e-4);
    EXPECT_TRUE(matches.front().plane_cov.isZero(0.0));
    EXPECT_TRUE((matches.front().point_cov - query.cov_world).isZero(0.0));

    auto invalidLidar                 = MakeQuery({0.3, 0.3, 1.01});
    invalidLidar.point_lidar.x()      = std::numeric_limits<double>::quiet_NaN();
    auto invalidWorld                 = MakeQuery({0.3, 0.3, 1.01});
    invalidWorld.point_world.y()      = std::numeric_limits<double>::quiet_NaN();
    auto invalidCovariance            = MakeQuery({0.3, 0.3, 1.01});
    invalidCovariance.cov_world(0, 0) = std::numeric_limits<double>::infinity();
    matches.clear();
    nonMatch.clear();
    manager.search(pv_lio_plus::MapPointList{invalidLidar, invalidWorld, invalidCovariance}, matches, nonMatch);
    EXPECT_TRUE(matches.empty());
    EXPECT_EQ(nonMatch.size(), 3u);

    MapManager degenerateManager(MakeManagerConfig());
    pv_lio_plus::MapPointList degeneratePoints(5, MakeQuery(Vector3::Zero()));
    degenerateManager.initialize(degeneratePoints);
    matches.clear();
    nonMatch.clear();
    degenerateManager.search(pv_lio_plus::MapPointList{MakeQuery(Vector3::Zero())}, matches, nonMatch);
    EXPECT_TRUE(matches.empty());
    EXPECT_EQ(nonMatch.size(), 1u);

    const std::uint64_t beforeRevision = manager.revision();
    manager.update(pv_lio_plus::MapPointList{MakeQuery({1.2, 1.2, 1.0})});
    EXPECT_EQ(manager.revision(), beforeRevision + 1);
    EXPECT_EQ(manager.point_count(), planePoints.size() + 1);

    const auto beforeSnapshot       = manager.snapshot();
    const auto beforeCount          = manager.point_count();
    const auto beforeFailedRevision = manager.revision();
    pv_lio_plus::MapPointList badInitial{MakeQuery({2.0, 2.0, 2.0}),
                                         MakeQuery({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0})};
    EXPECT_THROW(manager.initialize(badInitial), std::invalid_argument);
    EXPECT_EQ(manager.revision(), beforeFailedRevision);
    EXPECT_EQ(manager.point_count(), beforeCount);
    EXPECT_EQ(manager.snapshot()->size(), beforeSnapshot->size());

    MapManagerConfig badConfig = config;
    badConfig.igResolution_    = 0.0;
    EXPECT_THROW(manager.configure(badConfig), std::invalid_argument);
    EXPECT_EQ(manager.type(), MapType::IGLIOPlane);
    EXPECT_EQ(manager.snapshot()->size(), beforeSnapshot->size());

    badConfig                      = config;
    badConfig.local_window_enabled = true;
    EXPECT_THROW(manager.configure(badConfig), std::invalid_argument);
    EXPECT_EQ(manager.snapshot()->size(), beforeSnapshot->size());

    for (const int nearestCount : {0, -1})
    {
        badConfig                     = config;
        badConfig.nearest_point_count = nearestCount;
        EXPECT_THROW(manager.configure(badConfig), std::invalid_argument);
        EXPECT_EQ(manager.snapshot()->size(), beforeSnapshot->size());
        EXPECT_EQ(manager.revision(), beforeFailedRevision);
    }
}

}  // namespace

/** @brief 初始化 GoogleTest 并运行 iG-LIO 测试集。 */
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
