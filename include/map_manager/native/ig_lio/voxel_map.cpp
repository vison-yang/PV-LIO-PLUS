/*
 * Copyright (C) Zijie Chen, 2023
 * Copyright (C) 2026 PV-LIO-PLUS contributors
 * 许可沿固定上游 LICENSE；版本授予边界见 SOURCE.md。
 */

/**
 * @file voxel_map.cpp
 * @brief iG-LIO 增量统计格地图实现。
 */

#include "voxel_map.h"

#include <Eigen/LU>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

namespace pv_lio_plus::ig_lio_native
{
namespace
{
constexpr std::uint32_t g_hashX = 73856093U;
constexpr std::uint32_t g_hashY = 471943U;
constexpr std::uint32_t g_hashZ = 83492791U;

/** @brief 按距离、坐标、完整键和格内索引稳定排列等距邻点。 */
bool NeighborLess(const Neighbor &left, const Neighbor &right)
{
    if (left.distanceSquared_ != right.distanceSquared_) return left.distanceSquared_ < right.distanceSquared_;
    for (int axis = 0; axis < 3; ++axis)
    {
        if (left.point_[axis] != right.point_[axis]) return left.point_[axis] < right.point_[axis];
    }
    if (left.cellKey_ != right.cellKey_) return left.cellKey_ < right.cellKey_;
    return left.pointIndex_ < right.pointIndex_;
}

/** @brief 将完整键转换为不依赖 signed overflow 的 legacy 顺序值。 */
std::uint64_t LegacySignedValue(const CellKey &cellKey)
{
    const std::uint32_t x           = static_cast<std::uint32_t>(cellKey[0]);
    const std::uint32_t y           = static_cast<std::uint32_t>(cellKey[1]);
    const std::uint32_t z           = static_cast<std::uint32_t>(cellKey[2]);
    const std::uint32_t wrappedHash = (x * g_hashX) ^ (y * g_hashY) ^ (z * g_hashZ);

    // 在容量取模前保留旧 int32 转换到 size_t 的符号扩展语义。
    std::uint64_t signedValue = wrappedHash;
    if ((wrappedHash & 0x80000000U) != 0U) signedValue |= 0xffffffff00000000ULL;
    return signedValue;
}
}  // namespace

/**
 * @brief 校验原生参数并生成与上游顺序相同的邻格偏移。
 * @throws std::invalid_argument 分辨率、容量、存点门限或邻域模式非法时抛出。
 */
VoxelMap::VoxelMap(const Config &config) : config_(config)
{
    if (!std::isfinite(config_.resolution_) || config_.resolution_ <= 0.0)
    {
        throw std::invalid_argument("iG-LIO resolution must be finite and positive");
    }
    inverseResolution_ = 1.0 / config_.resolution_;
    if (!std::isfinite(inverseResolution_) || inverseResolution_ <= 0.0)
    {
        throw std::invalid_argument("iG-LIO inverse resolution must be finite and positive");
    }
    if (config_.capacity_ < 2 || config_.gridMaxPoints_ == 0)
    {
        throw std::invalid_argument("iG-LIO capacity must be at least 2 and grid max points must be positive");
    }
    if (config_.searchMethod_ != 1 && config_.searchMethod_ != 7 && config_.searchMethod_ != 19 &&
        config_.searchMethod_ != 26)
    {
        throw std::invalid_argument("iG-LIO search method must be 1, 7, 19, or 26");
    }

    queryOffsets_.push_back({0, 0, 0});
    if (config_.searchMethod_ >= 7)
    {
        queryOffsets_.insert(queryOffsets_.end(),
                             {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}});
    }
    if (config_.searchMethod_ >= 19)
    {
        queryOffsets_.insert(queryOffsets_.end(), {{1, 1, 0},
                                                   {1, -1, 0},
                                                   {1, 0, 1},
                                                   {1, 0, -1},
                                                   {0, 1, 1},
                                                   {0, 1, -1},
                                                   {-1, 1, 0},
                                                   {-1, -1, 0},
                                                   {-1, 0, 1},
                                                   {-1, 0, -1},
                                                   {0, -1, 1},
                                                   {0, -1, -1}});
    }
    if (config_.searchMethod_ == 26)
    {
        queryOffsets_.insert(
            queryOffsets_.end(),
            {{1, 1, 1}, {1, 1, -1}, {1, -1, 1}, {1, -1, -1}, {-1, 1, 1}, {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1}});
    }
}

/**
 * @brief 先整批校验并 float 量化，再按 legacy 顺序分格应用输入批次。
 * @return 输入非空时返回 true；非法点会在任何格状态修改前抛出。
 * @throws std::invalid_argument 任一点非有限、超出 float 或 int32 格键安全域时抛出。
 */
bool VoxelMap::AddCloud(const std::vector<Eigen::Vector3d> &points)
{
    if (points.empty()) return false;

    std::vector<PreparedPoint, Eigen::aligned_allocator<PreparedPoint>> prepared;
    prepared.reserve(points.size());
    const double floatLimit = static_cast<double>(std::numeric_limits<float>::max());
    for (std::size_t index = 0; index < points.size(); ++index)
    {
        if (!points[index].allFinite()) throw std::invalid_argument("iG-LIO point batch must be finite");

        PreparedPoint preparedPoint;
        for (int axis = 0; axis < 3; ++axis)
        {
            if (std::abs(points[index][axis]) > floatLimit)
            {
                throw std::invalid_argument("iG-LIO point is outside the finite float coordinate domain");
            }
            const float quantized = static_cast<float>(points[index][axis]);
            if (!std::isfinite(quantized))
            {
                throw std::invalid_argument("iG-LIO point is outside the finite float coordinate domain");
            }
            preparedPoint.point_[axis] = static_cast<double>(quantized);
        }
        if (!TryGridKey(preparedPoint.point_, preparedPoint.cellKey_))
        {
            throw std::invalid_argument("iG-LIO point is outside the int32 voxel-key domain with neighbor margin");
        }
        preparedPoint.legacyHashIndex_ = LegacyHashIndex(preparedPoint.cellKey_);
        preparedPoint.inputIndex_      = index;
        prepared.push_back(preparedPoint);
    }

    // 完整格键区分 legacy hash 碰撞；输入序号保持同格点的原始顺序。
    std::stable_sort(prepared.begin(), prepared.end(), [](const PreparedPoint &left, const PreparedPoint &right) {
        if (left.legacyHashIndex_ != right.legacyHashIndex_) return left.legacyHashIndex_ < right.legacyHashIndex_;
        if (left.cellKey_ != right.cellKey_) return left.cellKey_ < right.cellKey_;
        return left.inputIndex_ < right.inputIndex_;
    });

    for (std::size_t first = 0; first < prepared.size();)
    {
        std::size_t last = first + 1;
        while (last < prepared.size() && prepared[last].cellKey_ == prepared[first].cellKey_) ++last;
        ApplyGridBatch(prepared[first].cellKey_, prepared.data() + first, prepared.data() + last);
        first = last;
    }
    return true;
}

/**
 * @brief 在指定格邻域内返回严格半径内的稳定 top-K 点，不触碰 LRU。
 * @note 查询保持 binary64；距离比较为严格小于 range²。
 * @return 找到至少一个符合条件的点时返回 true。
 */
bool VoxelMap::KNNByCondition(const Eigen::Vector3d &point, const std::size_t count, const double range,
                              NeighborList &results) const
{
    results.clear();
    const double rangeSquared = range * range;
    if (count == 0 || !point.allFinite() || !std::isfinite(range) || range <= 0.0 || !std::isfinite(rangeSquared) ||
        rangeSquared <= 0.0)
    {
        return false;
    }

    CellKey baseKey{};
    if (!TryGridKey(point, baseKey)) return false;

    NeighborList candidates;
    for (const auto &offset : queryOffsets_)
    {
        const CellKey cellKey{static_cast<std::int32_t>(static_cast<std::int64_t>(baseKey[0]) + offset[0]),
                              static_cast<std::int32_t>(static_cast<std::int64_t>(baseKey[1]) + offset[1]),
                              static_cast<std::int32_t>(static_cast<std::int64_t>(baseKey[2]) + offset[2])};
        const auto grid = grids_.find(cellKey);
        if (grid == grids_.end()) continue;

        for (std::size_t index = 0; index < grid->second.retainedPoints_.size(); ++index)
        {
            const Eigen::Vector3d &candidatePoint = grid->second.retainedPoints_[index];
            const double distanceSquared          = (point - candidatePoint).squaredNorm();
            if (distanceSquared < rangeSquared)
            {
                Neighbor neighbor;
                neighbor.point_           = candidatePoint;
                neighbor.distanceSquared_ = distanceSquared;
                neighbor.cellKey_         = cellKey;
                neighbor.pointIndex_      = index;
                candidates.push_back(neighbor);
            }
        }
    }

    const std::size_t retainedCount = std::min(count, candidates.size());
    std::partial_sort(candidates.begin(), candidates.begin() + retainedCount, candidates.end(), NeighborLess);
    candidates.resize(retainedCount);
    results = std::move(candidates);
    return !results.empty();
}

/** @brief 返回活动统计格数量。 */
std::size_t VoxelMap::Size() const
{
    return grids_.size();
}

/** @brief 返回所有活动格实际保留的 XYZ 点总数。 */
std::size_t VoxelMap::PointCount() const
{
    std::size_t count = 0;
    for (const auto &entry : grids_) count += entry.second.retainedPoints_.size();
    return count;
}

/** @brief 按 LRU 从新到旧、格内按输入顺序返回已保留 XYZ。 */
std::vector<Eigen::Vector3d> VoxelMap::Snapshot() const
{
    std::vector<Eigen::Vector3d> points;
    points.reserve(PointCount());
    for (const auto &cellKey : lru_)
    {
        const auto grid = grids_.find(cellKey);
        if (grid == grids_.end()) continue;
        for (const auto &point : grid->second.retainedPoints_) points.push_back(point);
    }
    return points;
}

/** @brief 按 LRU 从新到旧返回可供独立 oracle 检查的统计副本。 */
GridStatsList VoxelMap::GridStatsSnapshot() const
{
    GridStatsList stats;
    stats.reserve(grids_.size());
    for (const auto &cellKey : lru_)
    {
        const auto grid = grids_.find(cellKey);
        if (grid == grids_.end()) continue;
        GridStats gridStats;
        FillGridStats(cellKey, grid->second, gridStats);
        stats.push_back(gridStats);
    }
    return stats;
}

/** @brief 按完整格键读取统计副本，不更新格的 LRU 位置。 */
bool VoxelMap::GetGridStats(const CellKey &cellKey, GridStats &stats) const
{
    const auto grid = grids_.find(cellKey);
    if (grid == grids_.end()) return false;
    FillGridStats(cellKey, grid->second, stats);
    return true;
}

/** @brief 以定义的 64-bit 混合函数哈希完整 int32 三元组。 */
std::size_t VoxelMap::CellKeyHash::operator()(const CellKey &cellKey) const noexcept
{
    std::uint64_t hash = 0x9e3779b97f4a7c15ULL;
    for (const std::int32_t coordinate : cellKey)
    {
        const std::uint64_t value = static_cast<std::uint32_t>(coordinate);
        hash ^= value + 0x9e3779b97f4a7c15ULL + (hash << 6U) + (hash >> 2U);
    }
    return static_cast<std::size_t>(hash);
}

/** @brief floor(point×inverseResolution) 并保留 ±1 邻格所需的 int32 安全边界。 */
bool VoxelMap::TryGridKey(const Eigen::Vector3d &point, CellKey &cellKey) const
{
    if (!point.allFinite()) return false;

    const double minKey = static_cast<double>(std::numeric_limits<std::int32_t>::min()) + 1.0;
    const double maxKey = static_cast<double>(std::numeric_limits<std::int32_t>::max()) - 1.0;
    for (int axis = 0; axis < 3; ++axis)
    {
        const double scaled = point[axis] * inverseResolution_;
        if (!std::isfinite(scaled)) return false;
        const double floored = std::floor(scaled);
        if (floored < minKey || floored > maxKey) return false;
        cellKey[static_cast<std::size_t>(axis)] = static_cast<std::int32_t>(floored);
    }
    return true;
}

/** @brief 计算 legacy uint32 hash-index，避免 signed overflow 并保留 signed 扩展次序。 */
std::size_t VoxelMap::LegacyHashIndex(const CellKey &cellKey) const noexcept
{
    return static_cast<std::size_t>(LegacySignedValue(cellKey)) % config_.capacity_;
}

/**
 * @brief 更新一个键对应的统计格；每格提交前完成可能分配的存点空间。
 * @note 每格状态在异常前保持完整；已成功处理的前序格允许保留，符合批更新 basic guarantee。
 */
void VoxelMap::ApplyGridBatch(const CellKey &cellKey, const PreparedPoint *first, const PreparedPoint *last)
{
    const std::size_t batchCount  = static_cast<std::size_t>(last - first);
    Eigen::Vector3d batchPointSum = Eigen::Vector3d::Zero();
    Eigen::Matrix3d batchOuterSum = Eigen::Matrix3d::Zero();
    for (const PreparedPoint *point = first; point != last; ++point)
    {
        batchPointSum += point->point_;
        batchOuterSum += point->point_ * point->point_.transpose();
    }

    const auto grid = grids_.find(cellKey);
    if (grid == grids_.end())
    {
        Grid newGrid;
        newGrid.count_    = batchCount;
        newGrid.pointSum_ = batchPointSum;
        newGrid.outerSum_ = batchOuterSum;
        newGrid.centroid_ = newGrid.pointSum_ / static_cast<double>(newGrid.count_);
        ComputeShape(newGrid);
        newGrid.retainedPoints_.reserve(batchCount);
        for (const PreparedPoint *point = first; point != last; ++point)
            newGrid.retainedPoints_.push_back(point->point_);

        // 哈希表分配失败时撤回刚加入的 LRU 节点，保持两个容器一致。
        lru_.push_front(cellKey);
        GridMap::iterator insertedGrid;
        bool isInserted = false;
        try
        {
            const auto result = grids_.emplace(cellKey, std::move(newGrid));
            insertedGrid      = result.first;
            isInserted        = result.second;
        } catch (...)
        {
            lru_.pop_front();
            throw;
        }
        if (!isInserted)
        {
            lru_.pop_front();
            throw std::logic_error("iG-LIO batch key appeared during single-threaded insertion");
        }
        insertedGrid->second.lruPosition_ = lru_.begin();
        if (grids_.size() >= config_.capacity_) EvictLeastRecent();
        return;
    }

    if (grid->second.count_ >= 50)
    {
        Touch(grid);
        return;
    }
    if (batchCount > std::numeric_limits<std::size_t>::max() - grid->second.count_)
    {
        throw std::length_error("iG-LIO grid statistic count overflow");
    }

    Grid nextStats;
    nextStats.count_    = grid->second.count_ + batchCount;
    nextStats.pointSum_ = grid->second.pointSum_ + batchPointSum;
    nextStats.outerSum_ = grid->second.outerSum_ + batchOuterSum;
    nextStats.centroid_ = nextStats.pointSum_ / static_cast<double>(nextStats.count_);
    ComputeShape(nextStats);

    const bool shouldRetainPoints        = nextStats.count_ < config_.gridMaxPoints_;
    const std::size_t originalPointCount = grid->second.retainedPoints_.size();
    if (shouldRetainPoints)
    {
        if (batchCount > std::numeric_limits<std::size_t>::max() - originalPointCount)
        {
            throw std::length_error("iG-LIO retained point count overflow");
        }
        grid->second.retainedPoints_.reserve(originalPointCount + batchCount);
        try
        {
            for (const PreparedPoint *point = first; point != last; ++point)
            {
                grid->second.retainedPoints_.push_back(point->point_);
            }
        } catch (...)
        {
            while (grid->second.retainedPoints_.size() > originalPointCount) grid->second.retainedPoints_.pop_back();
            throw;
        }
    }

    // 所有存点分配完成后，再无分配地提交本格统计状态。
    grid->second.count_                   = nextStats.count_;
    grid->second.pointSum_                = nextStats.pointSum_;
    grid->second.outerSum_                = nextStats.outerSum_;
    grid->second.centroid_                = nextStats.centroid_;
    grid->second.sampleCovariance_        = nextStats.sampleCovariance_;
    grid->second.regularizedShape_        = nextStats.regularizedShape_;
    grid->second.inverseRegularizedShape_ = nextStats.inverseRegularizedShape_;
    grid->second.isValid_                 = nextStats.isValid_;
    Touch(grid);
}

/**
 * @brief 按上游样本矩公式计算 raw C，再用 SVD 奇异谱生成改形矩阵。
 * @note 改形固定使用 U·diag(1,1,1e-3)·Vᵀ；它不表示严格概率协方差或平面不确定性。
 */
void VoxelMap::ComputeShape(Grid &grid) const
{
    if (grid.count_ < 6)
    {
        grid.isValid_ = false;
        return;
    }

    grid.sampleCovariance_ =
        (grid.outerSum_ - grid.pointSum_ * grid.centroid_.transpose()) / (static_cast<double>(grid.count_) - 1.0);
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(grid.sampleCovariance_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Vector3d spectrum(1.0, 1.0, 1e-3);
    grid.regularizedShape_        = svd.matrixU() * spectrum.asDiagonal() * svd.matrixV().transpose();
    grid.inverseRegularizedShape_ = grid.regularizedShape_.inverse();
    grid.isValid_                 = true;
}

/** @brief 将内部格状态复制到不暴露可变存储的诊断快照。 */
void VoxelMap::FillGridStats(const CellKey &cellKey, const Grid &grid, GridStats &stats) const
{
    stats.cellKey_                 = cellKey;
    stats.count_                   = grid.count_;
    stats.pointSum_                = grid.pointSum_;
    stats.outerSum_                = grid.outerSum_;
    stats.centroid_                = grid.centroid_;
    stats.sampleCovariance_        = grid.sampleCovariance_;
    stats.regularizedShape_        = grid.regularizedShape_;
    stats.inverseRegularizedShape_ = grid.inverseRegularizedShape_;
    stats.isValid_                 = grid.isValid_;
    stats.retainedPointCount_      = grid.retainedPoints_.size();
}

/** @brief 将已查询格移至 LRU 首位；纯查询不调用此函数。 */
void VoxelMap::Touch(GridMap::iterator grid)
{
    if (grid->second.lruPosition_ != lru_.begin())
    {
        lru_.splice(lru_.begin(), lru_, grid->second.lruPosition_);
        grid->second.lruPosition_ = lru_.begin();
    }
}

/** @brief 删除 LRU 尾格；上游在插入后 size≥capacity 时触发。 */
void VoxelMap::EvictLeastRecent()
{
    if (lru_.empty()) return;
    const CellKey cellKey = lru_.back();
    lru_.pop_back();
    grids_.erase(cellKey);
}

}  // namespace pv_lio_plus::ig_lio_native
