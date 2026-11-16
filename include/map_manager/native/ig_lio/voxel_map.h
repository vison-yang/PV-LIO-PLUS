/*
 * Copyright (C) Zijie Chen, 2023
 * Copyright (C) 2026 PV-LIO-PLUS contributors
 * 许可沿固定上游 LICENSE；版本授予边界见 SOURCE.md。
 */

/**
 * @file voxel_map.h
 * @brief iG-LIO 增量统计格地图的安全原生接口。
 */
#pragma once

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <cstdint>
#include <list>
#include <unordered_map>
#include <vector>

namespace pv_lio_plus::ig_lio_native
{
using CellKey = std::array<std::int32_t, 3>;

/** @brief 原生地图配置；搜索模式沿用上游的 1/7/19/26 命名。 */
struct Config
{
    double resolution_         = 0.5;      ///< 统计格边长，单位为米。
    int searchMethod_          = 7;        ///< 查询格邻域模式。
    std::size_t capacity_      = 5000000;  ///< LRU 淘汰触发阈值；稳态最多保留 capacity-1 格。
    std::size_t gridMaxPoints_ = 20;       ///< 既有格追加存点的统计计数阈值。
};

/** @brief 一条原生邻点及其查询诊断信息。 */
struct Neighbor
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d point_  = Eigen::Vector3d::Zero();  ///< 已按 float 量化存储的世界坐标。
    double distanceSquared_ = 0.0;                      ///< 到 double 查询点的平方距离，单位为平方米。
    CellKey cellKey_{};                                 ///< 邻点所属的完整 int32 格键。
    std::size_t pointIndex_ = 0;                        ///< 邻点在该格存点数组中的索引。
};

using NeighborList = std::vector<Neighbor, Eigen::aligned_allocator<Neighbor>>;

/** @brief 可供独立数值 oracle 读取的单格统计快照。 */
struct GridStats
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CellKey cellKey_{};                                                  ///< 完整格键。
    std::size_t count_                       = 0;                        ///< 上游统计状态中的样本计数 N。
    Eigen::Vector3d pointSum_                = Eigen::Vector3d::Zero();  ///< 一阶矩 S=Σp。
    Eigen::Matrix3d outerSum_                = Eigen::Matrix3d::Zero();  ///< 二阶矩 Q=Σppᵀ。
    Eigen::Vector3d centroid_                = Eigen::Vector3d::Zero();  ///< 当前统计均值。
    Eigen::Matrix3d sampleCovariance_        = Eigen::Matrix3d::Zero();  ///< 未正则化样本协方差 C。
    Eigen::Matrix3d regularizedShape_        = Eigen::Matrix3d::Zero();  ///< 上游 SVD 改形矩阵 UDVᵀ。
    Eigen::Matrix3d inverseRegularizedShape_ = Eigen::Matrix3d::Zero();  ///< 改形矩阵的逆。
    bool isValid_                            = false;                    ///< N≥6 后是否计算过改形矩阵。
    std::size_t retainedPointCount_          = 0;                        ///< 实际保留的 XYZ 点数。
};

using GridStatsList = std::vector<GridStats, Eigen::aligned_allocator<GridStats>>;

/**
 * @brief 维护 iG-LIO 的格统计、XYZ 留存、邻域查询和原生 LRU。
 *
 * 格统计矩阵记录上游的样本矩及 SVD 改形结果，不表示地图平面协方差。
 * 地图按完整 int32 三元组寻址，legacy hash 仅控制批次遍历顺序。
 */
class VoxelMap
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit VoxelMap(const Config &config);

    VoxelMap(const VoxelMap &) = delete;
    VoxelMap &operator=(const VoxelMap &) = delete;

    bool AddCloud(const std::vector<Eigen::Vector3d> &points);

    bool KNNByCondition(const Eigen::Vector3d &point, std::size_t count, double range, NeighborList &results) const;

    std::size_t Size() const;

    std::size_t PointCount() const;

    std::vector<Eigen::Vector3d> Snapshot() const;

    GridStatsList GridStatsSnapshot() const;

    bool GetGridStats(const CellKey &cellKey, GridStats &stats) const;

   private:
    /** @brief 完成整批校验和 float 量化后的点；排序不改变同格输入顺序。 */
    struct PreparedPoint
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d point_ = Eigen::Vector3d::Zero();
        CellKey cellKey_{};
        std::size_t legacyHashIndex_ = 0;
        std::size_t inputIndex_      = 0;
    };

    /** @brief 保存上游格状态及本格 LRU 链表位置。 */
    struct Grid
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        std::size_t count_                       = 0;
        Eigen::Vector3d pointSum_                = Eigen::Vector3d::Zero();
        Eigen::Matrix3d outerSum_                = Eigen::Matrix3d::Zero();
        Eigen::Vector3d centroid_                = Eigen::Vector3d::Zero();
        Eigen::Matrix3d sampleCovariance_        = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d regularizedShape_        = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d inverseRegularizedShape_ = Eigen::Matrix3d::Zero();
        bool isValid_                            = false;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> retainedPoints_;
        std::list<CellKey>::iterator lruPosition_;
    };

    /** @brief 完整三元组的哈希器；哈希碰撞仍由 CellKey 相等判断区分。 */
    struct CellKeyHash
    {
        std::size_t operator()(const CellKey &cellKey) const noexcept;
    };

    using GridMap = std::unordered_map<CellKey, Grid, CellKeyHash>;

    bool TryGridKey(const Eigen::Vector3d &point, CellKey &cellKey) const;

    std::size_t LegacyHashIndex(const CellKey &cellKey) const noexcept;

    void ApplyGridBatch(const CellKey &cellKey, const PreparedPoint *first, const PreparedPoint *last);

    void ComputeShape(Grid &grid) const;

    void FillGridStats(const CellKey &cellKey, const Grid &grid, GridStats &stats) const;

    void Touch(GridMap::iterator grid);

    void EvictLeastRecent();

    Config config_;
    double inverseResolution_ = 0.0;
    std::vector<CellKey> queryOffsets_;
    GridMap grids_;
    std::list<CellKey> lru_;
};

}  // namespace pv_lio_plus::ig_lio_native
