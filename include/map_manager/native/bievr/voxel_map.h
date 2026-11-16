/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors
 * Upstream BSD-3-Clause notice is retained in LICENSE; see SOURCE.md.
 */

/**
 * @file voxel_map.h
 * @brief BIEVR 定向高度图的串行原生地图接口。
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <list>
#include <unordered_map>
#include <vector>

namespace pv_lio_plus::bievr_native
{
/** @brief 一个体素的累计几何统计、定向坐标系和高度像素。 */
struct Voxel
{
    bool observed_                     = false;
    Eigen::Isometry3d cameraFromWorld_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d objectFromWorld_ = Eigen::Isometry3d::Identity();
    Eigen::MatrixXf image_;
    Eigen::MatrixXf smoothed_;
    Eigen::MatrixXf weights_;
    Eigen::Vector3d pointSum_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d outerSum_ = Eigen::Matrix3d::Zero();
    std::size_t numPoints_    = 0;
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> pendingPoints_;
};

/** @brief Morton 21 位体素键、定向高度像素和 LRU 容量的参数。 */
struct Config
{
    double voxelSize_           = 0.5;
    double pixelSize_           = 0.05;
    std::size_t capacity_       = 10000000;
    bool weighted_              = false;
    bool smooth_                = false;
    double normalToleranceDeg_  = 3.0;
    std::size_t maxImagePixels_ = 1048576;
};

/**
 * @brief 维护 BIEVR 定向高度图、成熟前缓存和按最近更新淘汰的体素。
 *
 * 所有更新和读取串行调用；GetVoxel/FindVoxel 返回的地址只在下一次非常量操作前有效。
 */
class Map
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit Map(const Config &config);

    Map(const Map &) = delete;
    Map &operator=(const Map &) = delete;
    Map(Map &&)                 = delete;
    Map &operator=(Map &&) = delete;

    void Configure(const Config &config);

    bool Integrate(const std::vector<Eigen::Vector3d> &worldPoints, const std::vector<double> &ranges);

    const Voxel *GetVoxel(const Eigen::Vector3d &worldPoint) const;

    const Voxel *FindVoxel(const Eigen::Vector3d &worldPoint) const;

    std::size_t Size() const;

    std::size_t PointCount() const;

    std::vector<Eigen::Vector3d> Snapshot() const;

   private:
    struct PreparedPoint
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d worldPoint_ = Eigen::Vector3d::Zero();
        double range_               = 0.0;
        std::uint64_t mortonKey_    = 0;
        std::size_t inputIndex_     = 0;
    };

    struct VoxelEntry
    {
        Voxel voxel_;
        std::list<std::uint64_t>::iterator lruPosition_;
    };

    using VoxelMap = std::unordered_map<std::uint64_t, VoxelEntry>;

    static bool IsValidConfig(const Config &config);
    bool TryMortonKey(const Eigen::Vector3d &worldPoint, std::uint64_t &mortonKey) const;
    static std::uint64_t EncodeMorton(const Eigen::Vector3i &voxelIndex);
    static Eigen::Vector3i DecodeMorton(std::uint64_t mortonKey);
    static bool TryRoundPixel(double coordinate, int &pixelIndex);
    Eigen::Vector3d VoxelOrigin(const Eigen::Vector3i &voxelIndex) const;
    bool UpdateNormal(Voxel &voxel);
    void UpdateImage(const std::vector<PreparedPoint> &points, std::size_t first, std::size_t last, Voxel &voxel,
                     bool normalChanged);
    void ReprojectImage(Voxel &voxel, double uMin, double vMin, int rows, int cols, Eigen::MatrixXi &changed);
    void IntegratePixels(const std::vector<PreparedPoint> &points, std::size_t first, std::size_t last, Voxel &voxel);
    void SmoothChangedPixels(Voxel &voxel, const Eigen::MatrixXi &changed);
    void Touch(VoxelMap::iterator voxel);
    void EvictLeastRecent();

    Config config_;
    double inverseVoxelSize_   = 0.0;
    double inversePixelSize_   = 0.0;
    double normalToleranceRad_ = 0.0;
    VoxelMap voxels_;
    std::list<std::uint64_t> lru_;
};

}  // namespace pv_lio_plus::bievr_native
