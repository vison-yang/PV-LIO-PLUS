/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors
 * Upstream BSD-3-Clause notice is retained in LICENSE; see SOURCE.md.
 */

/**
 * @file voxel_map.cpp
 * @brief BIEVR 定向高度图串行融合、投影和缓存实现。
 */

#include "voxel_map.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace pv_lio_plus::bievr_native
{
namespace
{
constexpr std::int32_t g_mortonOffset = 1 << 20;
constexpr std::int32_t g_mortonMin    = -g_mortonOffset;
constexpr std::int32_t g_mortonMax    = g_mortonOffset - 1;
constexpr double g_pi                 = 3.141592653589793238462643383279502884;

/** @brief 将 21 位坐标展开到 Morton 编码的隔位位置。 */
std::uint64_t ExpandBits(const std::int32_t coordinate)
{
    std::uint64_t value = static_cast<std::uint64_t>(static_cast<std::int64_t>(coordinate) + g_mortonOffset);
    value &= 0x1fffffULL;
    value = (value | (value << 32U)) & 0x1f00000000ffffULL;
    value = (value | (value << 16U)) & 0x1f0000ff0000ffULL;
    value = (value | (value << 8U)) & 0x100f00f00f00f00fULL;
    value = (value | (value << 4U)) & 0x10c30c30c30c30c3ULL;
    value = (value | (value << 2U)) & 0x1249249249249249ULL;
    return value;
}

/** @brief 返回作者实现使用的半径 1、高斯 sigma 0.8 平滑核。 */
const Eigen::Matrix3f &GaussianKernel()
{
    static const Eigen::Matrix3f kernel = [] {
        Eigen::Matrix3f values;
        float sum = 0.0f;
        for (int y = -1; y <= 1; ++y)
        {
            for (int x = -1; x <= 1; ++x)
            {
                const float value =
                    static_cast<float>(std::exp(-static_cast<double>(x * x + y * y) / (2.0 * 0.8 * 0.8)));
                values(y + 1, x + 1) = value;
                sum += value;
            }
        }
        values /= sum;
        return values;
    }();
    return kernel;
}

/** @brief 计算范围加权开关对应的正融合权重。 */
double EffectiveWeight(const bool weighted, const double range)
{
    return weighted ? std::min(0.5, 1.0 / range) : 1.0;
}

/** @brief 检查 double 值转换到像素高度 float 后仍有限。 */
bool IsFiniteFloat(const double value)
{
    return std::isfinite(value) && std::abs(value) <= static_cast<double>(std::numeric_limits<float>::max());
}
}  // namespace

/**
 * @brief 以有效配置初始化空地图。
 * @throws std::invalid_argument 分辨率、容量、角度或像素尺寸上限非法时抛出。
 */
Map::Map(const Config &config)
{
    Configure(config);
}

/**
 * @brief 先校验配置，再释放旧体素并开始新的空地图生命周期。
 * @throws std::invalid_argument 配置非法时保留地图原状态并抛出。
 */
void Map::Configure(const Config &config)
{
    if (!IsValidConfig(config)) throw std::invalid_argument("BIEVR map configuration is invalid");

    Config nextConfig                   = config;
    const double nextInverseVoxelSize   = 1.0 / config.voxelSize_;
    const double nextInversePixelSize   = 1.0 / config.pixelSize_;
    const double nextNormalToleranceRad = config.normalToleranceDeg_ * g_pi / 180.0;
    VoxelMap nextVoxels;
    std::list<std::uint64_t> nextLru;

    config_             = nextConfig;
    inverseVoxelSize_   = nextInverseVoxelSize;
    inversePixelSize_   = nextInversePixelSize;
    normalToleranceRad_ = nextNormalToleranceRad;
    voxels_.swap(nextVoxels);
    lru_.swap(nextLru);
}

/** @brief 判断配置能否用于有限 Morton 键、图像和 LRU 存储。 */
bool Map::IsValidConfig(const Config &config)
{
    if (!std::isfinite(config.voxelSize_) || config.voxelSize_ <= 0.0 || !std::isfinite(1.0 / config.voxelSize_) ||
        1.0 / config.voxelSize_ <= 0.0)
    {
        return false;
    }
    if (!std::isfinite(config.pixelSize_) || config.pixelSize_ <= 0.0 || !std::isfinite(1.0 / config.pixelSize_) ||
        1.0 / config.pixelSize_ <= 0.0)
    {
        return false;
    }
    if (config.capacity_ == 0 || config.maxImagePixels_ == 0 || !std::isfinite(config.normalToleranceDeg_) ||
        config.normalToleranceDeg_ < 0.0 || config.normalToleranceDeg_ > 180.0)
    {
        return false;
    }

    const double maximumOrigin = static_cast<double>(g_mortonOffset) * config.voxelSize_;
    return std::isfinite(maximumOrigin);
}

/**
 * @brief 全批预检后以稳定 Morton 分组顺序更新统计、图像和 LRU。
 * @return 非空批次成功时为 true，空批次返回 false。
 * @throws std::invalid_argument 长度、有限性、正量程或编码域非法时，在任何地图写入前抛出。
 */
bool Map::Integrate(const std::vector<Eigen::Vector3d> &worldPoints, const std::vector<double> &ranges)
{
    if (worldPoints.size() != ranges.size()) throw std::invalid_argument("BIEVR point/range lengths differ");
    if (worldPoints.empty()) return false;

    std::vector<PreparedPoint> points;
    points.reserve(worldPoints.size());
    const float maximumWeight = std::numeric_limits<float>::max();
    for (std::size_t index = 0; index < worldPoints.size(); ++index)
    {
        if (!worldPoints[index].allFinite()) throw std::invalid_argument("BIEVR world points must be finite");
        if (!std::isfinite(ranges[index]) || ranges[index] <= 0.0)
        {
            throw std::invalid_argument("BIEVR ranges must be finite and positive");
        }
        std::uint64_t mortonKey = 0;
        if (!TryMortonKey(worldPoints[index], mortonKey))
        {
            throw std::invalid_argument("BIEVR point is outside the 21-bit Morton domain");
        }
        const double weight      = EffectiveWeight(config_.weighted_, ranges[index]);
        const float storedWeight = static_cast<float>(weight);
        if (!std::isfinite(weight) || weight <= 0.0 || !std::isfinite(storedWeight) || storedWeight <= 0.0f ||
            storedWeight > maximumWeight)
        {
            throw std::invalid_argument("BIEVR range weight is not representable in the image");
        }

        PreparedPoint preparedPoint;
        preparedPoint.worldPoint_ = worldPoints[index];
        preparedPoint.range_      = ranges[index];
        preparedPoint.mortonKey_  = mortonKey;
        preparedPoint.inputIndex_ = index;
        points.push_back(preparedPoint);
    }

    // 稳定保留同 x 输入次序，修复单体素批次未生成起点的问题。
    std::stable_sort(points.begin(), points.end(), [](const PreparedPoint &left, const PreparedPoint &right) {
        if (left.mortonKey_ != right.mortonKey_) return left.mortonKey_ < right.mortonKey_;
        return left.worldPoint_.x() < right.worldPoint_.x();
    });
    std::vector<std::size_t> starts{0};
    starts.reserve(points.size() + 1);
    for (std::size_t index = 1; index < points.size(); ++index)
    {
        if (points[index].mortonKey_ != points[index - 1].mortonKey_) starts.push_back(index);
    }
    starts.push_back(points.size());

    // 完整模拟统计累加和计数上界，防止输入数值问题造成半批次写入。
    for (std::size_t group = 0; group + 1 < starts.size(); ++group)
    {
        const auto existing = voxels_.find(points[starts[group]].mortonKey_);
        Eigen::Vector3d simulatedSum =
            existing == voxels_.end() ? Eigen::Vector3d::Zero() : existing->second.voxel_.pointSum_;
        Eigen::Matrix3d simulatedOuter =
            existing == voxels_.end() ? Eigen::Matrix3d::Zero() : existing->second.voxel_.outerSum_;
        const std::size_t groupSize = starts[group + 1] - starts[group];
        const std::size_t oldCount  = existing == voxels_.end() ? 0 : existing->second.voxel_.numPoints_;
        if (groupSize > std::numeric_limits<std::size_t>::max() - oldCount)
        {
            throw std::invalid_argument("BIEVR voxel point count would overflow");
        }
        for (std::size_t index = starts[group]; index < starts[group + 1]; ++index)
        {
            const Eigen::Vector3d &point = points[index].worldPoint_;
            const Eigen::Matrix3d outer  = point * point.transpose();
            simulatedSum += point;
            simulatedOuter += outer;
            if (!outer.allFinite() || !simulatedSum.allFinite() || !simulatedOuter.allFinite())
            {
                throw std::invalid_argument("BIEVR point statistics exceed finite double range");
            }
        }
    }

    for (std::size_t group = 0; group + 1 < starts.size(); ++group)
    {
        const std::size_t first       = starts[group];
        const std::size_t last        = starts[group + 1];
        const std::uint64_t mortonKey = points[first].mortonKey_;
        auto voxelIter                = voxels_.find(mortonKey);
        if (voxelIter == voxels_.end())
        {
            lru_.push_front(mortonKey);
            try
            {
                auto inserted = voxels_.emplace(mortonKey, VoxelEntry{Voxel{}, lru_.begin()});
                voxelIter     = inserted.first;
                voxelIter->second.voxel_.pendingPoints_.reserve(8);
            } catch (...)
            {
                if (!lru_.empty() && lru_.front() == mortonKey && voxels_.find(mortonKey) == voxels_.end())
                {
                    lru_.pop_front();
                }
                throw;
            }
        }

        Voxel &voxel = voxelIter->second.voxel_;
        std::vector<PreparedPoint> voxelPoints;
        const std::size_t pendingSize = voxel.pendingPoints_.size();
        voxelPoints.reserve(last - first + pendingSize);
        for (std::size_t index = first; index < last; ++index)
        {
            voxel.pointSum_ += points[index].worldPoint_;
            voxel.outerSum_.noalias() += points[index].worldPoint_ * points[index].worldPoint_.transpose();
            ++voxel.numPoints_;
            voxelPoints.push_back(points[index]);
        }

        const bool observedBefore = voxel.observed_;
        const bool normalChanged  = UpdateNormal(voxel);
        if (!observedBefore && voxel.observed_)
        {
            // 作者先写入当前批，再投影成熟前缓存的历史点并释放缓存。
            for (const Eigen::Vector4d &pending : voxel.pendingPoints_)
            {
                PreparedPoint pendingPoint;
                pendingPoint.worldPoint_ = pending.head<3>();
                pendingPoint.range_      = pending[3];
                pendingPoint.mortonKey_  = mortonKey;
                voxelPoints.push_back(pendingPoint);
            }
            std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>().swap(voxel.pendingPoints_);
        }
        else if (!voxel.observed_)
        {
            for (std::size_t index = first; index < last; ++index)
            {
                Eigen::Vector4d pendingPoint;
                pendingPoint.head<3>() = points[index].worldPoint_;
                pendingPoint[3]        = points[index].range_;
                voxel.pendingPoints_.push_back(pendingPoint);
            }
        }

        UpdateImage(voxelPoints, 0, voxelPoints.size(), voxel, normalChanged);
        Touch(voxelIter);
    }

    while (voxels_.size() > config_.capacity_) EvictLeastRecent();
    return true;
}

/** @brief 将有限世界点按 floor(point / voxelSize) 转为 21 位 Morton 键。 */
bool Map::TryMortonKey(const Eigen::Vector3d &worldPoint, std::uint64_t &mortonKey) const
{
    if (!worldPoint.allFinite()) return false;
    Eigen::Vector3i voxelIndex;
    for (int axis = 0; axis < 3; ++axis)
    {
        const double scaled = worldPoint[axis] * inverseVoxelSize_;
        if (!std::isfinite(scaled)) return false;
        const double floored = std::floor(scaled);
        if (floored < static_cast<double>(g_mortonMin) || floored > static_cast<double>(g_mortonMax)) return false;
        voxelIndex[axis] = static_cast<std::int32_t>(floored);
    }
    mortonKey = EncodeMorton(voxelIndex);
    return true;
}

/** @brief 将三轴 21 位坐标编码为无别名 Morton 键。 */
std::uint64_t Map::EncodeMorton(const Eigen::Vector3i &voxelIndex)
{
    return ExpandBits(voxelIndex.x()) | (ExpandBits(voxelIndex.y()) << 1U) | (ExpandBits(voxelIndex.z()) << 2U);
}

/** @brief 将 63 位 Morton 键逐位解码为带符号体素索引。 */
Eigen::Vector3i Map::DecodeMorton(const std::uint64_t mortonKey)
{
    Eigen::Vector3i voxelIndex;
    for (int axis = 0; axis < 3; ++axis)
    {
        std::uint32_t coordinate = 0;
        for (int bit = 0; bit < 21; ++bit)
        {
            coordinate |= static_cast<std::uint32_t>((mortonKey >> (3 * bit + axis)) & 1ULL) << bit;
        }
        voxelIndex[axis] = static_cast<std::int32_t>(coordinate) - g_mortonOffset;
    }
    return voxelIndex;
}

/** @brief 仅在 round 结果可安全转成 int 时返回像素索引。 */
bool Map::TryRoundPixel(const double coordinate, int &pixelIndex)
{
    if (!std::isfinite(coordinate)) return false;
    const double rounded = std::round(coordinate);
    if (!std::isfinite(rounded) || rounded < static_cast<double>(std::numeric_limits<int>::min()) ||
        rounded > static_cast<double>(std::numeric_limits<int>::max()))
    {
        return false;
    }
    pixelIndex = static_cast<int>(rounded);
    return true;
}

/** @brief 返回与体素索引对应的世界坐标原点。 */
Eigen::Vector3d Map::VoxelOrigin(const Eigen::Vector3i &voxelIndex) const
{
    return voxelIndex.cast<double>() * config_.voxelSize_;
}

/**
 * @brief 以 n−1 散布矩阵最小特征向量更新定向平面坐标系。
 * @return 首次观测或超过法向阈值时返回 true。
 * @throws std::runtime_error 特征求解或构造有限正交坐标系失败时抛出。
 */
bool Map::UpdateNormal(Voxel &voxel)
{
    if (voxel.numPoints_ < 4) return false;
    const Eigen::Vector3d mean = voxel.pointSum_ / static_cast<double>(voxel.numPoints_);
    const Eigen::Matrix3d scatter =
        (voxel.outerSum_ - voxel.pointSum_ * mean.transpose()) / (static_cast<double>(voxel.numPoints_) - 1.0);
    if (!mean.allFinite() || !scatter.allFinite()) throw std::runtime_error("BIEVR scatter matrix is not finite");

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(scatter);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() || !solver.eigenvectors().allFinite())
    {
        throw std::runtime_error("BIEVR scatter eigensolver failed");
    }
    Eigen::Vector3d normal    = solver.eigenvectors().col(0);
    const double normalLength = normal.norm();
    if (!std::isfinite(normalLength) || normalLength <= 0.0) throw std::runtime_error("BIEVR normal is invalid");
    normal /= normalLength;

    bool normalChanged = !voxel.observed_;
    if (voxel.observed_)
    {
        const Eigen::Vector3d previousNormal = voxel.objectFromWorld_.linear().row(2).transpose();
        const double cosine                  = std::clamp(std::abs(previousNormal.dot(normal)), 0.0, 1.0);
        const double angle                   = std::acos(cosine);
        if (!std::isfinite(angle)) throw std::runtime_error("BIEVR normal angle is not finite");
        normalChanged = angle > normalToleranceRad_;
    }

    if (normalChanged)
    {
        const Eigen::Vector3d xAxis = normal.unitOrthogonal();
        const Eigen::Vector3d yAxis = normal.cross(xAxis).normalized();
        if (!xAxis.allFinite() || !yAxis.allFinite()) throw std::runtime_error("BIEVR frame axes are invalid");
        Eigen::Isometry3d worldFromObject = Eigen::Isometry3d::Identity();
        worldFromObject.linear().col(0)   = xAxis;
        worldFromObject.linear().col(1)   = yAxis;
        worldFromObject.linear().col(2)   = normal;
        worldFromObject.translation()     = mean;
        if (!worldFromObject.matrix().allFinite()) throw std::runtime_error("BIEVR object frame is not finite");
        voxel.objectFromWorld_ = worldFromObject.inverse();
    }
    voxel.observed_ = true;
    return normalChanged;
}

/** @brief 在必要时重投影旧像素，再融合当前点并执行掩码平滑。 */
void Map::UpdateImage(const std::vector<PreparedPoint> &points, const std::size_t first, const std::size_t last,
                      Voxel &voxel, const bool normalChanged)
{
    if (!voxel.observed_) return;
    if (first >= last) throw std::runtime_error("BIEVR observed voxel update has no points");

    Eigen::MatrixXi changed;
    if (normalChanged)
    {
        const Eigen::Vector3i voxelIndex = DecodeMorton(points[first].mortonKey_);
        const Eigen::Vector3d origin     = VoxelOrigin(voxelIndex);
        double uMin                      = std::numeric_limits<double>::infinity();
        double uMax                      = -std::numeric_limits<double>::infinity();
        double vMin                      = std::numeric_limits<double>::infinity();
        double vMax                      = -std::numeric_limits<double>::infinity();
        for (int corner = 0; corner < 8; ++corner)
        {
            const Eigen::Vector3d worldCorner  = origin + Eigen::Vector3d((corner & 1) ? config_.voxelSize_ : 0.0,
                                                                         (corner & 2) ? config_.voxelSize_ : 0.0,
                                                                         (corner & 4) ? config_.voxelSize_ : 0.0);
            const Eigen::Vector3d objectCorner = voxel.objectFromWorld_ * worldCorner;
            if (!objectCorner.allFinite()) throw std::runtime_error("BIEVR projected voxel corner is not finite");
            uMin = std::min(uMin, objectCorner.x());
            uMax = std::max(uMax, objectCorner.x());
            vMin = std::min(vMin, objectCorner.y());
            vMax = std::max(vMax, objectCorner.y());
        }

        const double widthValue  = std::ceil((uMax - uMin) * inversePixelSize_) + 1.0;
        const double heightValue = std::ceil((vMax - vMin) * inversePixelSize_) + 1.0;
        const double intLimit    = static_cast<double>(std::numeric_limits<int>::max());
        if (!std::isfinite(widthValue) || !std::isfinite(heightValue) || widthValue < 1.0 || heightValue < 1.0 ||
            widthValue > intLimit || heightValue > intLimit)
        {
            throw std::runtime_error("BIEVR image dimensions exceed the integer domain");
        }
        const auto cols = static_cast<std::size_t>(widthValue);
        const auto rows = static_cast<std::size_t>(heightValue);
        if (cols > config_.maxImagePixels_ / rows)
        {
            throw std::runtime_error("BIEVR image exceeds configured maxImagePixels");
        }
        ReprojectImage(voxel, uMin, vMin, static_cast<int>(rows), static_cast<int>(cols), changed);
    }
    else
    {
        changed = Eigen::MatrixXi::Zero(voxel.image_.rows(), voxel.image_.cols());
    }

    IntegratePixels(points, first, last, voxel);
    for (std::size_t index = first; index < last; ++index)
    {
        const Eigen::Vector3d pointCamera = voxel.cameraFromWorld_ * points[index].worldPoint_;
        int xPixel                        = 0;
        int yPixel                        = 0;
        if (!TryRoundPixel(pointCamera.x() * inversePixelSize_, xPixel) ||
            !TryRoundPixel(pointCamera.y() * inversePixelSize_, yPixel))
        {
            throw std::runtime_error("BIEVR pixel coordinate cannot be represented as int");
        }
        if (xPixel < 0 || xPixel >= voxel.image_.cols() || yPixel < 0 || yPixel >= voxel.image_.rows())
        {
            throw std::runtime_error("BIEVR point projects outside its bounded image");
        }
        changed(yPixel, xPixel) = 1;
    }

    if (!config_.smooth_)
    {
        voxel.smoothed_ = voxel.image_;
        return;
    }

    if (!normalChanged)
    {
        Eigen::MatrixXi dilated = Eigen::MatrixXi::Zero(changed.rows(), changed.cols());
        for (int y = 0; y < changed.rows(); ++y)
        {
            for (int x = 0; x < changed.cols(); ++x)
            {
                if (changed(y, x) == 0) continue;
                for (int offsetY = -1; offsetY <= 1; ++offsetY)
                {
                    for (int offsetX = -1; offsetX <= 1; ++offsetX)
                    {
                        const int neighborY = y + offsetY;
                        const int neighborX = x + offsetX;
                        if (neighborY < 0 || neighborY >= changed.rows() || neighborX < 0 ||
                            neighborX >= changed.cols())
                        {
                            continue;
                        }
                        if (voxel.weights_(neighborY, neighborX) > 0.0f) dilated(neighborY, neighborX) = 1;
                    }
                }
            }
        }
        changed.swap(dilated);
    }
    SmoothChangedPixels(voxel, changed);
}

/** @brief 建立新相机坐标并将旧图有效像素投影到新图。 */
void Map::ReprojectImage(Voxel &voxel, const double uMin, const double vMin, const int rows, const int cols,
                         Eigen::MatrixXi &changed)
{
    const Eigen::MatrixXf oldImage             = voxel.image_;
    const Eigen::MatrixXf oldWeights           = voxel.weights_;
    const Eigen::Isometry3d worldFromOldCamera = voxel.cameraFromWorld_.inverse();
    voxel.image_                               = Eigen::MatrixXf::Zero(rows, cols);
    voxel.smoothed_                            = Eigen::MatrixXf::Zero(rows, cols);
    voxel.weights_                             = Eigen::MatrixXf::Zero(rows, cols);
    changed                                    = Eigen::MatrixXi::Zero(rows, cols);

    Eigen::Isometry3d worldFromCamera = Eigen::Isometry3d::Identity();
    worldFromCamera.linear()          = voxel.objectFromWorld_.linear().transpose();
    worldFromCamera.translation()     = voxel.objectFromWorld_.inverse() * Eigen::Vector3d(uMin, vMin, 0.0);
    voxel.cameraFromWorld_            = worldFromCamera.inverse();
    if (!voxel.cameraFromWorld_.matrix().allFinite()) throw std::runtime_error("BIEVR camera transform is not finite");
    const Eigen::Isometry3d cameraFromOldCamera = voxel.cameraFromWorld_ * worldFromOldCamera;

    // 重投影按旧图行列顺序覆盖新像素，保持作者的碰撞写入次序。
    for (int y = 0; y < oldImage.rows(); ++y)
    {
        for (int x = 0; x < oldImage.cols(); ++x)
        {
            if (oldWeights(y, x) <= 0.0f) continue;
            const Eigen::Vector3d oldPoint(static_cast<double>(x) * config_.pixelSize_,
                                           static_cast<double>(y) * config_.pixelSize_, oldImage(y, x));
            const Eigen::Vector3d newPoint = cameraFromOldCamera * oldPoint;
            int newX                       = 0;
            int newY                       = 0;
            if (!newPoint.allFinite() || !TryRoundPixel(newPoint.x() * inversePixelSize_, newX) ||
                !TryRoundPixel(newPoint.y() * inversePixelSize_, newY))
            {
                throw std::runtime_error("BIEVR reprojected pixel is not finite or representable");
            }
            if (newX < 0 || newX >= cols || newY < 0 || newY >= rows) continue;
            if (!IsFiniteFloat(newPoint.z())) throw std::runtime_error("BIEVR reprojected height exceeds float range");
            voxel.image_(newY, newX)   = static_cast<float>(newPoint.z());
            voxel.weights_(newY, newX) = oldWeights(y, x);
            changed(newY, newX)        = 1;
        }
    }
}

/** @brief 以原像素权重和范围权重融合当前批次的高度值。 */
void Map::IntegratePixels(const std::vector<PreparedPoint> &points, const std::size_t first, const std::size_t last,
                          Voxel &voxel)
{
    for (std::size_t index = first; index < last; ++index)
    {
        const Eigen::Vector3d pointCamera = voxel.cameraFromWorld_ * points[index].worldPoint_;
        int xPixel                        = 0;
        int yPixel                        = 0;
        if (!pointCamera.allFinite() || !TryRoundPixel(pointCamera.x() * inversePixelSize_, xPixel) ||
            !TryRoundPixel(pointCamera.y() * inversePixelSize_, yPixel))
        {
            throw std::runtime_error("BIEVR point pixel coordinate is not finite or representable");
        }
        if (xPixel < 0 || xPixel >= voxel.image_.cols() || yPixel < 0 || yPixel >= voxel.image_.rows())
        {
            throw std::runtime_error("BIEVR point projects outside its bounded image");
        }

        const double newWeight      = EffectiveWeight(config_.weighted_, points[index].range_);
        const double oldWeight      = static_cast<double>(voxel.weights_(yPixel, xPixel));
        const double combinedWeight = oldWeight + newWeight;
        const double height =
            (oldWeight * static_cast<double>(voxel.image_(yPixel, xPixel)) + newWeight * pointCamera.z()) /
            combinedWeight;
        if (!std::isfinite(combinedWeight) || combinedWeight <= 0.0 || !IsFiniteFloat(combinedWeight) ||
            !IsFiniteFloat(pointCamera.z()) || !IsFiniteFloat(height))
        {
            throw std::runtime_error("BIEVR pixel accumulation exceeds finite float range");
        }
        voxel.weights_(yPixel, xPixel) = static_cast<float>(combinedWeight);
        voxel.image_(yPixel, xPixel)   = static_cast<float>(height);
    }
}

/** @brief 对变化像素用有效权重掩码执行半径 1 Gaussian 平滑。 */
void Map::SmoothChangedPixels(Voxel &voxel, const Eigen::MatrixXi &changed)
{
    const Eigen::Matrix3f &kernel = GaussianKernel();
    for (int y = 0; y < changed.rows(); ++y)
    {
        for (int x = 0; x < changed.cols(); ++x)
        {
            if (changed(y, x) == 0) continue;
            float weightedSum = 0.0f;
            float weightSum   = 0.0f;
            for (int offsetY = -1; offsetY <= 1; ++offsetY)
            {
                const int neighborY = y + offsetY;
                if (neighborY < 0 || neighborY >= changed.rows()) continue;
                for (int offsetX = -1; offsetX <= 1; ++offsetX)
                {
                    const int neighborX = x + offsetX;
                    if (neighborX < 0 || neighborX >= changed.cols() || voxel.weights_(neighborY, neighborX) <= 0.0f)
                    {
                        continue;
                    }
                    const float weight = kernel(offsetY + 1, offsetX + 1);
                    weightedSum += voxel.image_(neighborY, neighborX) * weight;
                    weightSum += weight;
                }
            }
            voxel.smoothed_(y, x) = weightSum > 0.0f ? weightedSum / weightSum : 0.0f;
        }
    }
}

/** @brief 将体素移至 LRU 链表头部。 */
void Map::Touch(VoxelMap::iterator voxel)
{
    lru_.splice(lru_.begin(), lru_, voxel->second.lruPosition_);
}

/** @brief 淘汰最近最少更新的尾部体素。 */
void Map::EvictLeastRecent()
{
    if (lru_.empty()) return;
    voxels_.erase(lru_.back());
    lru_.pop_back();
}

/** @brief 返回查询点所在格；无效或越界查询返回空指针且不触碰 LRU。 */
const Voxel *Map::GetVoxel(const Eigen::Vector3d &worldPoint) const
{
    std::uint64_t mortonKey = 0;
    if (!TryMortonKey(worldPoint, mortonKey)) return nullptr;
    const auto voxel = voxels_.find(mortonKey);
    return voxel == voxels_.end() ? nullptr : &voxel->second.voxel_;
}

/** @brief 按 self、+x、-x、+y、-y、+z、-z 查找最近成熟体素。 */
const Voxel *Map::FindVoxel(const Eigen::Vector3d &worldPoint) const
{
    std::uint64_t mortonKey = 0;
    if (!TryMortonKey(worldPoint, mortonKey)) return nullptr;
    const auto local = voxels_.find(mortonKey);
    if (local != voxels_.end() && local->second.voxel_.observed_) return &local->second.voxel_;

    const Eigen::Vector3i base                          = DecodeMorton(mortonKey);
    constexpr std::array<std::array<int, 3>, 6> offsets = {
        {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}}};
    const Voxel *closest   = nullptr;
    double minimumDistance = std::numeric_limits<double>::infinity();
    for (const auto &offset : offsets)
    {
        Eigen::Vector3i neighbor = base;
        bool valid               = true;
        for (int axis = 0; axis < 3; ++axis)
        {
            const std::int64_t coordinate = static_cast<std::int64_t>(base[axis]) + offset[axis];
            if (coordinate < g_mortonMin || coordinate > g_mortonMax)
            {
                valid = false;
                break;
            }
            neighbor[axis] = static_cast<int>(coordinate);
        }
        if (!valid) continue;
        const auto voxelIter = voxels_.find(EncodeMorton(neighbor));
        if (voxelIter == voxels_.end() || !voxelIter->second.voxel_.observed_) continue;

        const Voxel &voxel             = voxelIter->second.voxel_;
        const Eigen::Vector3d centroid = voxel.pointSum_ / static_cast<double>(voxel.numPoints_);
        const double distance          = (worldPoint - centroid).squaredNorm();
        if (!std::isfinite(distance)) continue;
        if (distance <= minimumDistance)
        {
            minimumDistance = distance;
            closest         = &voxel;
        }
    }
    return closest;
}

/** @brief 返回当前地图驻留体素数。 */
std::size_t Map::Size() const
{
    return voxels_.size();
}

/** @brief 返回当前驻留体素累计统计点数，不包含已淘汰体素。 */
std::size_t Map::PointCount() const
{
    std::size_t count = 0;
    for (const auto &entry : voxels_)
    {
        if (entry.second.voxel_.numPoints_ > std::numeric_limits<std::size_t>::max() - count)
        {
            throw std::overflow_error("BIEVR resident point count overflow");
        }
        count += entry.second.voxel_.numPoints_;
    }
    return count;
}

/** @brief 按 LRU 新到旧、图像行列顺序返回有效原始像素的世界坐标。 */
std::vector<Eigen::Vector3d> Map::Snapshot() const
{
    std::vector<Eigen::Vector3d> points;
    for (const std::uint64_t mortonKey : lru_)
    {
        const auto voxelIter = voxels_.find(mortonKey);
        if (voxelIter == voxels_.end()) continue;
        const Voxel &voxel = voxelIter->second.voxel_;
        for (int y = 0; y < voxel.image_.rows(); ++y)
        {
            for (int x = 0; x < voxel.image_.cols(); ++x)
            {
                if (voxel.weights_(y, x) <= 0.0f) continue;
                const Eigen::Vector3d cameraPoint(static_cast<double>(x) * config_.pixelSize_,
                                                  static_cast<double>(y) * config_.pixelSize_, voxel.image_(y, x));
                points.push_back(voxel.cameraFromWorld_.inverse() * cameraPoint);
            }
        }
    }
    return points;
}

}  // namespace pv_lio_plus::bievr_native
