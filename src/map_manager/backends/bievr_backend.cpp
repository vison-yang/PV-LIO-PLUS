/** @file bievr_backend.cpp @brief 将公共世界点与 IMU 系量程绑定后送入高度图。 */
#include "bievr_backend.h"

#include <cmath>
#include <stdexcept>

namespace pv_lio_plus::backend_bievr
{
/** @brief 校验公共约束；高度噪声是本地滤波尺度，并非作者像素融合权重。 */
bievr_native::Config MakeConfig(const MapManagerConfig &config)
{
    if (config.local_window_enabled)
        throw std::invalid_argument("BIEVR uses native LRU; common local_window is unsupported");
    if (config.bievrCapacity_ <= 0 || config.bievrMaxImagePixels_ <= 0 ||
        !ValidNearestMaxRange(config.bievrHeightSigmaM_) ||
        !std::isfinite(1.0 / (config.bievrHeightSigmaM_ * config.bievrHeightSigmaM_)) ||
        !std::isfinite(config.bievrHuberDeltaM_) || config.bievrHuberDeltaM_ <= 0.0)
        throw std::invalid_argument("invalid BIEVR capacity, image limit, height sigma or Huber scale");
    bievr_native::Config native;
    native.voxelSize_          = config.bievrVoxelSize_;
    native.pixelSize_          = config.bievrPixelSize_;
    native.capacity_           = static_cast<std::size_t>(config.bievrCapacity_);
    native.maxImagePixels_     = static_cast<std::size_t>(config.bievrMaxImagePixels_);
    native.weighted_           = config.bievrWeighted_;
    native.smooth_             = config.bievrSmooth_;
    native.normalToleranceDeg_ = config.bievrNormalToleranceDeg_;
    return native;
}

/** @brief 全批绑定量程；外参或点非法时，尚未调用会修改地图的积分函数。 */
void Update(bievr_native::Map &map, const MapPointList &points, const MapUpdateContext &context)
{
    if (!context.rotImuLidar_ || !context.transImuLidar_ || !context.rotImuLidar_->allFinite() ||
        !context.transImuLidar_->allFinite() ||
        !(context.rotImuLidar_->transpose() * *context.rotImuLidar_).isApprox(M3D::Identity(), 1e-8) ||
        std::abs(context.rotImuLidar_->determinant() - 1.0) > 1e-8)
        throw std::invalid_argument("BIEVR update requires finite rigid LiDAR-to-IMU extrinsics");
    std::vector<Eigen::Vector3d> world;
    std::vector<double> ranges;
    world.reserve(points.size());
    ranges.reserve(points.size());
    for (const auto &point : points)
    {
        const double range = (*context.rotImuLidar_ * point.point_lidar + *context.transImuLidar_).norm();
        if (!point.point_lidar.allFinite() || !point.point_world.allFinite() || !std::isfinite(range) || range <= 0.0)
            throw std::invalid_argument("BIEVR update requires finite world/LiDAR points and positive IMU ranges");
        world.push_back(point.point_world);
        ranges.push_back(range);
    }
    map.Integrate(world, ranges);
}

/** @brief 完整构建替换地图，调用者仅在成功后销毁旧图。 */
std::unique_ptr<bievr_native::Map> Build(const MapManagerConfig &config, const MapPointList &points,
                                         const MapUpdateContext &context)
{
    auto map = std::make_unique<bievr_native::Map>(MakeConfig(config));
    Update(*map, points, context);
    return map;
}

/** @brief 本格或近邻格选定后只采样该格；不跨格补造无效像素。 */
bool Sample(const bievr_native::Map &map, const MapManagerConfig &config, const V3D &pointWorld,
            observation::HeightSample &sample)
{
    const auto *voxel = map.FindVoxel(pointWorld);
    if (!voxel) return false;
    return observation::SampleBievrHeight(config.bievrSmooth_ ? voxel->smoothed_ : voxel->image_, voxel->weights_,
                                          voxel->cameraFromWorld_, config.bievrPixelSize_, pointWorld, sample);
}
}  // namespace pv_lio_plus::backend_bievr
