/* Copyright (C) 2026 PV-LIO-PLUS contributors. SPDX-License-Identifier: GPL-2.0-or-later */
#include "map_manager/hvox_observation.h"
#include "map_manager/point_plane_jacobian.h"

#include <algorithm>
#include <cmath>

namespace pv_lio_plus
{
namespace
{
lio::PKOConfig AuthorEstimatorConfig()
{
    lio::PKOConfig config;
    config.use_adaptive = true;
    config.min_scale_factor = 0.001;
    config.max_scale_factor = 10.0;
    config.num_alpha_segments = 25;
    config.truncated_threshold = 10.0;
    config.gmm_components = 2;
    config.gmm_sample_size = 100;
    return config;
}

bool Residuals(const HVoxCorrespondences &matches, const Eigen::Quaterniond &world_from_imu,
               const Eigen::Quaterniond &imu_from_lidar, const Eigen::Vector3d &lidar_origin_in_imu,
               const Eigen::Vector3d &imu_position_world, std::vector<double> &residuals)
{
    residuals.clear();
    if (!world_from_imu.coeffs().allFinite() || !imu_from_lidar.coeffs().allFinite() ||
        !lidar_origin_in_imu.allFinite() || !imu_position_world.allFinite()) return false;
    residuals.reserve(matches.size());
    for (const auto &match : matches)
    {
        if (!match.point_lidar.allFinite() || !match.normal_world.allFinite() || !match.center_world.allFinite() ||
            match.normal_world.squaredNorm() < 1e-12) return false;
        const Eigen::Vector3d world = world_from_imu * (imu_from_lidar * match.point_lidar + lidar_origin_in_imu) +
                                      imu_position_world;
        const double h = -match.normal_world.dot(world - match.center_world);
        if (!world.allFinite() || !std::isfinite(h)) return false;
        residuals.push_back(h);
    }
    return true;
}
}  // namespace

HVoxObservationGroup::HVoxObservationGroup() : pko_(AuthorEstimatorConfig()) {}

void HVoxObservationGroup::BeginFrame()
{
    valid_ = false;
    matches_.clear();
}

bool HVoxObservationGroup::NeedsRefresh(bool converged, std::uint64_t map_revision) const
{
    return !valid_ || converged || map_revision != map_revision_;
}

bool HVoxObservationGroup::Refresh(const HVoxCorrespondences &matches, const Eigen::Quaterniond &world_from_imu,
                                   const Eigen::Quaterniond &imu_from_lidar,
                                   const Eigen::Vector3d &lidar_origin_in_imu,
                                   const Eigen::Vector3d &imu_position_world, std::uint64_t map_revision)
{
    BeginFrame();
    std::vector<double> residuals;
    if (matches.empty() || !Residuals(matches, world_from_imu, imu_from_lidar, lidar_origin_in_imu,
                                      imu_position_world, residuals)) return false;

    // Population standard deviation, evaluated only when correspondences refresh.
    double mean = 0.0;
    double m2 = 0.0;
    std::size_t count = 0;
    for (double value : residuals)
    {
        ++count;
        const double delta = value - mean;
        mean += delta / static_cast<double>(count);
        m2 += delta * (value - mean);
        if (!std::isfinite(mean) || !std::isfinite(m2)) return false;
    }
    const double variance = m2 / static_cast<double>(count);
    if (!std::isfinite(variance) || variance < 0.0) return false;
    scale_ = std::max(std::sqrt(variance) / 3.0, 1e-6);
    if (!std::isfinite(scale_)) return false;
    matches_ = matches;
    map_revision_ = map_revision;
    pko_.Reset();
    alpha_ = 0.0;
    valid_ = true;
    return true;
}

bool HVoxObservationGroup::Evaluate(const Eigen::Quaterniond &world_from_imu,
                                    const Eigen::Quaterniond &imu_from_lidar,
                                    const Eigen::Vector3d &lidar_origin_in_imu,
                                    const Eigen::Vector3d &imu_position_world, bool estimate_extrinsics,
                                    double lidar_noise_std, HVoxObservationRows &rows)
{
    rows.clear();
    std::vector<double> residuals;
    const double variance = 0.001 + lidar_noise_std * lidar_noise_std;
    if (!valid_ || !std::isfinite(lidar_noise_std) || lidar_noise_std < 0.0 || !std::isfinite(variance) ||
        !std::isfinite(scale_) || scale_ <= 0.0 ||
        !Residuals(matches_, world_from_imu, imu_from_lidar, lidar_origin_in_imu, imu_position_world, residuals))
    {
        BeginFrame();
        return false;
    }

    std::vector<double> normalized;
    normalized.reserve(residuals.size());
    for (double residual : residuals)
    {
        const double value = residual / scale_;
        if (!std::isfinite(value)) { BeginFrame(); return false; }
        normalized.push_back(value);
    }
    alpha_ = pko_.CalculateScaleFactor(normalized);
    if (!std::isfinite(alpha_) || alpha_ <= 0.0) { BeginFrame(); return false; }

    rows.reserve(matches_.size());
    for (std::size_t i = 0; i < matches_.size(); ++i)
    {
        HVoxObservationRow row;
        row.residual = residuals[i];
        row.jacobian = VoxelMapPlusPointPlaneJacobian(matches_[i].point_lidar, matches_[i].normal_world,
                                                       world_from_imu, imu_from_lidar, lidar_origin_in_imu,
                                                       estimate_extrinsics);
        const double magnitude = std::abs(normalized[i]);
        const double huber = magnitude <= alpha_ ? 1.0 : alpha_ / magnitude;
        row.information = huber / variance;
        if (!row.jacobian.allFinite() || !std::isfinite(row.information))
        {
            rows.clear();
            BeginFrame();
            return false;
        }
        rows.push_back(row);
    }
    return true;
}
}  // namespace pv_lio_plus
