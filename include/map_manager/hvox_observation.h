/* Copyright (C) 2026 PV-LIO-PLUS contributors. SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <vector>

#include "map_manager/native/hvox/ProbabilisticKernelOptimizer.h"

namespace pv_lio_plus
{
struct HVoxCorrespondence
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d point_lidar = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d center_world = Eigen::Vector3d::Zero();
};
using HVoxCorrespondences = std::vector<HVoxCorrespondence, Eigen::aligned_allocator<HVoxCorrespondence>>;

struct HVoxObservationRow
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<double, 1, 12> jacobian;
    double residual = 0.0;
    double information = 0.0;
};
using HVoxObservationRows = std::vector<HVoxObservationRow, Eigen::aligned_allocator<HVoxObservationRow>>;

/** One fixed-correspondence PKO group inside a shared IKFoM frame. */
class HVoxObservationGroup
{
   public:
    HVoxObservationGroup();
    void BeginFrame();
    bool NeedsRefresh(bool converged, std::uint64_t map_revision) const;
    bool Refresh(const HVoxCorrespondences &matches, const Eigen::Quaterniond &world_from_imu,
                 const Eigen::Quaterniond &imu_from_lidar, const Eigen::Vector3d &lidar_origin_in_imu,
                 const Eigen::Vector3d &imu_position_world, std::uint64_t map_revision);
    bool Evaluate(const Eigen::Quaterniond &world_from_imu, const Eigen::Quaterniond &imu_from_lidar,
                  const Eigen::Vector3d &lidar_origin_in_imu, const Eigen::Vector3d &imu_position_world,
                  bool estimate_extrinsics, double lidar_noise_std, HVoxObservationRows &rows);
    double scale() const { return scale_; }
    double alpha() const { return alpha_; }
    std::size_t size() const { return matches_.size(); }

   private:
    HVoxCorrespondences matches_;
    lio::ProbabilisticKernelOptimizer pko_;
    std::uint64_t map_revision_ = 0;
    double scale_ = 1e-6;
    double alpha_ = 0.0;
    bool valid_ = false;
};
}  // namespace pv_lio_plus
