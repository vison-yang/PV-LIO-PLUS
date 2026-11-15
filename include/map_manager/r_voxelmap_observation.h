/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef PV_LIO_PLUS_R_VOXELMAP_OBSERVATION_H
#define PV_LIO_PLUS_R_VOXELMAP_OBSERVATION_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include "map_manager/point_plane_jacobian.h"

namespace pv_lio_plus
{
struct RVoxelMapMeasurement
{
    Eigen::Matrix<double, 1, 12> jacobian;
    double residual;
    double information;
};

// Mirrors the upstream consumer's plane-uncertainty plus rotated sensor
// covariance. State uncertainty belongs to the match gate, not this noise.
inline bool MakeRVoxelMapMeasurement(const Eigen::Vector3d &point_lidar, const Eigen::Vector3d &point_world,
                                     const Eigen::Vector3d &normal, const Eigen::Vector3d &center, double d,
                                     const Eigen::Matrix<double, 6, 6> &plane_cov, const Eigen::Matrix3d &lidar_cov,
                                     const Eigen::Quaterniond &world_from_imu, const Eigen::Quaterniond &imu_from_lidar,
                                     const Eigen::Vector3d &lidar_origin_in_imu, bool estimate_extrinsics,
                                     RVoxelMapMeasurement &out)
{
    if (!point_lidar.allFinite() || !point_world.allFinite() || !normal.allFinite() || !center.allFinite() ||
        !plane_cov.allFinite() || !lidar_cov.allFinite() || !std::isfinite(d) ||
        !world_from_imu.coeffs().allFinite() || !imu_from_lidar.coeffs().allFinite() ||
        !lidar_origin_in_imu.allFinite()) return false;

    Eigen::Matrix<double, 1, 6> jacobian_plane;
    jacobian_plane << (point_world - center).transpose(), -normal.transpose();
    const Eigen::Matrix3d world_from_lidar = (world_from_imu * imu_from_lidar).toRotationMatrix();
    const Eigen::Matrix3d sensor_cov_world = world_from_lidar * lidar_cov * world_from_lidar.transpose();
    const double variance = (jacobian_plane * plane_cov * jacobian_plane.transpose())(0, 0) +
                            normal.dot(sensor_cov_world * normal);
    const double signed_distance = normal.dot(point_world) + d;
    if (!std::isfinite(variance) || variance <= 0.0 || !std::isfinite(signed_distance)) return false;
    out.jacobian = VoxelMapPlusPointPlaneJacobian(point_lidar, normal, world_from_imu, imu_from_lidar,
                                                  lidar_origin_in_imu, estimate_extrinsics);
    out.residual = -signed_distance;
    out.information = 1.0 / variance;
    return out.jacobian.allFinite() && std::isfinite(out.information);
}
}  // namespace pv_lio_plus
#endif
