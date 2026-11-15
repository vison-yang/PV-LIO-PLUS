/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef PV_LIO_PLUS_POINT_PLANE_JACOBIAN_H
#define PV_LIO_PLUS_POINT_PLANE_JACOBIAN_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace pv_lio_plus
{
/**
 * Jacobian of a fixed world-frame plane's signed distance, in the IKFoM
 * position, right-perturbed IMU rotation, right-perturbed LiDAR-to-IMU
 * rotation, and IMU-frame LiDAR translation coordinates, respectively.
 * The input point is in the LiDAR frame and normal_world is a unit vector.
 */
inline Eigen::Matrix<double, 1, 12> VoxelMapPlusPointPlaneJacobian(
    const Eigen::Vector3d &point_lidar, const Eigen::Vector3d &normal_world,
    const Eigen::Quaterniond &world_from_imu, const Eigen::Quaterniond &imu_from_lidar,
    const Eigen::Vector3d &lidar_origin_in_imu, bool estimate_extrinsics)
{
    const Eigen::Vector3d normal_imu = world_from_imu.conjugate() * normal_world;
    const Eigen::Vector3d point_imu = imu_from_lidar * point_lidar + lidar_origin_in_imu;
    Eigen::Matrix<double, 1, 12> jacobian = Eigen::Matrix<double, 1, 12>::Zero();
    jacobian.segment<3>(0) = normal_world.transpose();
    jacobian.segment<3>(3) = point_imu.cross(normal_imu).transpose();
    if (estimate_extrinsics)
    {
        jacobian.segment<3>(6) = point_lidar.cross(imu_from_lidar.conjugate() * normal_imu).transpose();
        jacobian.segment<3>(9) = normal_imu.transpose();
    }
    return jacobian;
}
}  // namespace pv_lio_plus

#endif  // PV_LIO_PLUS_POINT_PLANE_JACOBIAN_H
