/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <array>

#include "map_manager/point_plane_jacobian.h"
#include "use-ikfom.hpp"

namespace
{
using Row12 = Eigen::Matrix<double, 1, 12>;

double PlaneDistance(const state_ikfom &state, const Eigen::Vector3d &point_lidar,
                     const Eigen::Vector3d &unit_normal, double plane_offset)
{
    const Eigen::Vector3d world_point =
        state.rot * (state.offset_R_L_I * point_lidar + state.offset_T_L_I) + state.pos;
    return unit_normal.dot(world_point) + plane_offset;
}

Row12 EvaluateJacobian(const state_ikfom &state, const Eigen::Vector3d &point_lidar,
                       const Eigen::Vector3d &unit_normal, bool estimate_extrinsics)
{
    return pv_lio_plus::VoxelMapPlusPointPlaneJacobian(
        point_lidar, unit_normal, state.rot, state.offset_R_L_I, state.offset_T_L_I, estimate_extrinsics);
}

TEST(VoxelMapPlusObservation, RightPerturbationFiniteDifferencesAndExtrinsicSwitch)
{
    // Distinct, nonidentity poses and non-axis-aligned planes expose frame,
    // rotation-side, sign, and translation-coordinate errors.
    const std::array<Eigen::Vector3d, 3> points = {
        Eigen::Vector3d(2.3, -1.1, 0.8), Eigen::Vector3d(-0.6, 3.4, 1.7), Eigen::Vector3d(1.2, 0.7, -2.8)};
    const std::array<Eigen::Vector3d, 3> normals = {
        Eigen::Vector3d(0.4, -0.3, 0.8).normalized(), Eigen::Vector3d(-0.7, 0.5, 0.2).normalized(),
        Eigen::Vector3d(0.1, 0.9, -0.4).normalized()};
    for (std::size_t sample = 0; sample < points.size(); ++sample)
    {
        SCOPED_TRACE(sample);
        state_ikfom state;
        state.pos = Eigen::Vector3d(1.7, -2.5, 0.9);
        state.rot = Eigen::Quaterniond(Eigen::AngleAxisd(0.43 + 0.2 * sample,
                                                          Eigen::Vector3d(0.2, 0.7, -0.5).normalized()));
        state.offset_R_L_I = Eigen::Quaterniond(Eigen::AngleAxisd(-0.63 + 0.1 * sample,
                                                                   Eigen::Vector3d(-0.6, 0.3, 0.8).normalized()));
        state.offset_T_L_I = Eigen::Vector3d(0.24, -0.37, 0.51);
        const Row12 enabled = EvaluateJacobian(state, points[sample], normals[sample], true);
        const Row12 disabled = EvaluateJacobian(state, points[sample], normals[sample], false);
        EXPECT_LT((enabled.head<6>() - disabled.head<6>()).norm(), 1e-14);
        EXPECT_DOUBLE_EQ(disabled.tail<6>().norm(), 0.0);
        EXPECT_GT(enabled.segment<3>(6).norm(), 0.01);
        EXPECT_GT(enabled.segment<3>(9).norm(), 0.99);

        constexpr double epsilon = 1e-6;
        for (int column = 0; column < 12; ++column)
        {
            Eigen::Matrix<double, state_ikfom::DOF, 1> delta =
                Eigen::Matrix<double, state_ikfom::DOF, 1>::Zero();
            delta(column) = epsilon;
            state_ikfom plus = state;
            plus.boxplus(delta);
            state_ikfom minus = state;
            minus.boxplus(delta, -1.0);
            const double numeric = (PlaneDistance(plus, points[sample], normals[sample], -0.8) -
                                    PlaneDistance(minus, points[sample], normals[sample], -0.8)) /
                                   (2.0 * epsilon);
            EXPECT_NEAR(enabled(column), numeric, 2e-8) << "column " << column;
            if (column < 6)
            {
                EXPECT_NEAR(disabled(column), numeric, 2e-8) << "column " << column;
            }
        }
    }
}
}  // namespace

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
