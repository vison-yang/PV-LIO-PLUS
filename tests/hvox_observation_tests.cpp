/* Copyright (C) 2026 PV-LIO-PLUS contributors. SPDX-License-Identifier: GPL-2.0-or-later */
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include "map_manager/hvox_observation.h"
#include "use-ikfom.hpp"

namespace {
using namespace pv_lio_plus;
using V3D = Eigen::Vector3d;
lio::PKOConfig AuthorConfig() {
    lio::PKOConfig config;
    config.use_adaptive = true; config.min_scale_factor = .001; config.max_scale_factor = 10.;
    config.num_alpha_segments = 25; config.truncated_threshold = 10.;
    config.gmm_components = 2; config.gmm_sample_size = 100;
    return config;
}
HVoxCorrespondence Correspondence(const V3D &point, const V3D &normal, const V3D &center) {
    HVoxCorrespondence c; c.point_lidar = point; c.normal_world = normal; c.center_world = center; return c;
}
bool Refresh(HVoxObservationGroup &group, const HVoxCorrespondences &matches, const state_ikfom &state,
             std::uint64_t revision = 7) {
    return group.Refresh(matches, state.rot, state.offset_R_L_I, state.offset_T_L_I, state.pos, revision);
}
bool Evaluate(HVoxObservationGroup &group, const state_ikfom &state, bool extrinsics, HVoxObservationRows &rows,
              double noise = .05) {
    return group.Evaluate(state.rot, state.offset_R_L_I, state.offset_T_L_I, state.pos, extrinsics, noise, rows);
}
double SignedDistance(const state_ikfom &s, const HVoxCorrespondence &c) {
    return c.normal_world.dot(s.rot * (s.offset_R_L_I * c.point_lidar + s.offset_T_L_I) + s.pos - c.center_world);
}

TEST(HVoxObservation, RightPerturbationFiniteDifferencesAndExtrinsicSwitch) {
    state_ikfom state;
    state.pos = V3D(1.7,-2.5,.9);
    state.rot = Eigen::Quaterniond(Eigen::AngleAxisd(.43, V3D(.2,.7,-.5).normalized()));
    state.offset_R_L_I = Eigen::Quaterniond(Eigen::AngleAxisd(-.63, V3D(-.6,.3,.8).normalized()));
    state.offset_T_L_I = V3D(.24,-.37,.51);
    const HVoxCorrespondence match = Correspondence(V3D(2.3,-1.1,.8), V3D(.4,-.3,.8).normalized(), V3D(.2,-.9,1.));
    HVoxObservationGroup group;
    ASSERT_TRUE(Refresh(group, {match}, state));
    HVoxObservationRows enabled, disabled;
    ASSERT_TRUE(Evaluate(group, state, true, enabled));
    ASSERT_TRUE(Evaluate(group, state, false, disabled));
    ASSERT_EQ(enabled.size(), 1u);
    EXPECT_NEAR(enabled[0].residual, -SignedDistance(state, match), 1e-12);
    EXPECT_EQ(disabled[0].jacobian.tail<6>().norm(), 0.0);
    constexpr double epsilon = 1e-6;
    for (int col=0; col<12; ++col) {
        Eigen::Matrix<double, state_ikfom::DOF, 1> delta = Eigen::Matrix<double, state_ikfom::DOF, 1>::Zero();
        delta(col) = epsilon;
        state_ikfom plus=state, minus=state;
        plus.boxplus(delta); minus.boxplus(delta, -1.0);
        const double numeric = (SignedDistance(plus,match)-SignedDistance(minus,match))/(2*epsilon);
        EXPECT_NEAR(enabled[0].jacobian(col), numeric, 2e-8) << col;
        if (col<6) { EXPECT_NEAR(disabled[0].jacobian(col), numeric, 2e-8) << col; }
    }
}

TEST(HVoxObservation, AuthorFixedSourceSignedPKOOracle) {
    // Values generated independently from upstream Surfel-LIO@73348c1 PKO.cpp.
    lio::ProbabilisticKernelOptimizer pko(AuthorConfig());
    std::vector<double> positive, negative;
    for (int i=0; i<137; ++i) {
        const double x = 2.0 + .6*std::sin(i*.37);
        positive.push_back(x); negative.push_back(-x);
    }
    EXPECT_NEAR(pko.CalculateScaleFactor(positive), 1.8245153251428796, 1e-12);
    EXPECT_NEAR(pko.CalculateScaleFactor(negative), .021428707896358711, 1e-12);
    pko.Reset();
    EXPECT_NEAR(pko.CalculateScaleFactor(negative), .021428707896358711, 1e-12);
    pko.Reset();
    EXPECT_NEAR(pko.CalculateScaleFactor(positive), 1.8245153251428796, 1e-12);
}

TEST(HVoxObservation, FixedScaleCurrentResidualWeightsAndGroupReset) {
    state_ikfom state;
    HVoxCorrespondences matches;
    std::vector<double> residuals;
    for (int i=0; i<137; ++i) {
        const double h=2.0+.6*std::sin(i*.37);
        matches.push_back(Correspondence(V3D(0,0,-h), V3D::UnitZ(), V3D::Zero()));
        residuals.push_back(h);
    }
    const double mean=std::accumulate(residuals.begin(),residuals.end(),0.0)/residuals.size();
    double variance=0;
    for (double h:residuals) variance+=(h-mean)*(h-mean);
    const double scale=std::sqrt(variance/residuals.size())/3.;
    std::vector<double> normalized;
    for (double h:residuals) normalized.push_back(h/scale);
    lio::ProbabilisticKernelOptimizer author(AuthorConfig());
    const double first_alpha=author.CalculateScaleFactor(normalized);
    HVoxObservationGroup group;
    EXPECT_TRUE(group.NeedsRefresh(false,7));
    ASSERT_TRUE(Refresh(group,matches,state));
    EXPECT_FALSE(group.NeedsRefresh(false,7));
    EXPECT_TRUE(group.NeedsRefresh(true,7));
    EXPECT_TRUE(group.NeedsRefresh(false,8));
    EXPECT_NEAR(group.scale(),scale,1e-12);
    HVoxObservationRows rows;
    ASSERT_TRUE(Evaluate(group,state,true,rows));
    ASSERT_EQ(rows.size(),matches.size());
    EXPECT_NEAR(group.alpha(),first_alpha,1e-12);
    for (std::size_t i=0; i<rows.size(); ++i) {
        const double huber = std::min(1.0, first_alpha/std::abs(normalized[i]));
        EXPECT_NEAR(rows[i].residual,residuals[i],1e-12);
        EXPECT_NEAR(rows[i].information,huber/(.001+.05*.05),1e-10);
    }
    state.pos.z() += .1;
    const double unchanged_scale=group.scale();
    ASSERT_TRUE(Evaluate(group,state,true,rows));
    EXPECT_DOUBLE_EQ(group.scale(),unchanged_scale);
    EXPECT_NEAR(rows[0].residual,residuals[0]-.1,1e-12);  // cached plane, current state
    group.BeginFrame();
    EXPECT_TRUE(group.NeedsRefresh(false,7));
    EXPECT_FALSE(Evaluate(group,state,true,rows));
    state.pos.z() -= .1;
    ASSERT_TRUE(Refresh(group,matches,state));  // resets group graduation, retains candidate tables
    ASSERT_TRUE(Evaluate(group,state,true,rows));
    EXPECT_NEAR(group.alpha(),first_alpha,1e-12);
}

TEST(HVoxObservation, EmptyZeroConstantSingleAndNonfiniteAreExplicit) {
    state_ikfom state;
    HVoxObservationGroup group;
    HVoxObservationRows rows;
    EXPECT_FALSE(Refresh(group,{},state));
    const auto zero=Correspondence(V3D::Zero(),V3D::UnitZ(),V3D::Zero());
    ASSERT_TRUE(Refresh(group,{zero},state));
    ASSERT_DOUBLE_EQ(group.scale(),1e-6);
    ASSERT_TRUE(Evaluate(group,state,true,rows));
    ASSERT_EQ(rows.size(),1u);
    EXPECT_DOUBLE_EQ(rows[0].residual,0.0);
    EXPECT_NEAR(group.alpha(),.021428707896358711,1e-12);  // no hardcoded alpha=1
    EXPECT_NEAR(rows[0].information,1./(.001+.05*.05),1e-12);
    const auto constant=Correspondence(V3D(0,0,-2),V3D::UnitZ(),V3D::Zero());
    ASSERT_TRUE(Refresh(group,{constant,constant},state));
    EXPECT_DOUBLE_EQ(group.scale(),1e-6);
    ASSERT_TRUE(Evaluate(group,state,true,rows));
    const double nan=std::numeric_limits<double>::quiet_NaN();
    auto bad=zero; bad.point_lidar.x()=nan;
    EXPECT_FALSE(Refresh(group,{bad},state));
    EXPECT_TRUE(group.NeedsRefresh(false,7));
    ASSERT_TRUE(Refresh(group,{zero},state));
    EXPECT_FALSE(Evaluate(group,state,true,rows,std::numeric_limits<double>::infinity()));
    EXPECT_TRUE(rows.empty());
    ASSERT_TRUE(Refresh(group,{zero},state));
    state.pos.x()=nan;
    EXPECT_FALSE(Evaluate(group,state,true,rows));
    EXPECT_TRUE(group.NeedsRefresh(false,7));
}
}  // namespace

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
