/**
 * @file weight_model_tests.cpp
 * @brief Geman-McClure 归一化权重的范围、极值和参数验证。
 */
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "observation/measurement.h"
#include "observation/update_guard.h"
#include "observation/weight_config.h"

namespace
{
using pv_lio_plus::observation::GemanMcClureWeight;

/** RunGuardedUpdate 所需的最小状态和协方差测试替身。 */
struct FakeFilter
{
    Eigen::Vector2d state_      = Eigen::Vector2d(1.0, -2.0);
    Eigen::Matrix2d covariance_ = Eigen::Matrix2d::Identity();

    Eigen::Vector2d get_x() const
    {
        return state_;
    }
    Eigen::Matrix2d get_P() const
    {
        return covariance_;
    }
    void change_x(const Eigen::Vector2d& state)
    {
        state_ = state;
    }
    void change_P(const Eigen::Matrix2d& covariance)
    {
        covariance_ = covariance;
    }
};

TEST(GemanMcClureWeight, MatchesNormalizedModelAndIgnoresResidualSign)
{
    EXPECT_DOUBLE_EQ(GemanMcClureWeight(0.0, 2.0), 1.0);
    EXPECT_DOUBLE_EQ(GemanMcClureWeight(2.0, 2.0), 0.25);
    EXPECT_DOUBLE_EQ(GemanMcClureWeight(-2.0, 2.0), 0.25);
    EXPECT_NEAR(GemanMcClureWeight(4.0, 2.0), 0.04, 1e-15);
}

TEST(GemanMcClureWeight, AvoidsIntermediateSquareOverflowAndAllowsUnderflowToZero)
{
    const double max = std::numeric_limits<double>::max();
    EXPECT_DOUBLE_EQ(GemanMcClureWeight(1e200, 1e200), 0.25);
    EXPECT_DOUBLE_EQ(GemanMcClureWeight(max, max), 0.25);
    EXPECT_DOUBLE_EQ(GemanMcClureWeight(max, 1.0), 0.0);
    EXPECT_TRUE(std::isfinite(GemanMcClureWeight(max, 1.0)));
}

TEST(GemanMcClureWeight, RejectsNonfiniteResidualsAndNonpositiveOrNonfiniteScales)
{
    const double nan      = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    EXPECT_THROW(GemanMcClureWeight(nan, 1.0), std::invalid_argument);
    EXPECT_THROW(GemanMcClureWeight(infinity, 1.0), std::invalid_argument);
    EXPECT_THROW(GemanMcClureWeight(1.0, nan), std::invalid_argument);
    EXPECT_THROW(GemanMcClureWeight(1.0, infinity), std::invalid_argument);
    EXPECT_THROW(GemanMcClureWeight(1.0, 0.0), std::invalid_argument);
    EXPECT_THROW(GemanMcClureWeight(1.0, -1.0), std::invalid_argument);
}

TEST(ObservationUpdateGuard, RestoresPredictionAfterStickyRejection)
{
    FakeFilter filter;
    const Eigen::Vector2d predictedState      = filter.get_x();
    const Eigen::Matrix2d predictedCovariance = filter.get_P();
    bool isRejected                           = true;
    int validCallbackCount                    = 0;

    const bool isConverged = pv_lio_plus::observation::RunGuardedUpdate(filter, isRejected, [&]() {
        auto processCallback = [&](bool isValid) {
            if (!isValid)
            {
                isRejected = true;
                return false;
            }
            filter.change_x(filter.get_x() + Eigen::Vector2d(0.5, -0.25));
            filter.change_P(2.0 * filter.get_P());
            ++validCallbackCount;
            return true;
        };

        EXPECT_TRUE(processCallback(true));
        EXPECT_FALSE(processCallback(false));
        EXPECT_TRUE(processCallback(true));  // 后续有效行不清除此前拒绝标志。
        return true;
    });

    EXPECT_FALSE(isConverged);
    EXPECT_TRUE(isRejected);
    EXPECT_EQ(validCallbackCount, 2);
    EXPECT_TRUE(filter.get_x().isApprox(predictedState));
    EXPECT_TRUE(filter.get_P().isApprox(predictedCovariance));
}

TEST(ObservationUpdateGuard, KeepsStateAndCovarianceOnValidPath)
{
    FakeFilter filter;
    bool isRejected        = false;
    const bool isConverged = pv_lio_plus::observation::RunGuardedUpdate(filter, isRejected, [&]() {
        filter.change_x(filter.get_x() + Eigen::Vector2d(0.5, 0.25));
        filter.change_P(3.0 * filter.get_P());
        return true;
    });

    EXPECT_TRUE(isConverged);
    EXPECT_FALSE(isRejected);
    EXPECT_TRUE(filter.get_x().isApprox(Eigen::Vector2d(1.5, -1.75)));
    EXPECT_TRUE(filter.get_P().isApprox(3.0 * Eigen::Matrix2d::Identity()));
}
/** @brief 生产行校验覆盖求解器边界及维度和信息失效。 */
TEST(ObservationUpdateGuard, ValidatesZeroTwentyTwoTwentyThreeAndTwentyFourRows)
{
    /** @brief 与生产动态观测相同字段的测试记录。 */
    struct Measurement
    {
        bool valid = true;
        Eigen::MatrixXd h_x;
        Eigen::VectorXd h;
        Eigen::MatrixXd R;
    };
    for (const int count : {0, 22, 23, 24})
    {
        Measurement data;
        data.h_x = Eigen::MatrixXd::Ones(count, 12);
        data.h   = Eigen::VectorXd::Zero(count);
        data.R   = Eigen::MatrixXd::Ones(count, 1);
        EXPECT_EQ(pv_lio_plus::observation::ValidScalarRows(data, 23), count >= 23);
        if (count < 23) continue;
        data.R(0) = 0.0;
        EXPECT_FALSE(pv_lio_plus::observation::ValidScalarRows(data, 23));
        data.R(0)      = 1.0;
        data.h_x(0, 0) = std::numeric_limits<double>::quiet_NaN();
        EXPECT_FALSE(pv_lio_plus::observation::ValidScalarRows(data, 23));
        data.h_x = Eigen::MatrixXd::Ones(count - 1, 12);
        EXPECT_FALSE(pv_lio_plus::observation::ValidScalarRows(data, 23));
    }
}

/** @brief 验证真实启动加载器使用的组合规则及关闭行为。 */
TEST(ObservationWeightConfig, AcceptsOnlyDeclaredBackendCombinations)
{
    using pv_lio_plus::observation::ValidateWeightConfig;
    for (const auto* backend : {"ikdtree", "ivox", "i_octree", "ig_lio_plane"})
        EXPECT_NO_THROW(ValidateWeightConfig("geman_mcclure", backend, 0.1));
    for (const auto* backend : {"voxelmap", "voxelmap_plus", "c3p_voxelmap", "octvox", "r_voxelmap", "hvox", "bievr"})
        EXPECT_THROW(ValidateWeightConfig("geman_mcclure", backend, 0.1), std::invalid_argument);
    EXPECT_NO_THROW(ValidateWeightConfig("backend_default", "bievr", 0.1));
    EXPECT_THROW(ValidateWeightConfig("typo", "ivox", 0.1), std::invalid_argument);
    for (const double scale :
         {0.0, -0.1, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
        EXPECT_THROW(ValidateWeightConfig("geman_mcclure", "ivox", scale), std::invalid_argument);
}
}  // namespace

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
