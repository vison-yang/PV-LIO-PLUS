/**
 * @file bievr_observation_tests.cpp
 * @brief BIEVR 掩码双线性高度采样与共同状态观测行的独立数值验收。
 */
#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <limits>

#include "observation/measurement.h"
#include "use-ikfom.hpp"

namespace
{
using pv_lio_plus::observation::EvaluateBievrObservation;
using pv_lio_plus::observation::HeightSample;
using pv_lio_plus::observation::ObservationRow;
using pv_lio_plus::observation::SampleBievrHeight;

/** 生成供 oracle 独立采样的仿射或二次高度图。 */
double HeightField(double u, double v, bool nonAffine)
{
    const double affine = 0.17 + 0.12 * u - 0.08 * v;
    return nonAffine ? affine + 0.004 * u * u - 0.003 * u * v + 0.002 * v * v : affine;
}

/** 用测试侧定义的插值公式计算连续高度，不调用生产采样函数。 */
bool OracleHeight(const Eigen::MatrixXf& image, const Eigen::MatrixXf& weights, double u, double v, double& height)
{
    if (image.rows() != weights.rows() || image.cols() != weights.cols() || image.rows() < 2 || image.cols() < 2 ||
        !std::isfinite(u) || !std::isfinite(v) || u < 0.0 || v < 0.0 || u >= static_cast<double>(image.cols() - 1) ||
        v >= static_cast<double>(image.rows() - 1))
    {
        return false;
    }

    const Eigen::Index x0                  = static_cast<Eigen::Index>(std::floor(u));
    const Eigen::Index y0                  = static_cast<Eigen::Index>(std::floor(v));
    const double x                         = u - static_cast<double>(x0);
    const double y                         = v - static_cast<double>(y0);
    const std::array<Eigen::Index, 4> rows = {y0, y0, y0 + 1, y0 + 1};
    const std::array<Eigen::Index, 4> cols = {x0, x0 + 1, x0, x0 + 1};
    const std::array<double, 4> terms      = {(1.0 - x) * (1.0 - y), x * (1.0 - y), (1.0 - x) * y, x * y};
    double weightedHeight                  = 0.0;
    double total                           = 0.0;
    for (std::size_t i = 0; i < terms.size(); ++i)
    {
        if (weights(rows[i], cols[i]) > 0.0F)
        {
            weightedHeight += terms[i] * static_cast<double>(image(rows[i], cols[i]));
            total += terms[i];
        }
    }
    if (total <= 0.0)
    {
        return false;
    }
    height = weightedHeight / total;
    return std::isfinite(height);
}

/** 构造与 HeightField 一致的像素数据，权重默认全有效。 */
void MakeImage(Eigen::MatrixXf& image, Eigen::MatrixXf& weights, bool nonAffine)
{
    image.resize(48, 52);
    weights = Eigen::MatrixXf::Ones(48, 52);
    for (Eigen::Index row = 0; row < image.rows(); ++row)
    {
        for (Eigen::Index col = 0; col < image.cols(); ++col)
        {
            image(row, col) =
                static_cast<float>(HeightField(static_cast<double>(col), static_cast<double>(row), nonAffine));
        }
    }
}

/** 以测试 oracle 计算状态扰动后的高度残差。 */
bool OracleResidual(const state_ikfom& state, const Eigen::Vector3d& pointLidar,
                    const Eigen::Isometry3d& cameraFromWorld, double pixelSize, const Eigen::MatrixXf& image,
                    const Eigen::MatrixXf& weights, double& residual)
{
    const Eigen::Vector3d pointImu    = state.offset_R_L_I.toRotationMatrix() * pointLidar + state.offset_T_L_I;
    const Eigen::Vector3d pointWorld  = state.rot.toRotationMatrix() * pointImu + state.pos;
    const Eigen::Vector3d pointCamera = cameraFromWorld * pointWorld;
    double height                     = 0.0;
    if (!OracleHeight(image, weights, pointCamera.x() / pixelSize, pointCamera.y() / pixelSize, height))
    {
        return false;
    }
    residual = pointCamera.z() - height;
    return std::isfinite(residual);
}

/** 在给定相机系坐标附近构造非单位视姿下的世界系测试点。 */
Eigen::Vector3d WorldPointAtPixel(const Eigen::Isometry3d& cameraFromWorld, double u, double v, double z)
{
    return cameraFromWorld.inverse() * Eigen::Vector3d(u, v, z);
}

TEST(BievrHeightSample, AffineAndMaskedNonAffineValuesAndGradients)
{
    Eigen::MatrixXf image;
    Eigen::MatrixXf weights;
    MakeImage(image, weights, false);
    Eigen::Isometry3d cameraFromWorld = Eigen::Isometry3d::Identity();
    cameraFromWorld.linear() = Eigen::AngleAxisd(0.31, Eigen::Vector3d(0.3, -0.4, 0.8).normalized()).toRotationMatrix();
    cameraFromWorld.translation()    = Eigen::Vector3d(2.0, -1.0, 0.7);
    constexpr double pixelSize       = 0.5;
    const Eigen::Vector3d pointWorld = WorldPointAtPixel(cameraFromWorld, 17.37, 19.62, 2.8);

    HeightSample sample;
    ASSERT_TRUE(SampleBievrHeight(image, weights, cameraFromWorld, pixelSize, pointWorld, sample));
    double oracleHeight = 0.0;
    ASSERT_TRUE(OracleHeight(image, weights, sample.pixelU_, sample.pixelV_, oracleHeight));
    EXPECT_NEAR(sample.height_, oracleHeight, 2e-7);
    EXPECT_NEAR(sample.residual_, (cameraFromWorld * pointWorld).z() - oracleHeight, 2e-7);
    EXPECT_NEAR(sample.gradientWorld_.norm(), Eigen::Vector3d(-0.24, 0.16, 1.0).norm(), 2e-6);

    // 由独立高度 oracle 对世界坐标做中心差分，核对未归一化梯度的坐标变换。
    constexpr double epsilon = 1e-5;
    for (int axis = 0; axis < 3; ++axis)
    {
        Eigen::Vector3d plusPoint  = pointWorld;
        Eigen::Vector3d minusPoint = pointWorld;
        plusPoint(axis) += epsilon;
        minusPoint(axis) -= epsilon;
        const Eigen::Vector3d plusCamera  = cameraFromWorld * plusPoint;
        const Eigen::Vector3d minusCamera = cameraFromWorld * minusPoint;
        double plusHeight                 = 0.0;
        double minusHeight                = 0.0;
        ASSERT_TRUE(OracleHeight(image, weights, plusCamera.x() / pixelSize, plusCamera.y() / pixelSize, plusHeight));
        ASSERT_TRUE(
            OracleHeight(image, weights, minusCamera.x() / pixelSize, minusCamera.y() / pixelSize, minusHeight));
        const double numeric = ((plusCamera.z() - plusHeight) - (minusCamera.z() - minusHeight)) / (2.0 * epsilon);
        EXPECT_NEAR(sample.gradientWorld_(axis), numeric, 3e-6);
    }

    MakeImage(image, weights, true);
    const Eigen::Index maskedU = static_cast<Eigen::Index>(std::floor(sample.pixelU_));
    const Eigen::Index maskedV = static_cast<Eigen::Index>(std::floor(sample.pixelV_));
    weights(maskedV, maskedU)  = 0.0F;
    image(maskedV, maskedU)    = std::numeric_limits<float>::quiet_NaN();  // 无效角不参与值或导数。
    ASSERT_TRUE(SampleBievrHeight(image, weights, cameraFromWorld, pixelSize, pointWorld, sample));
    ASSERT_TRUE(OracleHeight(image, weights, sample.pixelU_, sample.pixelV_, oracleHeight));
    EXPECT_NEAR(sample.height_, oracleHeight, 2e-7);

    const Eigen::Vector3d plusWorld =
        pointWorld + cameraFromWorld.linear().transpose() * Eigen::Vector3d(epsilon, 0.0, 0.0);
    const Eigen::Vector3d minusWorld =
        pointWorld - cameraFromWorld.linear().transpose() * Eigen::Vector3d(epsilon, 0.0, 0.0);
    const Eigen::Vector3d plusCamera  = cameraFromWorld * plusWorld;
    const Eigen::Vector3d minusCamera = cameraFromWorld * minusWorld;
    double plusHeight                 = 0.0;
    double minusHeight                = 0.0;
    ASSERT_TRUE(OracleHeight(image, weights, plusCamera.x() / pixelSize, plusCamera.y() / pixelSize, plusHeight));
    ASSERT_TRUE(OracleHeight(image, weights, minusCamera.x() / pixelSize, minusCamera.y() / pixelSize, minusHeight));
    const double numericU = ((plusCamera.z() - plusHeight) - (minusCamera.z() - minusHeight)) / (2.0 * epsilon);
    EXPECT_NEAR(sample.gradientWorld_.dot(cameraFromWorld.linear().transpose() * Eigen::Vector3d::UnitX()), numericU,
                3e-6);
}

TEST(BievrHeightSample, RejectsStencilDomainDimensionsAndNonfiniteInputs)
{
    Eigen::MatrixXf image                   = Eigen::MatrixXf::Ones(5, 6);
    Eigen::MatrixXf weights                 = Eigen::MatrixXf::Ones(5, 6);
    const Eigen::Isometry3d cameraFromWorld = Eigen::Isometry3d::Identity();
    HeightSample sample;
    EXPECT_FALSE(SampleBievrHeight(image, weights, cameraFromWorld, 1.0, Eigen::Vector3d(-0.01, 2.0, 1.0), sample));
    EXPECT_FALSE(SampleBievrHeight(image, weights, cameraFromWorld, 1.0, Eigen::Vector3d(5.0, 2.0, 1.0), sample));
    EXPECT_FALSE(SampleBievrHeight(image, weights, cameraFromWorld, 1.0, Eigen::Vector3d(2.0, 4.0, 1.0), sample));
    EXPECT_FALSE(SampleBievrHeight(image, weights, cameraFromWorld, 0.0, Eigen::Vector3d(2.0, 2.0, 1.0), sample));

    Eigen::MatrixXf mismatched = Eigen::MatrixXf::Ones(4, 6);
    EXPECT_FALSE(SampleBievrHeight(image, mismatched, cameraFromWorld, 1.0, Eigen::Vector3d(2.0, 2.0, 1.0), sample));
    Eigen::MatrixXf emptyMask = Eigen::MatrixXf::Zero(5, 6);
    EXPECT_FALSE(SampleBievrHeight(image, emptyMask, cameraFromWorld, 1.0, Eigen::Vector3d(2.25, 2.25, 1.0), sample));

    weights(2, 2) = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(SampleBievrHeight(image, weights, cameraFromWorld, 1.0, Eigen::Vector3d(2.25, 2.25, 1.0), sample));
    weights.setOnes();
    image(2, 2) = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(SampleBievrHeight(image, weights, cameraFromWorld, 1.0, Eigen::Vector3d(2.25, 2.25, 1.0), sample));

    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(SampleBievrHeight(image, weights, cameraFromWorld, nan, Eigen::Vector3d(2.0, 2.0, 1.0), sample));
    EXPECT_FALSE(SampleBievrHeight(image, weights, cameraFromWorld, 1.0, Eigen::Vector3d(nan, 2.0, 1.0), sample));
    Eigen::Isometry3d nonfiniteTransform = cameraFromWorld;
    nonfiniteTransform.translation().x() = nan;
    EXPECT_FALSE(SampleBievrHeight(image, weights, nonfiniteTransform, 1.0, Eigen::Vector3d(2.0, 2.0, 1.0), sample));
}

TEST(BievrObservation, TwelveColumnsMatchStateIkfomRightPerturbations)
{
    Eigen::MatrixXf image;
    Eigen::MatrixXf weights;
    MakeImage(image, weights, true);
    Eigen::Isometry3d cameraFromWorld = Eigen::Isometry3d::Identity();
    cameraFromWorld.linear() =
        Eigen::AngleAxisd(-0.22, Eigen::Vector3d(-0.2, 0.6, 0.7).normalized()).toRotationMatrix();
    cameraFromWorld.translation() = Eigen::Vector3d(18.0, 19.0, -0.3);
    constexpr double pixelSize    = 1.0;
    const Eigen::Vector3d pointLidar(0.42, -0.31, 2.1);
    state_ikfom state;
    state.pos          = Eigen::Vector3d(0.3, -0.7, 1.2);
    state.rot          = Eigen::Quaterniond(Eigen::AngleAxisd(0.35, Eigen::Vector3d(0.2, 0.7, -0.4).normalized()));
    state.offset_R_L_I = Eigen::Quaterniond(Eigen::AngleAxisd(-0.28, Eigen::Vector3d(-0.4, 0.3, 0.8).normalized()));
    state.offset_T_L_I = Eigen::Vector3d(0.12, -0.08, 0.04);

    HeightSample heightSample;
    const Eigen::Vector3d pointImu   = state.offset_R_L_I.toRotationMatrix() * pointLidar + state.offset_T_L_I;
    const Eigen::Vector3d pointWorld = state.rot.toRotationMatrix() * pointImu + state.pos;
    ASSERT_TRUE(SampleBievrHeight(image, weights, cameraFromWorld, pixelSize, pointWorld, heightSample));
    double oracleResidual = 0.0;
    ASSERT_TRUE(OracleResidual(state, pointLidar, cameraFromWorld, pixelSize, image, weights, oracleResidual));
    EXPECT_NEAR(heightSample.residual_, oracleResidual, 2e-7);

    constexpr double sigma = 0.23;
    constexpr double delta = 0.19;
    ObservationRow enabled;
    ObservationRow disabled;
    ASSERT_TRUE(EvaluateBievrObservation(pointLidar, state.rot.toRotationMatrix(),
                                         state.offset_R_L_I.toRotationMatrix(), state.offset_T_L_I, heightSample, sigma,
                                         delta, true, enabled));
    ASSERT_TRUE(EvaluateBievrObservation(pointLidar, state.rot.toRotationMatrix(),
                                         state.offset_R_L_I.toRotationMatrix(), state.offset_T_L_I, heightSample, sigma,
                                         delta, false, disabled));
    EXPECT_NEAR(enabled.innovation_, -oracleResidual, 2e-7);
    EXPECT_DOUBLE_EQ(disabled.jacobian_.template tail<6>().norm(), 0.0);
    EXPECT_NEAR((enabled.jacobian_.template head<6>() - disabled.jacobian_.template head<6>()).norm(), 0.0, 1e-14);

    constexpr double epsilon = 1e-6;
    for (int column = 0; column < 12; ++column)
    {
        Eigen::Matrix<double, state_ikfom::DOF, 1> increment = Eigen::Matrix<double, state_ikfom::DOF, 1>::Zero();
        increment(column)                                    = epsilon;
        state_ikfom plus                                     = state;
        state_ikfom minus                                    = state;
        plus.boxplus(increment);
        minus.boxplus(increment, -1.0);
        double residualPlus  = 0.0;
        double residualMinus = 0.0;
        ASSERT_TRUE(OracleResidual(plus, pointLidar, cameraFromWorld, pixelSize, image, weights, residualPlus));
        ASSERT_TRUE(OracleResidual(minus, pointLidar, cameraFromWorld, pixelSize, image, weights, residualMinus));
        const double numeric = (residualPlus - residualMinus) / (2.0 * epsilon);
        EXPECT_NEAR(enabled.jacobian_(column), numeric, 2e-6) << "column " << column;
        if (column < 6)
        {
            EXPECT_NEAR(disabled.jacobian_(column), numeric, 2e-6) << "column " << column;
        }
    }
}

TEST(BievrObservation, HuberInformationAtZeroAndThresholdSigns)
{
    HeightSample sample;
    sample.gradientWorld_  = Eigen::Vector3d(0.3, -0.2, 1.0);
    sample.height_         = 0.4;
    sample.pixelU_         = 2.2;
    sample.pixelV_         = 3.3;
    constexpr double sigma = 0.25;
    constexpr double delta = 0.2;
    const Eigen::Vector3d pointLidar(0.2, -0.1, 1.1);
    const Eigen::Matrix3d rotWorldImu = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Matrix3d rotImuLidar = Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Vector3d transImuLidar(0.1, 0.2, -0.3);
    const std::array<double, 5> residuals = {0.0, delta, -delta, 2.0 * delta, -2.0 * delta};
    for (double residual : residuals)
    {
        sample.residual_ = residual;
        ObservationRow row;
        ASSERT_TRUE(EvaluateBievrObservation(pointLidar, rotWorldImu, rotImuLidar, transImuLidar, sample, sigma, delta,
                                             true, row));
        const double expectedHuber = std::abs(residual) <= delta ? 1.0 : 0.5;
        EXPECT_NEAR(row.information_, expectedHuber / (sigma * sigma), 1e-12);
        EXPECT_DOUBLE_EQ(row.innovation_, -residual);
    }
}
}  // namespace

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
