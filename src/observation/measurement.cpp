/**
 * @file measurement.cpp
 * @brief BIEVR 高度采样、共同状态观测行及 Geman-McClure 权重实现。
 */
#include "observation/measurement.h"

#include <array>
#include <cmath>
#include <stdexcept>

namespace pv_lio_plus::observation
{
/**
 * @brief 在固定 2x2 像素 stencil 上计算掩码双线性高度及精确分段导数。
 * @param image 米制高度图。
 * @param weights 有效像素权重图；仅大于零的 stencil 角参与插值。
 * @param cameraFromWorld 世界系到相机系的变换。
 * @param pixelSize 每像素对应的米数，必须为有限正数。
 * @param pointWorld 世界系查询点。
 * @param sample 成功时写入采样结果。
 * @return stencil 在图像内、输入有效且有效权重和为正时返回 true。
 */
bool SampleBievrHeight(const Eigen::MatrixXf& image, const Eigen::MatrixXf& weights,
                       const Eigen::Isometry3d& cameraFromWorld, double pixelSize, const Eigen::Vector3d& pointWorld,
                       HeightSample& sample)
{
    sample = HeightSample{};
    if (image.rows() < 2 || image.cols() < 2 || image.rows() != weights.rows() || image.cols() != weights.cols() ||
        !cameraFromWorld.matrix().allFinite() || !pointWorld.allFinite() || !std::isfinite(pixelSize) ||
        pixelSize <= 0.0)
    {
        return false;
    }

    const Eigen::Vector3d pointCamera = cameraFromWorld * pointWorld;
    const double pixelU               = pointCamera.x() / pixelSize;
    const double pixelV               = pointCamera.y() / pixelSize;
    if (!pointCamera.allFinite() || !std::isfinite(pixelU) || !std::isfinite(pixelV) || pixelU < 0.0 || pixelV < 0.0 ||
        pixelU >= static_cast<double>(image.cols() - 1) || pixelV >= static_cast<double>(image.rows() - 1))
    {
        return false;
    }

    // floor 固定本次局部分支；不在 stencil 外搜索或补充像素。
    const Eigen::Index u0                 = static_cast<Eigen::Index>(std::floor(pixelU));
    const Eigen::Index v0                 = static_cast<Eigen::Index>(std::floor(pixelV));
    const double du                       = pixelU - static_cast<double>(u0);
    const double dv                       = pixelV - static_cast<double>(v0);
    const std::array<Eigen::Index, 4> row = {v0, v0, v0 + 1, v0 + 1};
    const std::array<Eigen::Index, 4> col = {u0, u0 + 1, u0, u0 + 1};
    const std::array<double, 4> basis     = {(1.0 - du) * (1.0 - dv), du * (1.0 - dv), (1.0 - du) * dv, du * dv};
    const std::array<double, 4> basisU    = {-(1.0 - dv), 1.0 - dv, -dv, dv};
    const std::array<double, 4> basisV    = {-(1.0 - du), -du, 1.0 - du, du};

    double numerator    = 0.0;
    double denominator  = 0.0;
    double numeratorU   = 0.0;
    double denominatorU = 0.0;
    double numeratorV   = 0.0;
    double denominatorV = 0.0;
    for (std::size_t i = 0; i < basis.size(); ++i)
    {
        const float weight = weights(row[i], col[i]);
        if (!std::isfinite(weight))
        {
            return false;
        }
        if (weight <= 0.0F)
        {
            continue;
        }

        const double height = static_cast<double>(image(row[i], col[i]));
        if (!std::isfinite(height))
        {
            return false;
        }
        numerator += basis[i] * height;
        denominator += basis[i];
        numeratorU += basisU[i] * height;
        denominatorU += basisU[i];
        numeratorV += basisV[i] * height;
        denominatorV += basisV[i];
    }
    if (!std::isfinite(numerator) || !std::isfinite(denominator) || denominator <= 0.0 || !std::isfinite(numeratorU) ||
        !std::isfinite(denominatorU) || !std::isfinite(numeratorV) || !std::isfinite(denominatorV))
    {
        return false;
    }

    const double height = numerator / denominator;
    // 商法则对当前掩码 stencil 求导，不借用上游中心差分近似。
    const double heightU = (numeratorU - height * denominatorU) / denominator;
    const double heightV = (numeratorV - height * denominatorV) / denominator;
    const Eigen::Vector3d gradientCamera(-heightU / pixelSize, -heightV / pixelSize, 1.0);
    const Eigen::Vector3d gradientWorld = cameraFromWorld.linear().transpose() * gradientCamera;
    const double residual               = pointCamera.z() - height;
    if (!std::isfinite(height) || !std::isfinite(heightU) || !std::isfinite(heightV) || !gradientWorld.allFinite() ||
        !std::isfinite(residual))
    {
        return false;
    }

    HeightSample result;
    result.residual_      = residual;
    result.gradientWorld_ = gradientWorld;
    result.height_        = height;
    result.pixelU_        = pixelU;
    result.pixelV_        = pixelV;
    sample                = result;
    return true;
}

/**
 * @brief 将高度残差映射为共同 IKFoM 状态的 12 列 Jacobian 和 Huber 信息量。
 * @param pointLidar 去畸变 LiDAR 系点。
 * @param rotWorldImu IMU 系到世界系旋转矩阵。
 * @param rotImuLidar LiDAR 系到 IMU 系旋转矩阵。
 * @param transImuLidar LiDAR 原点在 IMU 系的位置。
 * @param heightSample 高度采样结果。
 * @param sigma 米制残差标准差，必须有限且大于零。
 * @param delta 米制 Huber 阈值，必须有限且大于零。
 * @param extrinsicEnable 是否保留最后六个外参列。
 * @param row 成功时写入观测行。
 * @return 输入和计算结果有效时返回 true。
 */
bool EvaluateBievrObservation(const Eigen::Vector3d& pointLidar, const Eigen::Matrix3d& rotWorldImu,
                              const Eigen::Matrix3d& rotImuLidar, const Eigen::Vector3d& transImuLidar,
                              const HeightSample& heightSample, double sigma, double delta, bool extrinsicEnable,
                              ObservationRow& row)
{
    row = ObservationRow{};
    if (!pointLidar.allFinite() || !rotWorldImu.allFinite() || !rotImuLidar.allFinite() || !transImuLidar.allFinite() ||
        !heightSample.gradientWorld_.allFinite() || !std::isfinite(heightSample.residual_) ||
        !std::isfinite(heightSample.height_) || !std::isfinite(heightSample.pixelU_) ||
        !std::isfinite(heightSample.pixelV_) || !std::isfinite(sigma) || sigma <= 0.0 || !std::isfinite(delta) ||
        delta <= 0.0)
    {
        return false;
    }

    const double variance = sigma * sigma;
    if (!std::isfinite(variance) || variance <= 0.0)
    {
        return false;
    }

    const Eigen::Vector3d pointImu    = rotImuLidar * pointLidar + transImuLidar;
    const Eigen::Vector3d gradientImu = rotWorldImu.transpose() * heightSample.gradientWorld_;
    if (!pointImu.allFinite() || !gradientImu.allFinite())
    {
        return false;
    }

    ObservationRow result;
    result.jacobian_.template segment<3>(0) = heightSample.gradientWorld_.transpose();
    result.jacobian_.template segment<3>(3) = pointImu.cross(gradientImu).transpose();
    result.jacobian_.template segment<3>(6) = pointLidar.cross(rotImuLidar.transpose() * gradientImu).transpose();
    result.jacobian_.template segment<3>(9) = gradientImu.transpose();
    if (!extrinsicEnable)
    {
        result.jacobian_.template segment<6>(6).setZero();
    }

    const double absoluteResidual = std::abs(heightSample.residual_);
    const double huberWeight      = absoluteResidual <= delta ? 1.0 : delta / absoluteResidual;
    result.innovation_            = -heightSample.residual_;
    result.information_           = huberWeight / variance;
    if (!result.jacobian_.allFinite() || !std::isfinite(result.innovation_) || !std::isfinite(result.information_) ||
        result.information_ <= 0.0)
    {
        return false;
    }

    row = result;
    return true;
}

/**
 * @brief 计算归一化 Geman-McClure 信息权重。
 * @param residualM 米制残差。
 * @param scaleM 米制正尺度参数。
 * @return `(scaleM^2 / (scaleM^2 + residualM^2))^2`，允许下溢为零。
 * @throws std::invalid_argument 任一参数非有限或尺度非正。
 */
double GemanMcClureWeight(double residualM, double scaleM)
{
    if (!std::isfinite(residualM) || !std::isfinite(scaleM) || scaleM <= 0.0)
    {
        throw std::invalid_argument("Geman-McClure parameters must be finite and scale must be positive");
    }

    const double absoluteResidual = std::abs(residualM);
    if (absoluteResidual <= scaleM)
    {
        const double ratio = absoluteResidual / scaleM;
        const double base  = 1.0 / (1.0 + ratio * ratio);
        return base * base;
    }

    // 先取小于 1 的反比，避免 residual^2 或 scale^2 溢出。
    const double inverseRatio = scaleM / absoluteResidual;
    const double ratioSquared = inverseRatio * inverseRatio;
    const double base         = ratioSquared / (1.0 + ratioSquared);
    return base * base;
}
}  // namespace pv_lio_plus::observation
