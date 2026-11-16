/**
 * @file measurement.h
 * @brief BIEVR 高度观测与稳健权重的独立数学接口。
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdexcept>

namespace pv_lio_plus::observation
{
/** 一次掩码双线性高度采样及其世界系残差梯度。 */
struct HeightSample
{
    double residual_               = 0.0;                      ///< 米制高度残差。
    Eigen::Vector3d gradientWorld_ = Eigen::Vector3d::Zero();  ///< 未归一化的世界系残差梯度。
    double height_                 = 0.0;                      ///< 插值得到的米制高度。
    double pixelU_                 = 0.0;                      ///< 连续图像横坐标。
    double pixelV_                 = 0.0;                      ///< 连续图像纵坐标。
};

/** 一行可送入共同 IKFoM 的 12 列高度观测。 */
struct ObservationRow
{
    Eigen::Matrix<double, 1, 12> jacobian_ = Eigen::Matrix<double, 1, 12>::Zero();
    double innovation_                     = 0.0;  ///< 负残差，单位为米。
    double information_                    = 0.0;  ///< Huber 权重除以残差方差。
};

bool SampleBievrHeight(const Eigen::MatrixXf& image, const Eigen::MatrixXf& weights,
                       const Eigen::Isometry3d& cameraFromWorld, double pixelSize, const Eigen::Vector3d& pointWorld,
                       HeightSample& sample);

bool EvaluateBievrObservation(const Eigen::Vector3d& pointLidar, const Eigen::Matrix3d& rotWorldImu,
                              const Eigen::Matrix3d& rotImuLidar, const Eigen::Vector3d& transImuLidar,
                              const HeightSample& heightSample, double sigma, double delta, bool extrinsicEnable,
                              ObservationRow& row);

double GemanMcClureWeight(double residualM, double scaleM);
}  // namespace pv_lio_plus::observation
