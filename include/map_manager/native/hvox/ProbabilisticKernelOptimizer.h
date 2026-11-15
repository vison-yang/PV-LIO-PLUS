/**
 * @file      ProbabilisticKernelOptimizer.h
 * @brief     Probabilistic Kernel Optimization for adaptive robust estimation.
 * @author    Seungwon Choi
 * @email     csw3575@snu.ac.kr
 * @date      2025-11-19
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#ifndef PROBABILISTIC_KERNEL_OPTIMIZER_H
#define PROBABILISTIC_KERNEL_OPTIMIZER_H

#include <vector>
#include <string>

namespace lio {

/**
 * @brief Configuration parameters for Probabilistic Kernel Optimization
 */
struct PKOConfig {
    bool use_adaptive = true;                    // Enable/disable adaptive optimization
    double min_scale_factor = 0.01;              // Minimum Huber delta
    double max_scale_factor = 10.0;              // Maximum Huber delta
    int num_alpha_segments = 100;                // Number of alpha candidates
    double truncated_threshold = 10.0;           // Integration bound for partition function
    int gmm_components = 3;                      // Number of GMM components
    int gmm_sample_size = 1000;                  // Sample size for GMM fitting

    PKOConfig() = default;
};

/**
 * @brief Probabilistic Kernel Optimizer for adaptive Huber loss scale estimation
 *
 * This class implements the Probabilistic Kernel Optimization (PKO) method to
 * adaptively estimate the optimal scale factor (delta) for Huber loss function.
 *
 * PKO automatically tunes the robust kernel parameter by minimizing the
 * Jensen-Shannon divergence between the empirical residual distribution
 * (modeled by GMM) and the kernel-induced distribution. This eliminates the
 * need for manual parameter tuning while maintaining robust convergence through
 * graduated non-convexity.
 *
 * Key features:
 * - Gaussian Mixture Model (GMM) fitting to residual distribution
 * - Jensen-Shannon divergence minimization between data and kernel distributions
 * - Graduated non-convexity for robust convergence
 * - Pure double precision for numerical stability
 *
 * Reference:
 * Seungwon Choi and Tae-Wan Kim, "Probabilistic Kernel Optimization for Robust State Estimation,"
 * IEEE Robotics and Automation Letters, vol. 10, no. 3,
 * pp. 2998-3005, March 2025, doi: 10.1109/LRA.2025.3536294.
 */
class ProbabilisticKernelOptimizer {
public:
    /**
     * @brief Constructor with configuration
     * @param config PKO configuration parameters
     */
    explicit ProbabilisticKernelOptimizer(const PKOConfig& config = PKOConfig());

    /**
     * @brief Destructor
     */
    ~ProbabilisticKernelOptimizer() = default;

    /**
     * @brief Calculate optimal Huber loss scale factor from residuals
     * @param residuals Vector of residual values (assumed already normalized)
     * @return Optimal Huber delta (scale factor)
     */
    double CalculateScaleFactor(const std::vector<double>& residuals);

    /**
     * @brief Reset optimizer state for new optimization sequence
     */
    void Reset();

    /**
     * @brief Get current configuration
     * @return Reference to configuration
     */
    const PKOConfig& GetConfig() const { return m_config; }

private:
    /**
     * @brief Initialize alpha candidates and partition functions
     */
    void InitializePKO();

    /**
     * @brief Fit Gaussian Mixture Model to residual distribution
     * @param residuals Input residuals
     */
    void FitGMM(const std::vector<double>& residuals);

    /**
     * @brief Compute Gaussian probability density function
     * @param x Input value
     * @param mean Gaussian mean
     * @param variance Gaussian variance
     * @return PDF value
     */
    double GaussianPDF(double x, double mean, double variance) const;

    /**
     * @brief Calculate partition function for given alpha
     * @param alpha Scale parameter
     * @return Partition function value Z(alpha)
     */
    double CalculatePartitionFunction(double alpha) const;

    /**
     * @brief Compute Huber kernel weight
     * @param residual Residual value
     * @param delta Huber scale parameter
     * @return Weight value
     */
    double HuberKernelWeight(double residual, double delta) const;

    /**
     * @brief Calculate Jensen-Shannon divergence between data and kernel distributions
     * @param alpha Scale parameter
     * @return JS divergence value
     */
    double CalculateJSDivergence(double alpha);

private:
    // Configuration
    PKOConfig m_config;

    // PKO state
    std::vector<double> m_alpha_candidates;      // Candidate scale factors
    std::vector<double> m_partition_functions;   // Pre-computed Z(alpha) values
    double m_alpha_star_ref;                     // Reference alpha for graduated non-convexity
    bool m_initialized;                          // Initialization flag

    // GMM parameters
    std::vector<double> m_gmm_weights;           // GMM mixture weights
    std::vector<double> m_gmm_means;             // GMM component means
    std::vector<double> m_gmm_variances;         // GMM component variances
};

} // namespace lio

#endif // PROBABILISTIC_KERNEL_OPTIMIZER_H
