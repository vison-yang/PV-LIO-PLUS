/**
 * @file      ProbabilisticKernelOptimizer.cpp
 * @brief     Implementation of Probabilistic Kernel Optimization.
 * @author    Seungwon Choi
 * @email     csw3575@snu.ac.kr
 * @date      2025-11-19
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#include "ProbabilisticKernelOptimizer.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <limits>
#include <random>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace lio {

ProbabilisticKernelOptimizer::ProbabilisticKernelOptimizer(const PKOConfig& config)
    : m_config(config)
    , m_alpha_star_ref(config.max_scale_factor)
    , m_initialized(false) {
}

void ProbabilisticKernelOptimizer::Reset() {
    m_alpha_star_ref = m_config.max_scale_factor;
    m_gmm_weights.clear();
    m_gmm_means.clear();
    m_gmm_variances.clear();
}

double ProbabilisticKernelOptimizer::CalculateScaleFactor(const std::vector<double>& residuals) {
    if (residuals.empty()) {
        return 1.0;
    }

    if (!m_config.use_adaptive) {
        return m_config.min_scale_factor;
    }

    // Lazy initialization
    if (!m_initialized) {
        InitializePKO();
        m_initialized = true;
    }

    // Fit GMM to residual distribution
    FitGMM(residuals);

    // Find best alpha using JS divergence
    double best_alpha = m_config.min_scale_factor;
    double best_cost = std::numeric_limits<double>::max();

    for (size_t i = 1; i < m_alpha_candidates.size(); ++i) {
        double alpha = m_alpha_candidates[i];

        // Graduated non-convexity: skip alphas larger than reference
        if (alpha > m_alpha_star_ref) {
            continue;
        }

        double js_divergence = CalculateJSDivergence(alpha);

        if (js_divergence < best_cost) {
            best_cost = js_divergence;
            best_alpha = alpha;
        }
    }

    // Update reference alpha for next iteration
    m_alpha_star_ref = best_alpha;

    return best_alpha;
}

void ProbabilisticKernelOptimizer::InitializePKO() {
    m_alpha_candidates.clear();
    m_partition_functions.clear();

    m_alpha_candidates.resize(m_config.num_alpha_segments + 1);
    m_partition_functions.resize(m_config.num_alpha_segments + 1);

    // First value: min_scale_factor
    m_alpha_candidates[0] = m_config.min_scale_factor;
    m_partition_functions[0] = CalculatePartitionFunction(m_config.min_scale_factor);

    // Remaining values with log scaling for better coverage
    for (int i = 1; i <= m_config.num_alpha_segments; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(m_config.num_alpha_segments);
        double log_scaled_value = (std::pow(100.0, t) - 1.0) / 99.0;
        double alpha = m_config.min_scale_factor +
                      (m_config.max_scale_factor - m_config.min_scale_factor) * log_scaled_value;

        m_alpha_candidates[i] = alpha;
        m_partition_functions[i] = CalculatePartitionFunction(alpha);
    }
}

void ProbabilisticKernelOptimizer::FitGMM(const std::vector<double>& residuals) {
    if (residuals.empty()) {
        return;
    }

    int n = residuals.size();
    int sample_size = std::min(m_config.gmm_sample_size, n);

    // Random sampling
    std::vector<int> indices(n);
    std::iota(indices.begin(), indices.end(), 0);
    std::mt19937 gen(42);  // Fixed seed for reproducibility
    std::shuffle(indices.begin(), indices.end(), gen);

    std::vector<double> sampled_data(sample_size);
    for (int i = 0; i < sample_size; ++i) {
        sampled_data[i] = residuals[indices[i]];
    }

    // Initialize GMM parameters with K-means
    m_gmm_means.resize(m_config.gmm_components);
    m_gmm_means[0] = 0.0;  // First component fixed at zero (inliers)

    std::uniform_int_distribution<> dis(0, sampled_data.size() - 1);
    for (int i = 1; i < m_config.gmm_components; ++i) {
        m_gmm_means[i] = sampled_data[dis(gen)];
    }

    // K-means clustering
    std::vector<int> clusters(sampled_data.size());
    std::vector<double> new_means(m_config.gmm_components);

    for (int iter = 0; iter < 100; ++iter) {
        // Assignment step
        for (size_t i = 0; i < sampled_data.size(); ++i) {
            double min_dist = std::numeric_limits<double>::max();
            int cluster_index = 0;
            for (int j = 0; j < m_config.gmm_components; ++j) {
                double dist = std::abs(sampled_data[i] - m_gmm_means[j]);
                if (dist < min_dist) {
                    min_dist = dist;
                    cluster_index = j;
                }
            }
            clusters[i] = cluster_index;
        }

        // Update step
        std::fill(new_means.begin(), new_means.end(), 0.0);
        std::vector<int> counts(m_config.gmm_components, 0);
        for (size_t i = 0; i < sampled_data.size(); ++i) {
            new_means[clusters[i]] += sampled_data[i];
            counts[clusters[i]]++;
        }
        for (int j = 0; j < m_config.gmm_components; ++j) {
            if (j == 0) {
                new_means[j] = 0.0;  // Keep first mean fixed
            } else if (counts[j] > 0) {
                new_means[j] /= counts[j];
            }
        }

        // Check convergence
        if (m_gmm_means == new_means) {
            break;
        }
        new_means[0] = 0.0;
        m_gmm_means = new_means;
    }

    // Initialize variances
    double mean_of_data = std::accumulate(sampled_data.begin(), sampled_data.end(), 0.0) / sampled_data.size();
    double initial_variance = 0.0;
    for (double x : sampled_data) {
        initial_variance += (x - mean_of_data) * (x - mean_of_data);
    }
    initial_variance /= sampled_data.size();
    m_gmm_variances.assign(m_config.gmm_components, initial_variance);

    // Initialize weights from cluster sizes
    std::vector<int> cluster_counts(m_config.gmm_components, 0);
    for (int cluster : clusters) {
        cluster_counts[cluster]++;
    }

    m_gmm_weights.resize(m_config.gmm_components);
    for (int j = 0; j < m_config.gmm_components; ++j) {
        m_gmm_weights[j] = static_cast<double>(cluster_counts[j]) / sampled_data.size();
    }

    // EM algorithm
    const int max_iterations = 100;
    const double convergence_threshold = 1e-6;

    std::vector<std::vector<double>> responsibilities(sample_size,
                                                     std::vector<double>(m_config.gmm_components));

    for (int iter = 0; iter < max_iterations; ++iter) {
        // E-step: calculate responsibilities
        for (int i = 0; i < sample_size; ++i) {
            double sum_resp = 0.0;
            for (int j = 0; j < m_config.gmm_components; ++j) {
                responsibilities[i][j] = m_gmm_weights[j] *
                                        GaussianPDF(sampled_data[i], m_gmm_means[j], m_gmm_variances[j]);
                sum_resp += responsibilities[i][j];
            }
            for (int j = 0; j < m_config.gmm_components; ++j) {
                responsibilities[i][j] /= (sum_resp + 1e-10);
            }
        }

        // M-step: update parameters
        std::vector<double> N_k(m_config.gmm_components, 0.0);
        for (int j = 0; j < m_config.gmm_components; ++j) {
            for (int i = 0; i < sample_size; ++i) {
                N_k[j] += responsibilities[i][j];
            }
        }

        std::vector<double> new_weights(m_config.gmm_components);
        std::vector<double> new_means_em(m_config.gmm_components, 0.0);
        std::vector<double> new_variances(m_config.gmm_components, 0.0);

        for (int j = 0; j < m_config.gmm_components; ++j) {
            new_weights[j] = N_k[j] / sample_size;

            // Update mean (keep first component at zero)
            if (j == 0) {
                new_means_em[j] = 0.0;
            } else {
                for (int i = 0; i < sample_size; ++i) {
                    new_means_em[j] += responsibilities[i][j] * sampled_data[i];
                }
                new_means_em[j] /= (N_k[j] + 1e-10);
            }

            // Update variance
            for (int i = 0; i < sample_size; ++i) {
                double diff = sampled_data[i] - new_means_em[j];
                new_variances[j] += responsibilities[i][j] * diff * diff;
            }
            new_variances[j] /= (N_k[j] + 1e-10);
            new_variances[j] = std::max(new_variances[j], 1e-6);  // Prevent collapse
        }

        // Check convergence
        double param_change = 0.0;
        for (int j = 1; j < m_config.gmm_components; ++j) {
            param_change += std::abs(new_means_em[j] - m_gmm_means[j]);
        }

        m_gmm_weights = new_weights;
        new_means_em[0] = 0.0;
        m_gmm_means = new_means_em;
        m_gmm_variances = new_variances;

        if (param_change < convergence_threshold) {
            break;
        }
    }
}

double ProbabilisticKernelOptimizer::GaussianPDF(double x, double mean, double variance) const {
    if (variance <= 0.0) {
        return 0.0;
    }

    double diff = x - mean;
    double exponent = -0.5 * (diff * diff) / variance;
    double normalization = 1.0 / std::sqrt(2.0 * M_PI * variance);

    return normalization * std::exp(exponent);
}

double ProbabilisticKernelOptimizer::CalculatePartitionFunction(double alpha) const {
    // Numerical integration of Huber kernel from 0 to truncated_threshold
    const double integration_bound = m_config.truncated_threshold;
    const double integration_step = 0.01;

    double integral = 0.0;
    for (double x = 0.0; x <= integration_bound; x += integration_step) {
        double kernel_value = HuberKernelWeight(x, alpha);
        integral += kernel_value * integration_step;
    }

    return std::max(integral, 1e-10);
}

double ProbabilisticKernelOptimizer::HuberKernelWeight(double residual, double delta) const {
    double abs_residual = std::abs(residual);
    if (abs_residual <= delta) {
        return 1.0;
    } else {
        return delta / abs_residual;
    }
}

double ProbabilisticKernelOptimizer::CalculateJSDivergence(double alpha) {
    // Discretize residual range
    const int num_segments = 100;
    const double dr = m_config.truncated_threshold / num_segments;

    std::vector<double> resvec(num_segments);
    for (int i = 0; i < num_segments; ++i) {
        resvec[i] = dr * (1 + i);
    }

    // Find corresponding partition function
    double partition_func = 0.0;
    for (size_t j = 0; j < m_alpha_candidates.size(); ++j) {
        if (std::abs(m_alpha_candidates[j] - alpha) < 1e-10) {
            partition_func = m_partition_functions[j];
            break;
        }
    }

    if (partition_func == 0.0) {
        partition_func = CalculatePartitionFunction(alpha);
    }

    if (partition_func < 1e-10) {
        return std::numeric_limits<double>::max();
    }

    double cost = 0.0;
    double cnt = 0.0;

    for (double r : resvec) {
        // P(r): Empirical distribution from GMM
        double Pr = 0.0;
        for (int m = 0; m < m_config.gmm_components && m < static_cast<int>(m_gmm_weights.size()); ++m) {
            Pr += m_gmm_weights[m] * GaussianPDF(r, m_gmm_means[m], m_gmm_variances[m]);
        }
        Pr += 1e-10;

        // Q(r): Kernel distribution
        double kernel_val = HuberKernelWeight(r, alpha);
        double Q = kernel_val / (partition_func + 1e-10) + 1e-10;

        // Jensen-Shannon divergence
        double M = 0.5 * (Pr + Q);
        double jsd = 0.5 * (Pr * std::log(Pr / M) + Q * std::log(Q / M));

        if (!std::isnan(jsd)) {
            cost += jsd;
            cnt += 1.0;
        }
    }

    if (cnt == 0) {
        return std::numeric_limits<double>::max();
    }

    return cost / cnt;
}

} // namespace lio
