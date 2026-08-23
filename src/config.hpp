/**
 * @file config.hpp
 * @brief Runtime configuration and ROS parameter loading for PV-LIO-PLUS.
 */

#pragma once

#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

#include "imu_processing.hpp"

class Preprocess;

namespace pv_lio_plus
{
class MapManagerConfig;

/**
 * @brief Owns all user-configurable mapping, sensor, noise, and output values.
 *
 * Config is deliberately independent from Mapping's runtime buffers and EKF
 * state.  This keeps parameter loading in one place and makes the estimator
 * state easier to inspect.
 */
class Config
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** @brief Constructs configuration with project defaults. */
    Config();

    /** @brief Releases backend and preprocessing configuration. */
    ~Config();

    /** @brief Loads all ROS parameters and prepares derived backend values. */
    bool Load(ros::NodeHandle& nh);

    /** @brief Returns the configured map-manager settings. */
    MapManagerConfig& map_manager_config();
    const MapManagerConfig& map_manager_config() const;

    /** @brief Returns the configured LiDAR preprocessing object. */
    Preprocess& preprocess();
    const Preprocess& preprocess() const;

    // ------------------------------ Sensor and output -------------------------
    float det_range_        = 300.0f;
    bool time_sync_en_      = false;
    bool extrinsic_est_en_  = true;
    bool path_pub_en_       = true;
    bool pcd_save_en_       = false;
    bool scan_pub_en_       = false;
    bool scan_dense_pub_en_ = false;
    bool scan_body_pub_en_  = false;
    bool scan_lidar_pub_en_ = false;
    bool publish_voxel_map_ = false;
    bool map_publish_en_    = true;

    std::string root_dir_ = ".";
    std::string lidar_topic_;
    std::string imu_topic_;
    double lidar_time_offset_ = 0.0;

    // --------------------------------- Mapping --------------------------------
    double filter_size_surf_min_ = 0.0;
    double plannar_threshold_    = 0.003;
    double sigma_num_            = 2.0;
    double voxel_size_           = 1.0;
    int max_layer_               = 0;
    int max_points_size_         = 50;
    int max_cov_points_size_     = 50;
    int num_max_iterations_      = 0;
    int publish_max_voxel_layer_ = 0;
    int pcd_save_interval_       = -1;
    std::vector<double> extrin_t_{3, 0.0};
    std::vector<double> extrin_r_{9, 0.0};
    std::vector<double> layer_point_size_;
    std::vector<int> layer_size_;

    // --------------------------------- Noise ----------------------------------
    double ranging_cov_ = 0.0;
    double angle_cov_   = 0.0;
    double gyr_cov_     = 0.1;
    double acc_cov_     = 0.1;
    double b_gyr_cov_   = 0.0001;
    double b_acc_cov_   = 0.0001;

   private:
    std::unique_ptr<MapManagerConfig> map_manager_config_;
    std::shared_ptr<Preprocess> preprocess_;
};

}  // namespace pv_lio_plus
