/**
 * @file mapping.hpp
 * @brief PV-LIO-PLUS mapping state, lifecycle, and observation interface.
 */

#pragma once

#include <geometry_msgs/Quaternion.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>

#include <Eigen/StdVector>
#include <cstddef>
#include <memory>
#include <vector>

#include "config.hpp"
#include "imu_processing.hpp"
#include "publish_subscribe.hpp"

namespace pv_lio_plus
{
class MapManager;
struct MapManagerConfig;
struct MapPoint;

/**
 * @brief Owns one PV-LIO-PLUS mapping node and all mutable runtime state.
 *
 * Core mapping and observation methods are defined in mapping.cpp; ROS
 * callbacks, buffering, publication, and result persistence are defined in
 * utils.cpp.  The class remains internal to this ROS node.
 */
class Mapping
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** @brief Constructs an unstarted mapping instance. */
    Mapping();

    /** @brief Stops callbacks and releases the owned mapping state. */
    ~Mapping();

    Mapping(const Mapping&) = delete;
    Mapping& operator=(const Mapping&) = delete;
    Mapping(Mapping&&)                 = delete;
    Mapping& operator=(Mapping&&) = delete;

    /** @brief Runs the ROS node after parameters and interfaces are ready. */
    int Run();

   private:
    // ------------------------------ Configuration -----------------------------
    Config config_;

    // ----------------------------- Observation -------------------------------
    static void ObservationModelTrampoline(state_ikfom& state, esekfom::dyn_share_datastruct<double>& ekfom_data);
    void ObservationModel(state_ikfom& state, esekfom::dyn_share_datastruct<double>& ekfom_data);
    void ObservationModelVoxelMap(state_ikfom& state, esekfom::dyn_share_datastruct<double>& ekfom_data);
    void ObservationModelVoxelMapPlus(state_ikfom& state, esekfom::dyn_share_datastruct<double>& ekfom_data);
    void ObservationModelPointBackend(state_ikfom& state, esekfom::dyn_share_datastruct<double>& ekfom_data);
    void ObservationModelC3P(state_ikfom& state, esekfom::dyn_share_datastruct<double>& ekfom_data);

    // ---------------------- Geometry and conversion --------------------------
    void transformLidar2World(const state_ikfom& state, const PointCloudXYZI::Ptr& input_cloud,
                              PointCloudXYZI::Ptr& trans_cloud) const;
    M3D transformLidarCovToWorld(Eigen::Vector3d& point_lidar, const esekfom::esekf<state_ikfom, 12, input_ikfom>& kf,
                                 const Eigen::Matrix3d& cov_lidar) const;
    std::vector<MapPoint, Eigen::aligned_allocator<MapPoint>> make_manager_points(
        const PointCloudXYZI::Ptr& points_lidar, const PointCloudXYZI::Ptr& points_world,
        const std::vector<M3D>& cov_lidar) const;
    M3D calc_lidar_cov_for_backend(const V3D& point) const;

    // ------------------------------ Map lifecycle ----------------------------
    void InitializeMap();
    void UpdateMap();

    // ---------------------------- Sensor lifecycle ----------------------------
    bool first_scan_         = true;
    bool ekf_initialized_    = false;
    double first_lidar_time_ = 0.0;

    // ---------------------------- Current-frame data --------------------------
    PointCloudXYZI::Ptr feats_undistort_{new PointCloudXYZI()};
    PointCloudXYZI::Ptr feats_undistort_down_{new PointCloudXYZI()};
    std::vector<M3D> var_down_lidar_;
    pcl::VoxelGrid<PointType> down_size_filter_surf_;

    // ------------------------------ Estimator ---------------------------------
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;
    state_ikfom state_point_;
    std::shared_ptr<ImuProcess> imu_process_{new ImuProcess()};
    geometry_msgs::Quaternion geo_quat_;
    M3D lidar_r_wrt_imu_{Eye3d};
    V3D lidar_t_wrt_imu_{Zero3d};

    // --------------------------------- Map ------------------------------------
    std::unique_ptr<pv_lio_plus::MapManager> map_manager_;
    bool map_initialized_ = false;

    // -------------------------- ROS transport state ---------------------------
    PublishSubscribe publish_subscribe_;

    // -------------------------- Diagnostics/timing ----------------------------
    double res_mean_last_         = 0.05;
    double total_residual_        = 0.0;
    int effective_feature_num_    = 0;
    int feats_down_size_          = 0;
    std::size_t scan_total_count_ = 0;
    double search_time_           = 0.0;
    double eseikf_time_           = 0.0;
    double update_time_           = 0.0;
    double total_time_            = 0.0;
    double avg_search_time_       = 0.0;
    double avg_eseikf_time_       = 0.0;
    double avg_update_time_       = 0.0;
    double avg_total_time_        = 0.0;
    PointCloudXYZI::Ptr feats_with_correspondence_{new PointCloudXYZI()};

    static Mapping* active_instance_;
};

/**
 * @brief Initializes and runs one PV-LIO-PLUS mapping instance.
 * @param argc Process argument count.
 * @param argv Process argument vector.
 * @return Node exit code.
 */
int RunMappingNode(int argc, char** argv);

}  // namespace pv_lio_plus
