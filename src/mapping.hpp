/**
 * @file mapping.hpp
 * @brief PV-LIO-PLUS mapping state, lifecycle, and observation interface.
 */

#pragma once

#include "imu_processing.hpp"
#include "preprocess.h"

#include <Eigen/StdVector>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <livox_ros_driver/CustomMsg.h>
#include <memory>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>

namespace pv_lio_plus {

class MapManager;
struct MapManagerConfig;
struct MapPoint;

#if defined(ROOT_DIR)
static constexpr const char* kDefaultRootDir = ROOT_DIR;
#else
// Keep IntelliSense usable when the catkin-only ROOT_DIR definition is absent.
static constexpr const char* kDefaultRootDir = ".";
#endif

/**
 * @brief Owns one PV-LIO-PLUS mapping node and all mutable runtime state.
 *
 * Core mapping and observation methods are defined in mapping.cpp; ROS
 * callbacks, buffering, publication, and result persistence are defined in
 * utils.cpp.  The class remains internal to this ROS node.
 */
class Mapping {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** @brief Constructs an unstarted mapping instance. */
    Mapping();

    /** @brief Stops callbacks and releases the owned mapping state. */
    ~Mapping();

    Mapping(const Mapping&) = delete;
    Mapping& operator=(const Mapping&) = delete;
    Mapping(Mapping&&) = delete;
    Mapping& operator=(Mapping&&) = delete;

    /** @brief Runs the ROS node after parameters and interfaces are ready. */
    int Run();

  private:
    // ------------------------------ Configuration -----------------------------
    /** @brief Loads ROS parameters into this Mapping instance. */
    bool load_parameters(ros::NodeHandle& nh);

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
    std::vector<MapPoint, Eigen::aligned_allocator<MapPoint>>
    make_manager_points(const PointCloudXYZI::Ptr& points_lidar, const PointCloudXYZI::Ptr& points_world,
                        const std::vector<M3D>& cov_lidar) const;
    M3D calc_lidar_cov_for_backend(const V3D& point) const;

    // ------------------------------ Map lifecycle ----------------------------
    void InitializeMap();
    void UpdateMap();

    // ------------------------------ Configuration state ------------------------
    float det_range_ = 300.0f;
    bool time_sync_en_ = false;
    bool extrinsic_est_en_ = true;
    bool path_pub_en_ = true;
    bool pcd_save_en_ = false;
    bool scan_pub_en_ = false;
    bool scan_dense_pub_en_ = false;
    bool scan_body_pub_en_ = false;
    bool scan_lidar_pub_en_ = false;
    bool publish_voxel_map_ = false;
    bool map_publish_en_ = true;

    std::string root_dir_ = kDefaultRootDir;
    std::string lidar_topic_;
    std::string imu_topic_;
    double lidar_time_offset_ = 0.0;
    double ranging_cov_ = 0.0;
    double angle_cov_ = 0.0;
    double gyr_cov_ = 0.1;
    double acc_cov_ = 0.1;
    double b_gyr_cov_ = 0.0001;
    double b_acc_cov_ = 0.0001;
    double filter_size_surf_min_ = 0.0;
    double plannar_threshold_ = 0.003;
    double sigma_num_ = 2.0;
    double voxel_size_ = 1.0;
    int max_layer_ = 0;
    int max_points_size_ = 50;
    int max_cov_points_size_ = 50;
    int num_max_iterations_ = 0;
    int publish_max_voxel_layer_ = 0;
    std::vector<double> extrin_t_{3, 0.0};
    std::vector<double> extrin_r_{9, 0.0};
    std::vector<double> layer_point_size_;
    std::vector<int> layer_size_;
    std::unique_ptr<pv_lio_plus::MapManagerConfig> map_manager_config_;

    // ---------------------------- Sensor buffering ----------------------------
    std::mutex buffer_mutex_;
    std::condition_variable buffer_condition_;
    std::deque<double> time_buffer_;
    std::deque<PointCloudXYZI::Ptr> lidar_buffer_;
    std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer_;
    bool lidar_pushed_ = false;
    bool first_scan_ = true;
    bool ekf_initialized_ = false;
    double last_timestamp_lidar_ = 0.0;
    double last_timestamp_imu_ = -1.0;
    double lidar_end_time_ = 0.0;
    double first_lidar_time_ = 0.0;
    double timediff_lidar_wrt_imu_ = 0.0;
    bool timediff_set_ = false;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;
    int scan_count_ = 0;

    // ---------------------------- Current-frame data --------------------------
    MeasureGroup measures_;
    PointCloudXYZI::Ptr feats_undistort_{new PointCloudXYZI()};
    PointCloudXYZI::Ptr feats_undistort_down_{new PointCloudXYZI()};
    std::vector<M3D> var_down_lidar_;
    pcl::VoxelGrid<PointType> down_size_filter_surf_;

    // ------------------------------ Estimator ---------------------------------
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;
    state_ikfom state_point_;
    std::shared_ptr<Preprocess> preprocess_{new Preprocess()};
    std::shared_ptr<ImuProcess> imu_process_{new ImuProcess()};
    M3D lidar_r_wrt_imu_{Eye3d};
    V3D lidar_t_wrt_imu_{Zero3d};

    // --------------------------------- Map ------------------------------------
    std::unique_ptr<pv_lio_plus::MapManager> map_manager_;
    bool map_initialized_ = false;

    // -------------------------- ROS message state -----------------------------
    nav_msgs::Path path_;
    nav_msgs::Odometry odom_after_mapped_;
    geometry_msgs::Quaternion geo_quat_;
    geometry_msgs::PoseStamped body_pose_;

    // ---------------------------- Output state ---------------------------------
    std::vector<std::vector<double>> poses_;
    PointCloudXYZI::Ptr pcl_wait_save_{new PointCloudXYZI()};
    int pcd_save_interval_ = -1;
    int pcd_index_ = 0;
    int scan_wait_num_ = 0;

    // -------------------------- Diagnostics/timing ----------------------------
    double res_mean_last_ = 0.05;
    double total_residual_ = 0.0;
    int effective_feature_num_ = 0;
    int feats_down_size_ = 0;
    std::size_t scan_total_count_ = 0;
    double search_time_ = 0.0;
    double eseikf_time_ = 0.0;
    double update_time_ = 0.0;
    double total_time_ = 0.0;
    double avg_search_time_ = 0.0;
    double avg_eseikf_time_ = 0.0;
    double avg_update_time_ = 0.0;
    double avg_total_time_ = 0.0;
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

} // namespace pv_lio_plus
