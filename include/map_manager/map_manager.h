#ifndef PV_LIO_PLUS_MAP_MANAGER_H
#define PV_LIO_PLUS_MAP_MANAGER_H

/**
 * @file map_manager.h
 * @brief Common local-map types and lifecycle interface for PV-LIO-PLUS.
 */

#include <geometry_msgs/Quaternion.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class ImuProcess;

#include "common_lib.h"
#include "native/c3p_voxelmap/c3p_voxel_map_util.hpp"
#include "native/ikdtree/ikd_tree.h"
#include "native/ivox/ivox3d.h"
#include "native/voxelmap/voxel_map_util.hpp"
#include "native/voxelmap_plus/voxelmapplus_util.hpp"

namespace pv_lio_plus
{

/** @brief Local-map backends selectable by PV-LIO-PLUS. */
enum class MapType
{
    /// PV-LIO's original probabilistic adaptive voxel map.
    VoxelMap,
    /// VoxelMap++ with its native residual and update logic.
    VoxelMapPlus,
    /// FAST-LIO2's incremental ikd-tree point map.
    IKDTree,
    /// Faster-LIO's iVox point map.
    IVox,
    /// C3P-VoxelMap's compact probabilistic voxel-plane map.
    C3PVoxelMap
};

MapType ParseMapType(const std::string &name);

const char *MapTypeName(MapType type);

/** @brief A map point represented in LiDAR and world frames. */
struct MapPoint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Point coordinates in the current LiDAR frame.
    V3D point_lidar = V3D::Zero();
    /// Point coordinates in the world frame.
    V3D point_world = V3D::Zero();
    /// Point covariance in the LiDAR frame.
    M3D cov_lidar    = M3D::Zero();
    /// Point covariance in the world frame.
    M3D cov_world    = M3D::Zero();
};

/** @brief Eigen-aligned collection of map points. */
using MapPointList = std::vector<MapPoint, Eigen::aligned_allocator<MapPoint>>;

/**
 * @brief Backend-independent point-to-plane match.
 *
 * The fields cover the native residual contracts of the PV voxel maps and the
 * adapters for point-map backends, allowing the filter loop to remain common.
 */
struct PlaneMatch
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Backend that produced the match.
    MapType map_type = MapType::VoxelMap;
    /// Query point in the LiDAR frame.
    V3D point         = V3D::Zero();
    /// Query point in the world frame.
    V3D point_world   = V3D::Zero();
    /// Unit normal of the matched plane.
    V3D normal        = V3D::Zero();
    /// Representative center of the matched plane.
    V3D center        = V3D::Zero();
    /// Native residual direction used by VoxelMap++.
    V3D omega         = V3D::Zero();
    /// Query-point covariance in the world frame.
    M3D point_cov     = M3D::Zero();
    /// Six-dimensional plane covariance, when provided by the backend.
    Eigen::Matrix<double, 6, 6> plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
    /// Plane equation offset or native equivalent.
    double d          = 0.0;
    /// Signed point-to-plane residual.
    double distance   = 0.0;
    /// Norm of @ref omega.
    double omega_norm = 0.0;
    /// Native voxel-map layer containing the match.
    int layer          = 0;
    /// Native VoxelMap++ principal direction.
    int main_direction = 0;
};

/** @brief Eigen-aligned collection of plane matches. */
using PlaneMatchList = std::vector<PlaneMatch, Eigen::aligned_allocator<PlaneMatch>>;

/** @brief Configuration shared by all local-map backends. */
struct MapManagerConfig
{
    /// Backend selected for this manager.
    MapType type = MapType::VoxelMap;

    /// Base voxel size used by voxel-map backends.
    double voxel_size       = 1.0;
    /// Maximum voxel hierarchy layer used during search.
    int max_layer           = 2;
    /// Point limits for each voxel hierarchy layer.
    std::vector<int> layer_point_size;
    /// Maximum number of points retained in a voxel.
    int max_points_size     = 100;
    /// Maximum number of covariance points retained in a voxel.
    int max_cov_points_size = 100;
    /// Plane-fit threshold used by voxel backends.
    double plane_threshold  = 0.01;
    /// Sigma multiplier used by probabilistic voxel matching.
    double sigma_num        = 3.0;
    /// VoxelMap++ update threshold.
    int plus_update_size_threshold = 5;

    /// Number of neighbors requested by point-map backends.
    int nearest_point_count = NUM_MATCH_POINTS;
    /// Maximum accepted neighbor distance for point-map search.
    double nearest_max_range = 5.0;
    /// Plane-fit residual threshold for point-map backends.
    double plane_fit_threshold = 0.1;
    /// Downsampling leaf size for point-map backends.
    double point_map_downsample_size = 0.5;

    /// ikd-tree deletion parameter.
    double ikd_delete_param = 0.5;
    /// ikd-tree balance parameter.
    double ikd_balance_param = 0.7;
    /// ikd-tree box length parameter.
    double ikd_box_length = 0.2;

    /// iVox voxel resolution.
    double ivox_resolution = 0.2;
    /// iVox neighborhood type: 0, 6, 18, or 26.
    int ivox_nearby_type = 18;
    /// Maximum number of iVox grids.
    std::size_t ivox_capacity = 1000000;

    /// Enables C3P-VoxelMap plane merging.
    bool c3p_enable_voxel_merging = false;
    /// C3P plane-merge angular threshold.
    double c3p_merge_theta_thresh = 0.05;
    /// C3P plane-merge distance threshold.
    double c3p_merge_dist_thresh = 0.05;
    /// Minimum eigenvalue threshold for C3P plane covariance.
    double c3p_merge_cov_min_eigen_val_thresh = 0.002;
    /// Maximum C3 merge distance along the x axis.
    double c3p_merge_x_coord_diff_thresh = 5.0;
    /// Maximum C3 merge distance along the y axis.
    double c3p_merge_y_coord_diff_thresh = 5.0;

    /// Enables moving-window maintenance; disabled by default.
    bool local_window_enabled = false;
};

/** @brief Axis-aligned world-frame bounds of the active local window. */
struct MapWindow
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Whether the bounds have been initialized.
    bool initialized = false;
    /// Minimum x/y/z coordinates of the window.
    V3D min_bound    = V3D::Zero();
    /// Maximum x/y/z coordinates of the window.
    V3D max_bound    = V3D::Zero();
};

/**
 * @brief Owns the selected local-map backend and exposes one lifecycle contract.
 *
 * Native PV voxel maps, FAST-LIO's ikd-tree, Faster-LIO's iVox, and the
 * C3P-VoxelMap implementation are owned here.  PV voxel backends retain their
 * native match types, while point-map and C3P adapters expose PlaneMatch to the
 * common observation path.
 */
class MapManager
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MapManager();
    explicit MapManager(const MapManagerConfig &config);

    ~MapManager();

    MapManager(const MapManager &) = delete;
    MapManager &operator=(const MapManager &) = delete;

    void configure(const MapManagerConfig &config);

    const MapManagerConfig &config() const;
    MapType type() const;
    bool initialized() const;
    bool supports_selected_backend() const;

    void clear();

    void initialize(const MapPointList &points);

    void initialize(const std::vector<voxel_map_ns::pointWithCov> &points);

    void initialize(const std::vector<voxel_map_plus_ns::pointWithCov> &points);

    void update(const MapPointList &points, std::uint32_t frame_number = 0);

    void update(const std::vector<voxel_map_ns::pointWithCov> &points,
                std::uint32_t frame_number = 0);

    void update(const std::vector<voxel_map_plus_ns::pointWithCov> &points,
                std::uint32_t frame_number = 0);

    void search(const MapPointList &points, PlaneMatchList &matches,
                std::vector<V3D> &non_match);

    void search(const std::vector<voxel_map_ns::pointWithCov> &points,
                std::vector<voxel_map_ns::ptpl> &matches,
                std::vector<V3D> &non_match);

    void search(const std::vector<voxel_map_plus_ns::pointWithCov> &points,
                std::vector<voxel_map_plus_ns::ptpl> &matches,
                std::vector<V3D> &non_match);

    void move_window(const V3D &center, const V3D &half_extent);

    void erase_outside_window(const MapWindow &window);

    const MapWindow &window() const;
    size_t size() const;

    size_t point_count() const;
    PointCloudXYZI::Ptr snapshot() const;

    void publish_planes(const ros::Publisher &publisher,
                        int max_voxel_layer) const;

private:
    static voxel_map_ns::pointWithCov ToVoxelPoint(const MapPoint &point);

    static voxel_map_plus_ns::pointWithCov ToVoxelPlusPoint(const MapPoint &point);

    static MapPoint FromVoxelPoint(const voxel_map_ns::pointWithCov &point);

    static MapPoint FromVoxelPlusPoint(const voxel_map_plus_ns::pointWithCov &point);

    static PlaneMatch FromVoxelMatch(const voxel_map_ns::ptpl &match);

    static PlaneMatch FromVoxelPlusMatch(const voxel_map_plus_ns::ptpl &match);

    static PlaneMatch FromC3PMatch(const c3p_map_ns::ptpl &match);

    static PointType ToPointType(const V3D &point, float intensity = 0.0f);

    static MapPoint FromPointType(const PointType &point);

    void configure_point_backends();

    void initialize_point_backend(const MapPointList &points);

    void update_ikd_tree(const MapPointList &points);

    void update_ivox(const MapPointList &points);

    void initialize_c3p(const MapPointList &points);

    void update_c3p(const MapPointList &points, std::uint32_t frame_number);

    void search_point_backend(const MapPointList &points, PlaneMatchList &matches,
                              std::vector<V3D> &non_match);

    void search_ikd_tree(const MapPointList &points, PlaneMatchList &matches,
                         std::vector<V3D> &non_match);

    void search_ivox(const MapPointList &points, PlaneMatchList &matches,
                     std::vector<V3D> &non_match);

    void search_c3p(const MapPointList &points, PlaneMatchList &matches,
                    std::vector<V3D> &non_match);

    bool fit_point_plane(const MapPoint &query, const PointVector &neighbors,
                         PlaneMatch &match, MapType backend) const;

    void erase_outside_ikd_window(const MapWindow &window);

    void erase_outside_ivox_window(const MapWindow &window);

    void erase_outside_c3p_window(const MapWindow &window);

    void require_supported_backend() const;

    void erase_outside_voxel_window(const MapWindow &window);

    void erase_outside_voxel_plus_window(const MapWindow &window);

    static bool Inside(const V3D &point, const MapWindow &window);

    void filter_snapshot_to_window(const MapWindow &window);

    /// Active backend configuration.
    MapManagerConfig config_;
    /// Whether at least one point batch has initialized the backend.
    bool initialized_ = false;
    /// Current local-window bounds.
    MapWindow window_;
    /// Common points retained for snapshots and backend rebuilds.
    MapPointList map_points_;

    /// Native PV VoxelMap storage.
    std::unordered_map<voxel_map_ns::VOXEL_LOC, voxel_map_ns::OctoTree *> voxel_map_;
    /// Native VoxelMap++ storage.
    std::unordered_map<voxel_map_plus_ns::VOXEL_LOC, voxel_map_plus_ns::UnionFindNode *> voxel_map_plus_;

    /// Native Faster-LIO iVox type used by this manager.
    using IVoxBackend = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>;
    /// Native FAST-LIO2 ikd-tree instance.
    std::unique_ptr<KD_TREE<PointType>> ikd_tree_;
    /// Native Faster-LIO iVox instance.
    std::unique_ptr<IVoxBackend> ivox_;
    /// Native C3P-VoxelMap storage.
    std::unordered_map<c3p_map_ns::VOXEL_LOC, c3p_map_ns::OctoTree *> c3p_voxel_map_;
    /// C3P plane descriptors and merged planes.
    std::unordered_map<c3p_map_ns::PlaneDesc, c3p_map_ns::UnionPlane> c3p_plane_map_;
    /// C3P merged-node lookup table.
    std::unordered_map<int, std::unordered_set<c3p_map_ns::NodeLoc>> c3p_merged_table_;
};

}  // namespace pv_lio_plus

#endif  // PV_LIO_PLUS_MAP_MANAGER_H
