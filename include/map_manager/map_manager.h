#ifndef PV_LIO_PLUS_MAP_MANAGER_H
#define PV_LIO_PLUS_MAP_MANAGER_H

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
#include <vector>

class ImuProcess;

#include "common_lib.h"
#include "voxel_map_util.hpp"
#include "voxelmapplus_util.hpp"

namespace pv_lio_plus
{

enum class MapType
{
    VoxelMap,
    VoxelMapPlus,
    IKDTree,
    IVox,
    C3PVoxelMap
};

MapType ParseMapType(const std::string &name, bool legacy_voxelmap_plus);
const char *MapTypeName(MapType type);

struct MapPoint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    V3D point_lidar = V3D::Zero();
    V3D point_world = V3D::Zero();
    M3D cov_lidar    = M3D::Zero();
    M3D cov_world    = M3D::Zero();
};

using MapPointList = std::vector<MapPoint, Eigen::aligned_allocator<MapPoint>>;

/**
 * A map-independent point-to-plane match.
 *
 * The fields cover the native residual contracts used by PV's two existing
 * maps.  VoxelMap fills normal/center/d, while VoxelMap++ fills omega,
 * omega_norm/distance/main_direction.  Adapters for other maps can fill the
 * contract they support without changing the filter loop.
 */
struct PlaneMatch
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MapType map_type = MapType::VoxelMap;
    V3D point         = V3D::Zero();
    V3D point_world   = V3D::Zero();
    V3D normal        = V3D::Zero();
    V3D center        = V3D::Zero();
    V3D omega         = V3D::Zero();
    M3D point_cov     = M3D::Zero();
    Eigen::Matrix<double, 6, 6> plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
    double d          = 0.0;
    double distance   = 0.0;
    double omega_norm = 0.0;
    int layer          = 0;
    int main_direction = 0;
};

using PlaneMatchList = std::vector<PlaneMatch, Eigen::aligned_allocator<PlaneMatch>>;

struct MapManagerConfig
{
    MapType type = MapType::VoxelMap;

    double voxel_size       = 1.0;
    int max_layer           = 2;
    std::vector<int> layer_point_size;
    int max_points_size     = 100;
    int max_cov_points_size = 100;
    double plane_threshold  = 0.01;
    double sigma_num        = 3.0;
    int plus_update_size_threshold = 5;

    // Windowing is deliberately opt-in.  The original PV maps are growing
    // local maps, so enabling this changes their native lifetime semantics.
    bool local_window_enabled = false;
};

struct MapWindow
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool initialized = false;
    V3D min_bound    = V3D::Zero();
    V3D max_bound    = V3D::Zero();
};

/**
 * Owns the selected local-map backend and exposes one lifecycle contract.
 *
 * The current implementation intentionally keeps the native PV map data
 * structures and calls their original build/query/update functions.  The
 * other backends are represented in MapType and are added as independent
 * adapters in later phases.
 */
class MapManager
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MapManager() = default;
    explicit MapManager(const MapManagerConfig &config);
    ~MapManager();

    MapManager(const MapManager &) = delete;
    MapManager &operator=(const MapManager &) = delete;

    void configure(const MapManagerConfig &config);
    const MapManagerConfig &config() const { return config_; }
    MapType type() const { return config_.type; }
    bool initialized() const { return initialized_; }
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

    /** Search and fit residuals with the selected backend's native routine. */
    void search(const MapPointList &points, PlaneMatchList &matches,
                std::vector<V3D> &non_match);
    void search(const std::vector<voxel_map_ns::pointWithCov> &points,
                std::vector<voxel_map_ns::ptpl> &matches,
                std::vector<V3D> &non_match);
    void search(const std::vector<voxel_map_plus_ns::pointWithCov> &points,
                std::vector<voxel_map_plus_ns::ptpl> &matches,
                std::vector<V3D> &non_match);

    /**
     * Move the optional local window and remove map entries outside it.
     * This is not called by the PV loop unless local_window_enabled is true.
     */
    void move_window(const V3D &center, const V3D &half_extent);
    void erase_outside_window(const MapWindow &window);
    const MapWindow &window() const { return window_; }

    size_t size() const;
    size_t point_count() const { return map_points_.size(); }
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

    void require_supported_backend() const;
    void erase_outside_voxel_window(const MapWindow &window);
    void erase_outside_voxel_plus_window(const MapWindow &window);
    static bool Inside(const V3D &point, const MapWindow &window);
    void filter_snapshot_to_window(const MapWindow &window);

    MapManagerConfig config_;
    bool initialized_ = false;
    MapWindow window_;
    MapPointList map_points_;

    std::unordered_map<voxel_map_ns::VOXEL_LOC, voxel_map_ns::OctoTree *> voxel_map_;
    std::unordered_map<voxel_map_plus_ns::VOXEL_LOC, voxel_map_plus_ns::UnionFindNode *> voxel_map_plus_;
};

}  // namespace pv_lio_plus

#endif  // PV_LIO_PLUS_MAP_MANAGER_H
