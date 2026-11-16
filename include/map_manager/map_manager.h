/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * PV-LIO-PLUS local-map management facade.
 * Created: 2026-08-09; license notice updated: 2026-08-16.
 */

#ifndef PV_LIO_PLUS_MAP_MANAGER_H
#define PV_LIO_PLUS_MAP_MANAGER_H

/**
 * @file map_manager.h
 * @brief 局部地图统一生命周期接口。
 */

#include <geometry_msgs/Quaternion.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class ImuProcess;
namespace r_voxelmap_ns { class VoxelMap; }
namespace lio { class VoxelMap; }
namespace pv_lio_plus::i_octree_native { class Octree; }

#include "common_lib.h"
#include "map_manager/map_types.h"
#include "map_manager/map_config.h"
#include "native/c3p_voxelmap/c3p_voxel_map_util.hpp"
#include "native/ikdtree/ikd_tree.h"
#include "native/ivox/ivox3d.h"
#include "native/octvox/octvox_map.hpp"
#include "native/voxelmap/voxel_map_util.hpp"
#include "native/voxelmap_plus/voxelmapplus_util.hpp"

namespace pv_lio_plus
{
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

    void initialize(const MapPointList &points, const MapUpdateContext &context = MapUpdateContext{});

    void update(const MapPointList &points, std::uint32_t frame_number = 0,
                const MapUpdateContext &context = MapUpdateContext{});

    void search(const MapPointList &points, PlaneMatchList &matches, std::vector<V3D> &non_match);

    void search(const std::vector<voxel_map_ns::pointWithCov> &points, std::vector<voxel_map_ns::ptpl> &matches,
                std::vector<V3D> &non_match);

    void search(const std::vector<voxel_map_plus_ns::pointWithCov> &points,
                std::vector<voxel_map_plus_ns::ptpl> &matches, std::vector<V3D> &non_match);

    void move_window(const V3D &center, const V3D &half_extent);

    void erase_outside_window(const MapWindow &window);

    const MapWindow &window() const;
    std::uint64_t revision() const;
    size_t size() const;

    size_t point_count() const;
    PointCloudXYZI::Ptr snapshot() const;

    void publish_planes(const ros::Publisher &publisher, int max_voxel_layer) const;

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

    void initialize_voxel_map(const std::vector<voxel_map_ns::pointWithCov> &points);

    void initialize_voxel_map_plus(const std::vector<voxel_map_plus_ns::pointWithCov> &points);

    void update_voxel_map(const std::vector<voxel_map_ns::pointWithCov> &points, std::uint32_t frame_number);

    void update_voxel_map_plus(const std::vector<voxel_map_plus_ns::pointWithCov> &points, std::uint32_t frame_number);

    void initialize_point_backend(const MapPointList &points);

    void update_ikd_tree(const MapPointList &points);

    void update_ivox(const MapPointList &points);

    void initialize_c3p(const MapPointList &points);

    void update_c3p(const MapPointList &points, std::uint32_t frame_number);

    void initialize_octvox(const MapPointList &points);

    void update_octvox(const MapPointList &points);

    void initialize_r_voxelmap(const MapPointList &points);

    void update_r_voxelmap(const MapPointList &points);

    void search_r_voxelmap(const MapPointList &points, PlaneMatchList &matches, std::vector<V3D> &non_match);

    void initialize_hvox(const MapPointList &points, const MapUpdateContext &context);

    void update_hvox(const MapPointList &points, const MapUpdateContext &context);

    void search_hvox(const MapPointList &points, PlaneMatchList &matches, std::vector<V3D> &non_match);

    void initialize_i_octree(const MapPointList &points);

    void update_i_octree(const MapPointList &points);

    void search_i_octree(const MapPointList &points, PlaneMatchList &matches, std::vector<V3D> &non_match);

    void search_point_backend(const MapPointList &points, PlaneMatchList &matches, std::vector<V3D> &non_match);

    void search_ikd_tree(const MapPointList &points, PlaneMatchList &matches, std::vector<V3D> &non_match);

    void search_ivox(const MapPointList &points, PlaneMatchList &matches, std::vector<V3D> &non_match);

    void search_c3p(const MapPointList &points, PlaneMatchList &matches, std::vector<V3D> &non_match);

    void search_octvox(const MapPointList &points, PlaneMatchList &matches, std::vector<V3D> &non_match);

    bool fit_point_plane(const MapPoint &query, const PointVector &neighbors, PlaneMatch &match, MapType backend) const;

    void refresh_octvox_snapshot();

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
    std::uint64_t revision_ = 0;
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
    /// Native Super-LIO OctVox point type used by this manager.
    using OctVoxPoint = Eigen::Vector3f;
    /// Native Super-LIO OctVox map used by this manager.
    using OctVoxBackend = octvox_ns::OctVoxMap<OctVoxPoint>;
    /// Native FAST-LIO2 ikd-tree instance.
    std::unique_ptr<KD_TREE<PointType>> ikd_tree_;
    /// Native Faster-LIO iVox instance.
    std::unique_ptr<IVoxBackend> ivox_;
    /// Native Super-LIO OctVox instance.
    std::unique_ptr<OctVoxBackend> octvox_;
    /// Native R-VoxelMap hierarchy and its LRU-managed root voxels.
    std::unique_ptr<r_voxelmap_ns::VoxelMap> r_voxelmap_;
    std::unique_ptr<lio::VoxelMap> hvox_;
    std::unique_ptr<i_octree_native::Octree> i_octree_;
    /// Native C3P-VoxelMap storage.
    std::unordered_map<c3p_map_ns::VOXEL_LOC, c3p_map_ns::OctoTree *> c3p_voxel_map_;
    /// C3P plane descriptors and merged planes.
    std::unordered_map<c3p_map_ns::PlaneDesc, c3p_map_ns::UnionPlane> c3p_plane_map_;
    /// C3P merged-node lookup table.
    std::unordered_map<int, std::unordered_set<c3p_map_ns::NodeLoc>> c3p_merged_table_;
};

}  // namespace pv_lio_plus

#endif  // PV_LIO_PLUS_MAP_MANAGER_H
