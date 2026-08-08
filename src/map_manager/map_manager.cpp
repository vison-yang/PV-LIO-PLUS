#include "map_manager/map_manager.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace pv_lio_plus
{

MapType ParseMapType(const std::string &name, const bool legacy_voxelmap_plus)
{
    if (name.empty())
    {
        return legacy_voxelmap_plus ? MapType::VoxelMapPlus : MapType::VoxelMap;
    }

    if (name == "voxelmap" || name == "voxel_map" || name == "voxel")
    {
        return MapType::VoxelMap;
    }
    if (name == "voxelmap_plus" || name == "voxel_map_plus" || name == "plus")
    {
        return MapType::VoxelMapPlus;
    }
    if (name == "ikdtree" || name == "ikd_tree" || name == "ikd-tree")
    {
        return MapType::IKDTree;
    }
    if (name == "ivox" || name == "ivox3d")
    {
        return MapType::IVox;
    }
    if (name == "c3p_voxelmap" || name == "c3p-voxelmap" || name == "c3p")
    {
        return MapType::C3PVoxelMap;
    }

    throw std::invalid_argument("unknown mapping/map_type: " + name);
}

const char *MapTypeName(const MapType type)
{
    switch (type)
    {
    case MapType::VoxelMap:
        return "voxelmap";
    case MapType::VoxelMapPlus:
        return "voxelmap_plus";
    case MapType::IKDTree:
        return "ikdtree";
    case MapType::IVox:
        return "ivox";
    case MapType::C3PVoxelMap:
        return "c3p_voxelmap";
    }
    return "unknown";
}

MapManager::MapManager(const MapManagerConfig &config)
{
    configure(config);
}

MapManager::~MapManager()
{
    clear();
}

void MapManager::configure(const MapManagerConfig &config)
{
    if (initialized_)
    {
        clear();
    }

    config_ = config;
    voxel_map_plus_ns::update_size_threshold = config_.plus_update_size_threshold;
    voxel_map_plus_ns::max_points_size       = config_.max_points_size;
    voxel_map_plus_ns::voxel_size            = config_.voxel_size;
    voxel_map_plus_ns::planer_threshold      = config_.plane_threshold;
    voxel_map_plus_ns::sigma_num             = static_cast<int>(config_.sigma_num);
    voxel_map_plus_ns::quater_length         = config_.voxel_size / 4.0;
}

bool MapManager::supports_selected_backend() const
{
    return config_.type == MapType::VoxelMap || config_.type == MapType::VoxelMapPlus;
}

void MapManager::require_supported_backend() const
{
    if (!supports_selected_backend())
    {
        throw std::runtime_error(std::string("map backend '") + MapTypeName(config_.type)
                                 + "' is not enabled in this phase");
    }
}

void MapManager::clear()
{
    for (auto &entry : voxel_map_)
    {
        delete entry.second;
    }
    voxel_map_.clear();

    for (auto &entry : voxel_map_plus_)
    {
        delete entry.second;
    }
    voxel_map_plus_.clear();

    map_points_.clear();
    initialized_ = false;
    window_       = MapWindow();
}

voxel_map_ns::pointWithCov MapManager::ToVoxelPoint(const MapPoint &point)
{
    voxel_map_ns::pointWithCov result;
    result.point_lidar = point.point_lidar;
    result.point_world = point.point_world;
    result.cov_lidar   = point.cov_lidar;
    result.cov_world   = point.cov_world;
    return result;
}

voxel_map_plus_ns::pointWithCov MapManager::ToVoxelPlusPoint(const MapPoint &point)
{
    voxel_map_plus_ns::pointWithCov result;
    result.point_lidar = point.point_lidar;
    result.point_world = point.point_world;
    result.cov_lidar   = point.cov_lidar;
    result.cov_world   = point.cov_world;
    return result;
}

MapPoint MapManager::FromVoxelPoint(const voxel_map_ns::pointWithCov &point)
{
    MapPoint result;
    result.point_lidar = point.point_lidar;
    result.point_world = point.point_world;
    result.cov_lidar   = point.cov_lidar;
    result.cov_world   = point.cov_world;
    return result;
}

MapPoint MapManager::FromVoxelPlusPoint(const voxel_map_plus_ns::pointWithCov &point)
{
    MapPoint result;
    result.point_lidar = point.point_lidar;
    result.point_world = point.point_world;
    result.cov_lidar   = point.cov_lidar;
    result.cov_world   = point.cov_world;
    return result;
}

void MapManager::initialize(const MapPointList &points)
{
    require_supported_backend();
    if (config_.type == MapType::VoxelMap)
    {
        std::vector<voxel_map_ns::pointWithCov> native_points;
        native_points.reserve(points.size());
        for (const auto &point : points)
        {
            native_points.emplace_back(ToVoxelPoint(point));
        }
        initialize(native_points);
    }
    else
    {
        std::vector<voxel_map_plus_ns::pointWithCov> native_points;
        native_points.reserve(points.size());
        for (const auto &point : points)
        {
            native_points.emplace_back(ToVoxelPlusPoint(point));
        }
        initialize(native_points);
    }
}

void MapManager::initialize(const std::vector<voxel_map_ns::pointWithCov> &points)
{
    require_supported_backend();
    if (config_.type != MapType::VoxelMap)
    {
        throw std::logic_error("VoxelMap points passed to a non-VoxelMap backend");
    }
    clear();
    voxel_map_ns::buildVoxelMap(points,
                                config_.voxel_size,
                                config_.max_layer,
                                config_.layer_point_size,
                                config_.max_points_size,
                                config_.max_cov_points_size,
                                config_.plane_threshold,
                                voxel_map_);
    map_points_.reserve(points.size());
    for (const auto &point : points)
    {
        map_points_.emplace_back(FromVoxelPoint(point));
    }
    initialized_ = true;
}

void MapManager::initialize(const std::vector<voxel_map_plus_ns::pointWithCov> &points)
{
    require_supported_backend();
    if (config_.type != MapType::VoxelMapPlus)
    {
        throw std::logic_error("VoxelMap++ points passed to a non-VoxelMap++ backend");
    }
    clear();
    voxel_map_plus_ns::BuildVoxelMap(points, voxel_map_plus_);
    map_points_.reserve(points.size());
    for (const auto &point : points)
    {
        map_points_.emplace_back(FromVoxelPlusPoint(point));
    }
    initialized_ = true;
}

void MapManager::update(const MapPointList &points, const std::uint32_t frame_number)
{
    require_supported_backend();
    if (config_.type == MapType::VoxelMap)
    {
        std::vector<voxel_map_ns::pointWithCov> native_points;
        native_points.reserve(points.size());
        for (const auto &point : points)
        {
            native_points.emplace_back(ToVoxelPoint(point));
        }
        update(native_points, frame_number);
    }
    else
    {
        std::vector<voxel_map_plus_ns::pointWithCov> native_points;
        native_points.reserve(points.size());
        for (const auto &point : points)
        {
            native_points.emplace_back(ToVoxelPlusPoint(point));
        }
        update(native_points, frame_number);
    }
}

void MapManager::update(const std::vector<voxel_map_ns::pointWithCov> &points,
                        const std::uint32_t /*frame_number*/)
{
    require_supported_backend();
    if (config_.type != MapType::VoxelMap)
    {
        throw std::logic_error("VoxelMap points passed to a non-VoxelMap backend");
    }
    if (!initialized_)
    {
        initialize(points);
        return;
    }

    voxel_map_ns::updateVoxelMapOMP(points,
                                    config_.voxel_size,
                                    config_.max_layer,
                                    config_.layer_point_size,
                                    config_.max_points_size,
                                    config_.max_cov_points_size,
                                    config_.plane_threshold,
                                    voxel_map_);
    for (const auto &point : points)
    {
        map_points_.emplace_back(FromVoxelPoint(point));
    }
}

void MapManager::update(const std::vector<voxel_map_plus_ns::pointWithCov> &points,
                        const std::uint32_t /*frame_number*/)
{
    require_supported_backend();
    if (config_.type != MapType::VoxelMapPlus)
    {
        throw std::logic_error("VoxelMap++ points passed to a non-VoxelMap++ backend");
    }
    if (!initialized_)
    {
        initialize(points);
        return;
    }

    voxel_map_plus_ns::UpdateVoxelMap(points, voxel_map_plus_);
    for (const auto &point : points)
    {
        map_points_.emplace_back(FromVoxelPlusPoint(point));
    }
}

PlaneMatch MapManager::FromVoxelMatch(const voxel_map_ns::ptpl &match)
{
    PlaneMatch result;
    result.map_type   = MapType::VoxelMap;
    result.point      = match.point;
    result.point_world = match.point_world;
    result.normal     = match.normal;
    result.center     = match.center;
    result.plane_cov  = match.plane_cov;
    result.point_cov  = match.cov_world;
    result.d          = match.d;
    result.layer      = match.layer;
    result.omega      = match.normal;
    result.omega_norm = match.normal.norm();
    result.distance   = match.normal.dot(match.point_world) + match.d;
    return result;
}

PlaneMatch MapManager::FromVoxelPlusMatch(const voxel_map_plus_ns::ptpl &match)
{
    PlaneMatch result;
    result.map_type       = MapType::VoxelMapPlus;
    result.point          = match.point;
    result.point_world    = match.point_world;
    result.omega          = match.omega;
    result.omega_norm     = match.omega_norm;
    result.distance       = match.dist;
    result.plane_cov.setZero();
    result.plane_cov.block<3, 3>(0, 0) = match.plane_cov;
    result.point_cov      = match.point_cov;
    result.main_direction = match.main_direction;
    if (result.omega_norm > std::numeric_limits<double>::epsilon())
    {
        result.normal = result.omega / result.omega_norm;
    }
    return result;
}

void MapManager::search(const MapPointList &points, PlaneMatchList &matches,
                        std::vector<V3D> &non_match)
{
    require_supported_backend();
    matches.clear();
    non_match.clear();
    matches.reserve(points.size());

    if (config_.type == MapType::VoxelMap)
    {
        std::vector<voxel_map_ns::pointWithCov> native_points;
        std::vector<voxel_map_ns::ptpl> native_matches;
        std::vector<V3D> native_non_match;
        native_points.reserve(points.size());
        for (const auto &point : points)
        {
            native_points.emplace_back(ToVoxelPoint(point));
        }
        search(native_points, native_matches, native_non_match);
        for (const auto &match : native_matches)
        {
            matches.emplace_back(FromVoxelMatch(match));
        }
        non_match = std::move(native_non_match);
    }
    else
    {
        std::vector<voxel_map_plus_ns::pointWithCov> native_points;
        std::vector<voxel_map_plus_ns::ptpl> native_matches;
        std::vector<V3D> native_non_match;
        native_points.reserve(points.size());
        for (const auto &point : points)
        {
            native_points.emplace_back(ToVoxelPlusPoint(point));
        }
        search(native_points, native_matches, native_non_match);
        for (const auto &match : native_matches)
        {
            matches.emplace_back(FromVoxelPlusMatch(match));
        }
        non_match = std::move(native_non_match);
    }
}

void MapManager::search(const std::vector<voxel_map_ns::pointWithCov> &points,
                        std::vector<voxel_map_ns::ptpl> &matches,
                        std::vector<V3D> &non_match)
{
    require_supported_backend();
    if (config_.type != MapType::VoxelMap)
    {
        throw std::logic_error("VoxelMap points passed to a non-VoxelMap backend");
    }
    voxel_map_ns::BuildResidualListOMP(voxel_map_,
                                       config_.voxel_size,
                                       config_.sigma_num,
                                       config_.max_layer,
                                       points,
                                       matches,
                                       non_match);
}

void MapManager::search(const std::vector<voxel_map_plus_ns::pointWithCov> &points,
                        std::vector<voxel_map_plus_ns::ptpl> &matches,
                        std::vector<V3D> &non_match)
{
    require_supported_backend();
    if (config_.type != MapType::VoxelMapPlus)
    {
        throw std::logic_error("VoxelMap++ points passed to a non-VoxelMap++ backend");
    }
    voxel_map_plus_ns::BuildResidualListOMP(voxel_map_plus_, points, matches, non_match);
}

bool MapManager::Inside(const V3D &point, const MapWindow &window)
{
    return window.initialized &&
           (point.array() >= window.min_bound.array()).all() &&
           (point.array() <= window.max_bound.array()).all();
}

void MapManager::move_window(const V3D &center, const V3D &half_extent)
{
    if (!config_.local_window_enabled)
    {
        return;
    }

    MapWindow next_window;
    next_window.initialized = true;
    next_window.min_bound   = center - half_extent;
    next_window.max_bound   = center + half_extent;
    erase_outside_window(next_window);
    window_ = next_window;
}

void MapManager::erase_outside_window(const MapWindow &window)
{
    require_supported_backend();
    if (!window.initialized)
    {
        return;
    }

    if (config_.type == MapType::VoxelMap)
    {
        erase_outside_voxel_window(window);
    }
    else
    {
        erase_outside_voxel_plus_window(window);
    }
    filter_snapshot_to_window(window);
}

void MapManager::erase_outside_voxel_window(const MapWindow &window)
{
    for (auto iter = voxel_map_.begin(); iter != voxel_map_.end();)
    {
        const auto *tree = iter->second;
        const V3D center(tree->voxel_center_[0], tree->voxel_center_[1], tree->voxel_center_[2]);
        if (!Inside(center, window))
        {
            delete tree;
            iter = voxel_map_.erase(iter);
        }
        else
        {
            ++iter;
        }
    }
}

void MapManager::erase_outside_voxel_plus_window(const MapWindow &window)
{
    for (auto iter = voxel_map_plus_.begin(); iter != voxel_map_plus_.end();)
    {
        const auto *node = iter->second;
        const V3D center(node->voxel_center_[0], node->voxel_center_[1], node->voxel_center_[2]);
        if (!Inside(center, window))
        {
            delete node;
            iter = voxel_map_plus_.erase(iter);
        }
        else
        {
            ++iter;
        }
    }
}

void MapManager::filter_snapshot_to_window(const MapWindow &window)
{
    MapPointList filtered;
    filtered.reserve(map_points_.size());
    for (const auto &point : map_points_)
    {
        if (Inside(point.point_world, window))
        {
            filtered.emplace_back(point);
        }
    }
    map_points_.swap(filtered);
}

size_t MapManager::size() const
{
    if (config_.type == MapType::VoxelMap)
    {
        return voxel_map_.size();
    }
    if (config_.type == MapType::VoxelMapPlus)
    {
        return voxel_map_plus_.size();
    }
    return 0;
}

PointCloudXYZI::Ptr MapManager::snapshot() const
{
    PointCloudXYZI::Ptr result(new PointCloudXYZI());
    result->reserve(map_points_.size());
    for (const auto &point : map_points_)
    {
        PointType output;
        output.x         = static_cast<float>(point.point_world.x());
        output.y         = static_cast<float>(point.point_world.y());
        output.z         = static_cast<float>(point.point_world.z());
        output.intensity = static_cast<float>(point.point_world.norm());
        result->push_back(output);
    }
    return result;
}

void MapManager::publish_planes(const ros::Publisher &publisher, const int max_voxel_layer) const
{
    if (config_.type == MapType::VoxelMap)
    {
        voxel_map_ns::pubVoxelMap(voxel_map_, max_voxel_layer, publisher);
    }
    else if (config_.type == MapType::VoxelMapPlus)
    {
        voxel_map_plus_ns::pubVoxelMap(voxel_map_plus_, publisher);
    }
}

}  // namespace pv_lio_plus
