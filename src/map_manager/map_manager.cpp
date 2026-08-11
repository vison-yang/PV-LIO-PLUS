/**
 * @file map_manager.cpp
 * @brief Backend dispatch and local-window implementation for MapManager.
 */

#include "map_manager/map_manager.h"

// ikd-tree is a header/template implementation.  Keep the native source in
// the PV translation unit so the imported backend is self-contained.
#include "map_manager/native/ikdtree/ikd_tree.cpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <numeric>

namespace pv_lio_plus
{

/**
 * @brief Parses a configured local-map backend name.
 * @param name Backend name; an empty name selects the original VoxelMap.
 * @return The selected backend type.
 * @throws std::invalid_argument If @p name is not recognized.
 */
MapType ParseMapType(const std::string &name)
{
    if (name.empty())
    {
        return MapType::VoxelMap;
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

/**
 * @brief Returns the canonical configuration name of a backend.
 * @param type Backend type to name.
 * @return Stable configuration string for @p type.
 */
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

/** @brief Constructs an unconfigured map manager. */
MapManager::MapManager() = default;

/**
 * @brief Constructs and configures a map manager.
 * @param config Backend and map-maintenance configuration.
 */
MapManager::MapManager(const MapManagerConfig &config)
{
    configure(config);
}

/** @brief Releases all backend-owned map data. */
MapManager::~MapManager()
{
    clear();
}

/**
 * @brief Applies configuration and resets any initialized map.
 * @param config Backend and map-maintenance configuration.
 */
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
    configure_point_backends();
}

/**
 * @brief Returns the active backend configuration.
 * @return Current manager configuration.
 */
const MapManagerConfig &MapManager::config() const
{
    return config_;
}

/**
 * @brief Returns the selected backend type.
 * @return Active backend type.
 */
MapType MapManager::type() const
{
    return config_.type;
}

/**
 * @brief Reports whether a map has been initialized.
 * @return True after at least one point batch initialized the backend.
 */
bool MapManager::initialized() const
{
    return initialized_;
}

/**
 * @brief Checks whether the selected backend is supported.
 * @return True when the backend has an implementation in this package.
 */
bool MapManager::supports_selected_backend() const
{
    return config_.type == MapType::VoxelMap || config_.type == MapType::VoxelMapPlus ||
           config_.type == MapType::IKDTree || config_.type == MapType::IVox ||
           config_.type == MapType::C3PVoxelMap;
}

/**
 * @brief Throws when the selected backend is unsupported.
 * @throws std::runtime_error If no implementation handles the configured type.
 */
void MapManager::require_supported_backend() const
{
    if (!supports_selected_backend())
    {
        throw std::runtime_error(std::string("map backend '") + MapTypeName(config_.type) + "' is not enabled");
    }
}

/** @brief Creates or resets the selected point-map backend. */
void MapManager::configure_point_backends()
{
    ikd_tree_.reset();
    ivox_.reset();

    if (config_.type == MapType::IKDTree)
    {
        ikd_tree_ = std::make_unique<KD_TREE<PointType>>(static_cast<float>(config_.ikd_delete_param),
                                                         static_cast<float>(config_.ikd_balance_param),
                                                         static_cast<float>(config_.ikd_box_length));
        ikd_tree_->set_downsample_param(static_cast<float>(config_.point_map_downsample_size));
    }
    else if (config_.type == MapType::IVox)
    {
        IVoxBackend::Options options;
        options.resolution_ = static_cast<float>(config_.ivox_resolution);
        options.capacity_   = std::max<std::size_t>(config_.ivox_capacity, 1);
        switch (config_.ivox_nearby_type)
        {
        case 0:
            options.nearby_type_ = IVoxBackend::NearbyType::CENTER;
            break;
        case 6:
            options.nearby_type_ = IVoxBackend::NearbyType::NEARBY6;
            break;
        case 26:
            options.nearby_type_ = IVoxBackend::NearbyType::NEARBY26;
            break;
        case 18:
        default:
            options.nearby_type_ = IVoxBackend::NearbyType::NEARBY18;
            break;
        }
        ivox_ = std::make_unique<IVoxBackend>(options);
    }
}

/** @brief Clears all backend data, retained points, and window state. */
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

    for (auto &entry : c3p_voxel_map_)
    {
        delete entry.second;
    }
    c3p_voxel_map_.clear();
    c3p_plane_map_.clear();
    c3p_merged_table_.clear();

    ivox_.reset();
    ikd_tree_.reset();

    map_points_.clear();
    initialized_ = false;
    window_       = MapWindow();
}

/**
 * @brief Converts a common point to the VoxelMap representation.
 * @param point Common LiDAR/world point and covariance.
 * @return Native VoxelMap point.
 */
voxel_map_ns::pointWithCov MapManager::ToVoxelPoint(const MapPoint &point)
{
    voxel_map_ns::pointWithCov result;
    result.point_lidar = point.point_lidar;
    result.point_world = point.point_world;
    result.cov_lidar   = point.cov_lidar;
    result.cov_world   = point.cov_world;
    return result;
}

/**
 * @brief Converts a common point to the VoxelMap++ representation.
 * @param point Common LiDAR/world point and covariance.
 * @return Native VoxelMap++ point.
 */
voxel_map_plus_ns::pointWithCov MapManager::ToVoxelPlusPoint(const MapPoint &point)
{
    voxel_map_plus_ns::pointWithCov result;
    result.point_lidar = point.point_lidar;
    result.point_world = point.point_world;
    result.cov_lidar   = point.cov_lidar;
    result.cov_world   = point.cov_world;
    return result;
}

/**
 * @brief Converts a native VoxelMap point to the common representation.
 * @param point Native VoxelMap point.
 * @return Common point with both covariance frames preserved.
 */
MapPoint MapManager::FromVoxelPoint(const voxel_map_ns::pointWithCov &point)
{
    MapPoint result;
    result.point_lidar = point.point_lidar;
    result.point_world = point.point_world;
    result.cov_lidar   = point.cov_lidar;
    result.cov_world   = point.cov_world;
    return result;
}

/**
 * @brief Converts a native VoxelMap++ point to the common representation.
 * @param point Native VoxelMap++ point.
 * @return Common point with both covariance frames preserved.
 */
MapPoint MapManager::FromVoxelPlusPoint(const voxel_map_plus_ns::pointWithCov &point)
{
    MapPoint result;
    result.point_lidar = point.point_lidar;
    result.point_world = point.point_world;
    result.cov_lidar   = point.cov_lidar;
    result.cov_world   = point.cov_world;
    return result;
}

/**
 * @brief Converts world coordinates to the iVox/ikd-tree point type.
 * @param point World-frame position.
 * @param intensity Point intensity to store.
 * @return Native point-map point.
 */
PointType MapManager::ToPointType(const V3D &point, const float intensity)
{
    PointType result;
    result.x         = static_cast<float>(point.x());
    result.y         = static_cast<float>(point.y());
    result.z         = static_cast<float>(point.z());
    result.intensity = intensity;
    return result;
}

/**
 * @brief Converts a point-map point to the common representation.
 * @param point Native world-frame point.
 * @return Common point with its world position populated.
 */
MapPoint MapManager::FromPointType(const PointType &point)
{
    MapPoint result;
    result.point_world = V3D(point.x, point.y, point.z);
    return result;
}

/**
 * @brief Converts a native C3P match to the common contract.
 * @param match Native C3P point-to-plane match.
 * @return Backend-independent match.
 */
PlaneMatch MapManager::FromC3PMatch(const c3p_map_ns::ptpl &match)
{
    PlaneMatch result;
    result.map_type    = MapType::C3PVoxelMap;
    result.point       = match.point;
    result.point_world = match.point_world;
    result.normal      = match.normal;
    result.center      = match.center;
    result.plane_cov   = match.plane_cov;
    result.point_cov     = match.point_cov;
    result.omega         = match.normal;
    result.omega_norm    = match.normal.norm();
    result.d             = match.d;
    result.distance      = match.normal.dot(match.point_world) + match.d;
    result.layer         = match.layer;
    return result;
}

/**
 * @brief Fits and validates a plane from point-map neighbors.
 * @param query Query point and covariance.
 * @param neighbors Nearby world-frame points.
 * @param match Output backend-independent match.
 * @param backend Point-map backend applying its native residual gate.
 * @return True when a valid plane and backend-specific gate are obtained.
 */
bool MapManager::fit_point_plane(const MapPoint &query,
                                 const PointVector &neighbors,
                                 PlaneMatch &match,
                                 const MapType backend) const
{
    if (neighbors.size() < NUM_MATCH_POINTS)
    {
        return false;
    }

    VF(4) plane_coeff;
    if (!esti_plane(plane_coeff, neighbors, static_cast<float>(config_.plane_fit_threshold)))
    {
        return false;
    }

    const V3D point_world = query.point_world;
    const V3D normal(plane_coeff(0), plane_coeff(1), plane_coeff(2));
    const double pd2 = normal.dot(point_world) + plane_coeff(3);

    // FAST-LIO2 rejects planes whose residual is too large relative to the
    // point range; keep that native gate for ikd-tree.  Faster-LIO uses its
    // equivalent 81*residual^2 test for iVox.
    if (backend == MapType::IKDTree)
    {
        const double score = 1.0 - 0.9 * std::abs(pd2) / std::sqrt(query.point_lidar.norm());
        if (score <= 0.9)
        {
            return false;
        }
    }
    else if (backend == MapType::IVox && query.point_lidar.norm() <= 81.0 * pd2 * pd2)
    {
        return false;
    }

    match.map_type    = backend;
    match.point       = query.point_lidar;
    match.point_world = point_world;
    match.normal      = normal;
    match.omega       = normal;
    match.omega_norm  = normal.norm();
    match.d           = plane_coeff(3);
    match.distance    = pd2;
    match.point_cov   = query.cov_world;
    match.plane_cov.setZero();
    match.layer = 0;

    match.center.setZero();
    for (int i = 0; i < NUM_MATCH_POINTS; ++i)
    {
        match.center += V3D(neighbors[i].x, neighbors[i].y, neighbors[i].z);
    }
    match.center /= static_cast<double>(NUM_MATCH_POINTS);
    return true;
}

/**
 * @brief Initializes the selected backend from common map points.
 * @param points Initial points in LiDAR and world frames.
 * @throws std::logic_error If the selected backend cannot be initialized.
 */
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
    else if (config_.type == MapType::VoxelMapPlus)
    {
        std::vector<voxel_map_plus_ns::pointWithCov> native_points;
        native_points.reserve(points.size());
        for (const auto &point : points)
        {
            native_points.emplace_back(ToVoxelPlusPoint(point));
        }
        initialize(native_points);
    }
    else if (config_.type == MapType::C3PVoxelMap)
    {
        initialize_c3p(points);
    }
    else
    {
        initialize_point_backend(points);
    }
}

/**
 * @brief Initializes ikd-tree or iVox from common points.
 * @param points Initial world-frame points.
 * @throws std::logic_error If a non-point backend is selected.
 */
void MapManager::initialize_point_backend(const MapPointList &points)
{
    if (config_.type != MapType::IKDTree && config_.type != MapType::IVox)
    {
        throw std::logic_error("point backend initialization requested for a non-point backend");
    }

    clear();
    configure_point_backends();

    PointVector native_points;
    native_points.reserve(points.size());
    for (const auto &point : points)
    {
        native_points.emplace_back(ToPointType(point.point_world));
    }

    if (config_.type == MapType::IKDTree)
    {
        if (ikd_tree_ && !native_points.empty())
        {
            ikd_tree_->Build(native_points);
        }
    }
    else if (ivox_ && !native_points.empty())
    {
        ivox_->AddPoints(native_points);
    }

    map_points_ = points;
    initialized_ = true;
}

/**
 * @brief Initializes the C3P-VoxelMap backend.
 * @param points Initial points and world-frame covariances.
 * @throws std::logic_error If C3P-VoxelMap is not selected.
 */
void MapManager::initialize_c3p(const MapPointList &points)
{
    if (config_.type != MapType::C3PVoxelMap)
    {
        throw std::logic_error("C3P initialization requested for a non-C3P backend");
    }

    clear();

    std::vector<c3p_map_ns::pointWithCov> native_points;
    native_points.reserve(points.size());
    for (const auto &point : points)
    {
        c3p_map_ns::pointWithCov native_point;
        native_point.point       = point.point_world;
        native_point.point_world = point.point_world;
        native_point.cov         = point.cov_world;
        native_points.emplace_back(native_point);
    }

    std::vector<int> layer_point_size = config_.layer_point_size;
    const std::size_t required_layers = static_cast<std::size_t>(std::max(0, config_.max_layer) + 1);
    if (layer_point_size.empty())
    {
        layer_point_size.assign(required_layers, 5);
    }
    else if (layer_point_size.size() < required_layers)
    {
        layer_point_size.resize(required_layers, layer_point_size.back());
    }

    c3p_map_ns::buildVoxelMap(config_.c3p_enable_voxel_merging,
                              native_points,
                              static_cast<float>(config_.voxel_size),
                              config_.max_layer,
                              layer_point_size,
                              static_cast<float>(config_.plane_threshold),
                              c3p_voxel_map_,
                              c3p_plane_map_,
                              c3p_merged_table_,
                              config_.c3p_merge_theta_thresh,
                              config_.c3p_merge_dist_thresh,
                              config_.c3p_merge_cov_min_eigen_val_thresh,
                              config_.c3p_merge_x_coord_diff_thresh,
                              config_.c3p_merge_y_coord_diff_thresh);

    map_points_ = points;
    initialized_ = true;
}

/**
 * @brief Initializes VoxelMap from native points.
 * @param points Native VoxelMap points.
 * @throws std::logic_error If VoxelMap is not selected.
 */
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

/**
 * @brief Initializes VoxelMap++ from native points.
 * @param points Native VoxelMap++ points.
 * @throws std::logic_error If VoxelMap++ is not selected.
 */
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

/**
 * @brief Inserts a frame of common points into the selected backend.
 * @param points New points in LiDAR and world frames.
 * @param frame_number Backend update frame number.
 * @throws std::logic_error If the selected backend is unsupported.
 */
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
    else if (config_.type == MapType::VoxelMapPlus)
    {
        std::vector<voxel_map_plus_ns::pointWithCov> native_points;
        native_points.reserve(points.size());
        for (const auto &point : points)
        {
            native_points.emplace_back(ToVoxelPlusPoint(point));
        }
        update(native_points, frame_number);
    }
    else if (config_.type == MapType::C3PVoxelMap)
    {
        if (!initialized_)
        {
            initialize_c3p(points);
        }
        else
        {
            update_c3p(points, frame_number);
        }
    }
    else
    {
        if (!initialized_)
        {
            initialize_point_backend(points);
        }
        else if (config_.type == MapType::IKDTree)
        {
            update_ikd_tree(points);
        }
        else
        {
            update_ivox(points);
        }
    }
}

/**
 * @brief Updates the ikd-tree backend with a frame of points.
 * @param points New points in LiDAR and world frames.
 */
void MapManager::update_ikd_tree(const MapPointList &points)
{
    if (!ikd_tree_)
    {
        configure_point_backends();
    }

    PointVector points_to_add;
    PointVector point_no_need_downsample;
    points_to_add.reserve(points.size());
    point_no_need_downsample.reserve(points.size());

    const double downsample_size = std::max(config_.point_map_downsample_size, 1e-6);
    const int nearest_count = std::max(config_.nearest_point_count, NUM_MATCH_POINTS);
    const auto squared_distance = [](const PointType &lhs, const PointType &rhs) {
        const double dx = static_cast<double>(lhs.x) - rhs.x;
        const double dy = static_cast<double>(lhs.y) - rhs.y;
        const double dz = static_cast<double>(lhs.z) - rhs.z;
        return dx * dx + dy * dy + dz * dz;
    };

    for (const auto &point : points)
    {
        const PointType point_world = ToPointType(point.point_world);
        PointVector points_near;
        std::vector<float> point_search_sq_dist;
        if (ikd_tree_ && ikd_tree_->validnum() > 0)
        {
            ikd_tree_->Nearest_Search(point_world, nearest_count, points_near, point_search_sq_dist);
        }

        if (points_near.empty())
        {
            points_to_add.emplace_back(point_world);
            continue;
        }

        PointType mid_point;
        mid_point.x = static_cast<float>(std::floor(point_world.x / downsample_size) * downsample_size +
                                         0.5 * downsample_size);
        mid_point.y = static_cast<float>(std::floor(point_world.y / downsample_size) * downsample_size +
                                         0.5 * downsample_size);
        mid_point.z = static_cast<float>(std::floor(point_world.z / downsample_size) * downsample_size +
                                         0.5 * downsample_size);

        const double point_to_center = squared_distance(point_world, mid_point);
        if (std::abs(points_near.front().x - mid_point.x) > 0.5 * downsample_size &&
            std::abs(points_near.front().y - mid_point.y) > 0.5 * downsample_size &&
            std::abs(points_near.front().z - mid_point.z) > 0.5 * downsample_size)
        {
            point_no_need_downsample.emplace_back(point_world);
            continue;
        }

        bool need_add = true;
        const std::size_t compare_count = std::min<std::size_t>(NUM_MATCH_POINTS, points_near.size());
        for (std::size_t i = 0; i < compare_count; ++i)
        {
            if (squared_distance(points_near[i], mid_point) < point_to_center)
            {
                need_add = false;
                break;
            }
        }
        if (need_add)
        {
            points_to_add.emplace_back(point_world);
        }
    }

    if (ikd_tree_)
    {
        ikd_tree_->Add_Points(points_to_add, true);
        ikd_tree_->Add_Points(point_no_need_downsample, false);
    }

    for (const auto &point : points_to_add)
    {
        map_points_.emplace_back(FromPointType(point));
    }
    for (const auto &point : point_no_need_downsample)
    {
        map_points_.emplace_back(FromPointType(point));
    }
}

/**
 * @brief Updates the iVox backend with a frame of points.
 * @param points New points in LiDAR and world frames.
 */
void MapManager::update_ivox(const MapPointList &points)
{
    if (!ivox_)
    {
        configure_point_backends();
    }

    PointVector points_to_add;
    PointVector point_no_need_downsample;
    points_to_add.reserve(points.size());
    point_no_need_downsample.reserve(points.size());

    const double downsample_size = std::max(config_.point_map_downsample_size, 1e-6);
    const int nearest_count = std::max(config_.nearest_point_count, NUM_MATCH_POINTS);
    const auto squared_distance = [](const PointType &lhs, const PointType &rhs) {
        const double dx = static_cast<double>(lhs.x) - rhs.x;
        const double dy = static_cast<double>(lhs.y) - rhs.y;
        const double dz = static_cast<double>(lhs.z) - rhs.z;
        return dx * dx + dy * dy + dz * dz;
    };

    for (const auto &point : points)
    {
        const PointType point_world = ToPointType(point.point_world);
        PointVector points_near;
        const bool has_near = ivox_ && ivox_->GetClosestPoint(point_world,
                                                               points_near,
                                                               nearest_count,
                                                               config_.nearest_max_range);
        if (!has_near || points_near.empty())
        {
            points_to_add.emplace_back(point_world);
            continue;
        }

        PointType mid_point;
        mid_point.x = static_cast<float>(std::floor(point_world.x / downsample_size) * downsample_size +
                                         0.5 * downsample_size);
        mid_point.y = static_cast<float>(std::floor(point_world.y / downsample_size) * downsample_size +
                                         0.5 * downsample_size);
        mid_point.z = static_cast<float>(std::floor(point_world.z / downsample_size) * downsample_size +
                                         0.5 * downsample_size);

        const double point_to_center = squared_distance(point_world, mid_point);
        if (std::abs(points_near.front().x - mid_point.x) > 0.5 * downsample_size &&
            std::abs(points_near.front().y - mid_point.y) > 0.5 * downsample_size &&
            std::abs(points_near.front().z - mid_point.z) > 0.5 * downsample_size)
        {
            point_no_need_downsample.emplace_back(point_world);
            continue;
        }

        bool need_add = true;
        const std::size_t compare_count = std::min<std::size_t>(NUM_MATCH_POINTS, points_near.size());
        for (std::size_t i = 0; i < compare_count; ++i)
        {
            if (squared_distance(points_near[i], mid_point) < point_to_center + 1e-6)
            {
                need_add = false;
                break;
            }
        }
        if (need_add)
        {
            points_to_add.emplace_back(point_world);
        }
    }

    if (ivox_)
    {
        ivox_->AddPoints(points_to_add);
        ivox_->AddPoints(point_no_need_downsample);
    }

    for (const auto &point : points_to_add)
    {
        map_points_.emplace_back(FromPointType(point));
    }
    for (const auto &point : point_no_need_downsample)
    {
        map_points_.emplace_back(FromPointType(point));
    }
}

/**
 * @brief Updates the C3P-VoxelMap backend.
 * @param points New points and world-frame covariances.
 * @param frame_number C3P update frame number.
 */
void MapManager::update_c3p(const MapPointList &points, const std::uint32_t frame_number)
{
    if (!initialized_)
    {
        initialize_c3p(points);
        return;
    }

    std::vector<c3p_map_ns::pointWithCov> native_points;
    native_points.reserve(points.size());
    for (const auto &point : points)
    {
        c3p_map_ns::pointWithCov native_point;
        native_point.point       = point.point_world;
        native_point.point_world = point.point_world;
        native_point.cov         = point.cov_world;
        native_points.emplace_back(native_point);
    }

    std::vector<int> layer_point_size = config_.layer_point_size;
    const std::size_t required_layers = static_cast<std::size_t>(std::max(0, config_.max_layer) + 1);
    if (layer_point_size.empty())
    {
        layer_point_size.assign(required_layers, 5);
    }
    else if (layer_point_size.size() < required_layers)
    {
        layer_point_size.resize(required_layers, layer_point_size.back());
    }

    c3p_map_ns::updateVoxelMap(config_.c3p_enable_voxel_merging,
                               native_points,
                               static_cast<float>(config_.voxel_size),
                               config_.max_layer,
                               layer_point_size,
                               static_cast<float>(config_.plane_threshold),
                               c3p_voxel_map_,
                               c3p_plane_map_,
                               c3p_merged_table_,
                               frame_number,
                               config_.c3p_merge_theta_thresh,
                               config_.c3p_merge_dist_thresh,
                               config_.c3p_merge_cov_min_eigen_val_thresh,
                               config_.c3p_merge_x_coord_diff_thresh,
                               config_.c3p_merge_y_coord_diff_thresh);

    map_points_.insert(map_points_.end(), points.begin(), points.end());
}

/**
 * @brief Updates VoxelMap from native points.
 * @param points New native VoxelMap points.
 * @param frame_number Unused by VoxelMap; retained for API uniformity.
 */
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

/**
 * @brief Updates VoxelMap++ from native points.
 * @param points New native VoxelMap++ points.
 * @param frame_number Unused by VoxelMap++; retained for API uniformity.
 */
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

/**
 * @brief Converts a native VoxelMap match to the common contract.
 * @param match Native VoxelMap point-to-plane match.
 * @return Backend-independent match.
 */
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

/**
 * @brief Converts a native VoxelMap++ match to the common contract.
 * @param match Native VoxelMap++ point-to-plane match.
 * @return Backend-independent match.
 */
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

/**
 * @brief Searches the selected backend and returns common matches.
 * @param points Query points in LiDAR and world frames.
 * @param matches Accepted matches; cleared before filling.
 * @param non_match World-frame queries without a valid match.
 */
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
    else if (config_.type == MapType::VoxelMapPlus)
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
    else
    {
        search_point_backend(points, matches, non_match);
    }
}

/**
 * @brief Dispatches point-map search to the selected backend.
 * @param points Query points in LiDAR and world frames.
 * @param matches Output common point-to-plane matches.
 * @param non_match Output world-frame queries without a match.
 */
void MapManager::search_point_backend(const MapPointList &points,
                                      PlaneMatchList &matches,
                                      std::vector<V3D> &non_match)
{
    if (config_.type == MapType::IKDTree)
    {
        search_ikd_tree(points, matches, non_match);
    }
    else if (config_.type == MapType::IVox)
    {
        search_ivox(points, matches, non_match);
    }
    else if (config_.type == MapType::C3PVoxelMap)
    {
        search_c3p(points, matches, non_match);
    }
    else
    {
        throw std::logic_error("point-backend search requested for a PV voxel backend");
    }
}

/**
 * @brief Searches ikd-tree neighbors and fits planes.
 * @param points Query points in LiDAR and world frames.
 * @param matches Output accepted matches.
 * @param non_match Output world-frame queries without a valid match.
 */
void MapManager::search_ikd_tree(const MapPointList &points,
                                 PlaneMatchList &matches,
                                 std::vector<V3D> &non_match)
{
    matches.clear();
    non_match.clear();
    matches.reserve(points.size());
    non_match.reserve(points.size());

    const int nearest_count = std::max(config_.nearest_point_count, NUM_MATCH_POINTS);
    for (const auto &point : points)
    {
        PointVector neighbors;
        std::vector<float> point_search_sq_dist;
        if (ikd_tree_ && ikd_tree_->validnum() > 0)
        {
            ikd_tree_->Nearest_Search(ToPointType(point.point_world),
                                      nearest_count,
                                      neighbors,
                                      point_search_sq_dist);
        }

        if (neighbors.size() < NUM_MATCH_POINTS ||
            (point_search_sq_dist.size() >= NUM_MATCH_POINTS &&
             point_search_sq_dist[NUM_MATCH_POINTS - 1] > config_.nearest_max_range))
        {
            non_match.emplace_back(point.point_world);
            continue;
        }

        PlaneMatch match;
        if (fit_point_plane(point, neighbors, match, MapType::IKDTree))
        {
            matches.emplace_back(match);
        }
        else
        {
            non_match.emplace_back(point.point_world);
        }
    }
}

/**
 * @brief Searches iVox neighbors and fits planes.
 * @param points Query points in LiDAR and world frames.
 * @param matches Output accepted matches.
 * @param non_match Output world-frame queries without a valid match.
 */
void MapManager::search_ivox(const MapPointList &points,
                             PlaneMatchList &matches,
                             std::vector<V3D> &non_match)
{
    matches.clear();
    non_match.clear();
    matches.reserve(points.size());
    non_match.reserve(points.size());

    const int nearest_count = std::max(config_.nearest_point_count, NUM_MATCH_POINTS);
    for (const auto &point : points)
    {
        PointVector neighbors;
        const bool found = ivox_ && ivox_->GetClosestPoint(ToPointType(point.point_world),
                                                           neighbors,
                                                           nearest_count,
                                                           config_.nearest_max_range);
        PlaneMatch match;
        if (found && fit_point_plane(point, neighbors, match, MapType::IVox))
        {
            matches.emplace_back(match);
        }
        else
        {
            non_match.emplace_back(point.point_world);
        }
    }
}

/**
 * @brief Searches C3P-VoxelMap with its native residual builder.
 * @param points Query points in LiDAR and world frames.
 * @param matches Output common point-to-plane matches.
 * @param non_match Output world-frame queries without a valid match.
 */
void MapManager::search_c3p(const MapPointList &points,
                            PlaneMatchList &matches,
                            std::vector<V3D> &non_match)
{
    std::vector<c3p_map_ns::pointWithCov> native_points;
    std::vector<c3p_map_ns::ptpl> native_matches;
    native_points.reserve(points.size());
    for (const auto &point : points)
    {
        c3p_map_ns::pointWithCov native_point;
        // C3P uses point_world for voxel lookup but returns point as the
        // original lidar-frame query point for the state Jacobian.
        native_point.point       = point.point_lidar;
        native_point.point_world = point.point_world;
        native_point.cov         = point.cov_world;
        native_points.emplace_back(native_point);
    }

    c3p_map_ns::BuildResidualListOMP(c3p_voxel_map_,
                                     config_.voxel_size,
                                     config_.sigma_num,
                                     config_.max_layer,
                                     native_points,
                                     native_matches,
                                     non_match);
    matches.reserve(native_matches.size());
    for (const auto &match : native_matches)
    {
        matches.emplace_back(FromC3PMatch(match));
    }
}

/**
 * @brief Searches VoxelMap and returns native matches.
 * @param points Native VoxelMap query points.
 * @param matches Output native matches.
 * @param non_match Output world-frame queries without a match.
 */
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

/**
 * @brief Searches VoxelMap++ and returns native matches.
 * @param points Native VoxelMap++ query points.
 * @param matches Output native matches.
 * @param non_match Output world-frame queries without a match.
 */
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

/**
 * @brief Tests whether a point lies inside a local-window bound.
 * @param point World-frame point.
 * @param window Axis-aligned window.
 * @return True when the point is inside the initialized window.
 */
bool MapManager::Inside(const V3D &point, const MapWindow &window)
{
    return window.initialized &&
           (point.array() >= window.min_bound.array()).all() &&
           (point.array() <= window.max_bound.array()).all();
}

/**
 * @brief Moves the enabled local window and removes entries outside it.
 * @param center World-frame window center.
 * @param half_extent Positive x/y/z half extents.
 */
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

/**
 * @brief Removes backend entries outside a local-window bound.
 * @param window Initialized axis-aligned world-frame window.
 */
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
    else if (config_.type == MapType::VoxelMapPlus)
    {
        erase_outside_voxel_plus_window(window);
    }
    else if (config_.type == MapType::IKDTree)
    {
        erase_outside_ikd_window(window);
    }
    else if (config_.type == MapType::IVox)
    {
        erase_outside_ivox_window(window);
    }
    else if (config_.type == MapType::C3PVoxelMap)
    {
        erase_outside_c3p_window(window);
    }
    filter_snapshot_to_window(window);
}

/**
 * @brief Removes entries outside a VoxelMap window.
 * @param window Initialized axis-aligned world-frame window.
 */
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

/**
 * @brief Removes entries outside a VoxelMap++ window.
 * @param window Initialized axis-aligned world-frame window.
 */
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

/**
 * @brief Removes points outside the ikd-tree window.
 * @param window Initialized axis-aligned world-frame window.
 */
void MapManager::erase_outside_ikd_window(const MapWindow &window)
{
    if (!ikd_tree_ || ikd_tree_->validnum() <= 0)
    {
        return;
    }

    PointVector stored_points;
    ikd_tree_->flatten(ikd_tree_->Root_Node, stored_points, NOT_RECORD);
    PointVector points_to_delete;
    points_to_delete.reserve(stored_points.size());
    for (const auto &point : stored_points)
    {
        if (!Inside(V3D(point.x, point.y, point.z), window))
        {
            points_to_delete.emplace_back(point);
        }
    }
    if (!points_to_delete.empty())
    {
        ikd_tree_->Delete_Points(points_to_delete);
    }

    map_points_.clear();
    stored_points.clear();
    if (ikd_tree_->Root_Node)
    {
        ikd_tree_->flatten(ikd_tree_->Root_Node, stored_points, NOT_RECORD);
    }
    map_points_.reserve(stored_points.size());
    for (const auto &point : stored_points)
    {
        map_points_.emplace_back(FromPointType(point));
    }
}

/**
 * @brief Rebuilds iVox after filtering points by the window.
 * @param window Initialized axis-aligned world-frame window.
 */
void MapManager::erase_outside_ivox_window(const MapWindow &window)
{
    MapPointList retained;
    retained.reserve(map_points_.size());
    for (const auto &point : map_points_)
    {
        if (Inside(point.point_world, window))
        {
            retained.emplace_back(point);
        }
    }
    initialize(retained);
}

/**
 * @brief Rebuilds C3P-VoxelMap after filtering points by the window.
 * @param window Initialized axis-aligned world-frame window.
 */
void MapManager::erase_outside_c3p_window(const MapWindow &window)
{
    MapPointList retained;
    retained.reserve(map_points_.size());
    for (const auto &point : map_points_)
    {
        if (Inside(point.point_world, window))
        {
            retained.emplace_back(point);
        }
    }
    initialize(retained);
}

/**
 * @brief Filters retained snapshot points to a local window.
 * @param window Initialized axis-aligned world-frame window.
 */
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

/**
 * @brief Returns the current local-window bounds.
 * @return Current window descriptor.
 */
const MapWindow &MapManager::window() const
{
    return window_;
}

/**
 * @brief Returns the backend-dependent map size.
 * @return Number of backend entries or valid grids.
 */
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
    if (config_.type == MapType::IKDTree)
    {
        return ikd_tree_ ? static_cast<size_t>(std::max(0, ikd_tree_->validnum())) : 0;
    }
    if (config_.type == MapType::IVox)
    {
        return ivox_ ? ivox_->NumValidGrids() : 0;
    }
    if (config_.type == MapType::C3PVoxelMap)
    {
        return c3p_voxel_map_.size();
    }
    return 0;
}

/**
 * @brief Returns the number of points retained for snapshots and rebuilds.
 * @return Number of retained common points.
 */
size_t MapManager::point_count() const
{
    return map_points_.size();
}

/**
 * @brief Returns a world-frame point-cloud snapshot of the selected map.
 * @return Shared point-cloud snapshot.
 */
PointCloudXYZI::Ptr MapManager::snapshot() const
{
    PointCloudXYZI::Ptr result(new PointCloudXYZI());
    if (config_.type == MapType::IKDTree && ikd_tree_ && ikd_tree_->Root_Node)
    {
        PointVector stored_points;
        ikd_tree_->flatten(ikd_tree_->Root_Node, stored_points, NOT_RECORD);
        result->reserve(stored_points.size());
        for (const auto &point : stored_points)
        {
            PointType output = point;
            output.intensity = static_cast<float>(V3D(point.x, point.y, point.z).norm());
            result->push_back(output);
        }
        return result;
    }

    result->reserve(map_points_.size());
    for (const auto &point : map_points_)
    {
        PointType output = ToPointType(point.point_world);
        output.intensity = static_cast<float>(point.point_world.norm());
        result->push_back(output);
    }
    return result;
}

/**
 * @brief Publishes native voxel-plane markers when supported.
 * @param publisher ROS publisher for the marker array.
 * @param max_voxel_layer Highest voxel layer to publish.
 */
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
    else if (config_.type == MapType::C3PVoxelMap)
    {
        c3p_map_ns::pubVoxelMap(c3p_voxel_map_, max_voxel_layer, publisher);
    }
}

}  // namespace pv_lio_plus
