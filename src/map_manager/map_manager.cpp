/*
 * Copyright (C) 2026 PV-LIO-PLUS contributors.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * PV-LIO-PLUS local-map management implementation.
 * Created: 2026-08-09; license notice updated: 2026-08-16.
 */

/**
 * @file map_manager.cpp
 * @brief Backend dispatch and local-window implementation for MapManager.
 */

#include "map_manager/map_manager.h"

#include "backends/bievr_backend.h"
#include "backends/i_octree_backend.h"
#include "backends/ig_lio_backend.h"
#include "map_manager/native/hvox/VoxelMap.h"
#include "map_manager/native/r_voxelmap/r_voxelmap.hpp"
#include "point_plane.h"

// ikd-tree is a header/template implementation.  Keep the native source in
// the PV translation unit so the imported backend is self-contained.
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

#include "map_manager/native/ikdtree/ikd_tree.cpp"

namespace pv_lio_plus
{
namespace
{
void ValidateHVoxConfig(const MapManagerConfig &config)
{
    const float resolution = static_cast<float>(config.hvox_voxel_size);
    const float distance = static_cast<float>(config.hvox_max_distance);
    const float multiplier = static_cast<float>(config.hvox_box_multiplier);
    const float planarity = static_cast<float>(config.hvox_planarity_threshold);
    const auto positive = [](double value) { return std::isfinite(value) && value > 0.0; };
    if (!positive(config.hvox_voxel_size) || !positive(resolution) ||
        config.hvox_hierarchy_factor <= 0 || config.hvox_hierarchy_factor > (1 << 20) ||
        config.hvox_hierarchy_factor % 2 == 0 ||
        !positive(resolution * static_cast<float>(config.hvox_hierarchy_factor)) ||
        !std::isfinite(config.hvox_planarity_threshold) || config.hvox_planarity_threshold < 0.0 ||
        !std::isfinite(planarity) || !positive(config.hvox_max_distance) || !positive(distance) ||
        !positive(config.hvox_box_multiplier) || !positive(multiplier) ||
        !positive(distance * multiplier) || !positive(distance / multiplier) ||
        !std::isfinite(config.hvox_lidar_noise_std) || config.hvox_lidar_noise_std < 0.0 ||
        !std::isfinite(config.hvox_lidar_noise_std * config.hvox_lidar_noise_std))
    {
        throw std::invalid_argument("invalid mapping/hvox parameters");
    }
    if (config.local_window_enabled)
    {
        throw std::invalid_argument("mapping/local_window_en is not supported by hVox; it uses its native moving box");
    }
}

// The native implementation bins floats into int keys and offsets Morton keys.
// Check before conversion so nonfinite/out-of-range coordinates cannot invoke
// undefined float-to-int conversion or signed overflow in the native code.
void ValidateHVoxCoordinate(const V3D &position, const MapManagerConfig &config)
{
    if (!position.allFinite()) throw std::invalid_argument("hVox point must be finite");
    const float resolution = static_cast<float>(config.hvox_voxel_size);
    const double min_key = static_cast<double>(std::numeric_limits<int>::min()) + config.hvox_hierarchy_factor - 1;
    const double max_key = static_cast<double>(std::numeric_limits<int>::max()) - (1 << 20);
    for (int axis = 0; axis < 3; ++axis)
    {
        const float value = static_cast<float>(position[axis]);
        const float quotient = value / resolution;
        const double key = std::floor(static_cast<double>(quotient));
        if (!std::isfinite(value) || !std::isfinite(quotient) || key < min_key || key > max_key)
            throw std::invalid_argument("hVox point is outside the native float voxel-key domain");
    }
}

void ValidateHVoxUpdate(const MapPointList &points, const MapUpdateContext &context,
                        const MapManagerConfig &config)
{
    if (!context.imu_position_world || !context.imu_position_world->allFinite() ||
        !context.imu_position_world->cast<float>().allFinite())
        throw std::invalid_argument("hVox requires a finite world-frame IMU position");
    for (const auto &point : points) ValidateHVoxCoordinate(point.point_world, config);
}

lio::PointCloudPtr ToHVoxCloud(const MapPointList &points)
{
    auto cloud = std::make_shared<lio::PointCloud>();
    cloud->reserve(points.size());
    for (const auto &point : points) cloud->push_back(lio::Point3D::from_eigen(point.point_world));
    return cloud;
}

r_voxelmap_ns::voxel_map_config MakeRVoxelConfig(const MapManagerConfig &config)
{
    r_voxelmap_ns::voxel_map_config native{};
    native.voxel_size = static_cast<float>(config.voxel_size);
    native.max_layer = config.max_layer;
    native.planer_threshold = static_cast<float>(config.plane_threshold);
    native.max_points_size = config.max_points_size;
    native.max_cov_points_size = config.max_cov_points_size;
    native.max_root_points_size = config.r_max_root_points_size;
    native.update_size_threshold = config.r_update_size_threshold;
    native.update_inlier_distance_threshold = config.r_update_inlier_distance_threshold;
    native.reinit_size_vec = config.r_reinit_size_vec;
    native.lru_capacity = config.r_lru_capacity;
    native.ransac_max_iter = config.r_ransac_max_iter;
    native.ransac_sample_num = config.r_ransac_sample_num;
    native.ransac_inlier_distance_threshold = config.r_ransac_inlier_distance_threshold;
    native.ransac_isplane_p_threshold = config.r_ransac_isplane_p_threshold;
    native.sample_seed = config.r_sample_seed;
    native.valid_check_max_layer = config.r_valid_check_max_layer;
    native.valid_check_min_points_size = config.r_valid_check_min_points_size;
    native.valid_check_resolution = config.r_valid_check_resolution;

    const auto positive = [](double value) { return std::isfinite(value) && value > 0.0; };
    if (!positive(config.voxel_size) || !positive(native.voxel_size) || native.max_layer < 0 || native.max_layer > 32 ||
        !positive(std::ldexp(native.voxel_size, -native.max_layer - 2)) || !positive(native.planer_threshold) ||
        native.max_points_size <= 0 || native.max_cov_points_size <= 0 || native.max_root_points_size <= 0 ||
        native.update_size_threshold <= 0 || !positive(native.update_inlier_distance_threshold) ||
        native.lru_capacity <= 0 || native.ransac_max_iter <= 0 || native.ransac_sample_num < 3 ||
        !positive(native.ransac_inlier_distance_threshold) ||
        !std::isfinite(native.ransac_isplane_p_threshold) || native.ransac_isplane_p_threshold <= 0.0 ||
        native.ransac_isplane_p_threshold >= 1.0 || native.valid_check_max_layer < -1 ||
        native.valid_check_min_points_size < 3 || native.valid_check_resolution <= 0 ||
        native.valid_check_resolution > 1000000 || native.reinit_size_vec.empty())
    {
        throw std::invalid_argument("invalid mapping/r_voxelmap parameters or common voxel size/layer parameters");
    }
    int previous = 0;
    for (const int value : native.reinit_size_vec)
    {
        if (value <= previous)
        {
            throw std::invalid_argument("mapping/r_voxelmap_reinit_size_vec must contain increasing positive values");
        }
        previous = value;
    }
    native.layer_point_size = config.layer_point_size;
    const std::size_t layers = static_cast<std::size_t>(native.max_layer) + 1;
    if (native.layer_point_size.empty()) native.layer_point_size.assign(layers, 5);
    else if (native.layer_point_size.size() < layers) native.layer_point_size.resize(layers, native.layer_point_size.back());
    if (std::any_of(native.layer_point_size.begin(), native.layer_point_size.end(), [](int n) { return n < 2; }))
    {
        throw std::invalid_argument("mapping/layer_point_size entries for R-VoxelMap must be at least 2");
    }
    return native;
}

std::vector<r_voxelmap_ns::pointWithCov> ToRVoxelPoints(const MapPointList &points)
{
    std::vector<r_voxelmap_ns::pointWithCov> native;
    native.reserve(points.size());
    for (const MapPoint &point : points) native.push_back({point.point_world, point.cov_world});
    return native;
}
}  // namespace
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
    if (name == "octvox" || name == "oct_vox" || name == "oct-vox")
    {
        return MapType::OctVox;
    }
    if (name == "r_voxelmap" || name == "r_voxel_map" || name == "r-voxelmap")
    {
        return MapType::RVoxelMap;
    }
    if (name == "hvox" || name == "h_vox" || name == "h-vox")
    {
        return MapType::HVox;
    }
    if (name == "i_octree" || name == "ioctree" || name == "i-octree")
    {
        return MapType::IOctree;
    }
    if (name == "ig_lio_plane" || name == "ig-lio-plane")
    {
        return MapType::IGLIOPlane;
    }

    if (name == "bievr") return MapType::BIEVR;

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
        case MapType::OctVox:
            return "octvox";
        case MapType::RVoxelMap:
            return "r_voxelmap";
        case MapType::HVox:
            return "hvox";
        case MapType::IOctree:
            return "i_octree";
        case MapType::IGLIOPlane:
            return "ig_lio_plane";
        case MapType::BIEVR:
            return "bievr";
    }
    return "unknown";
}

/** @brief 按当前地图查询高度与世界梯度；结果按值返回，不跨更新保存体素指针。 */
bool MapManager::FindBievrHeight(const V3D &pointWorld, observation::HeightSample &sample) const
{
    if (config_.type != MapType::BIEVR) throw std::logic_error("height query requires BIEVR backend");
    return bievr_ && backend_bievr::Sample(*bievr_, config_, pointWorld, sample);
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
    if ((config.type == MapType::IKDTree || config.type == MapType::IVox) &&
        !ValidNearestMaxRange(config.nearest_max_range))
    {
        throw std::invalid_argument(
            "mapping/nearest_max_range must be positive and finite in metres, with a finite nonzero square");
    }
    if (config.type == MapType::RVoxelMap)
    {
        static_cast<void>(MakeRVoxelConfig(config));
    }
    if (config.type == MapType::HVox) ValidateHVoxConfig(config);
    if (config.type == MapType::IOctree)
    {
        backend_i_octree::ValidateConfig(config);
    }
    if (config.type == MapType::IGLIOPlane)
    {
        backend_ig_lio::ValidateConfig(config);
    }
    if (config.type == MapType::OctVox && config.local_window_enabled)
    {
        throw std::invalid_argument(
            "mapping/local_window_en is not supported by Super-LIO OctVox; it only owns capacity/LRU eviction");
    }
    if (config.type == MapType::RVoxelMap && config.local_window_enabled)
    {
        throw std::invalid_argument("mapping/local_window_en is not supported by R-VoxelMap; it uses native LRU eviction");
    }

    if (config.type == MapType::BIEVR)
    {
        bievr_native::Map validated(backend_bievr::MakeConfig(config));
    }

    if (initialized_)
    {
        clear();
    }

    config_                                  = config;
    voxel_map_plus_ns::update_size_threshold = config_.plus_update_size_threshold;
    voxel_map_plus_ns::max_points_size       = config_.max_points_size;
    voxel_map_plus_ns::voxel_size            = config_.voxel_size;
    voxel_map_plus_ns::planer_threshold      = config_.plane_threshold;
    voxel_map_plus_ns::sigma_num             = static_cast<int>(config_.sigma_num);
    voxel_map_plus_ns::quater_length         = config_.voxel_size / 4.0;
    configure_point_backends();
    ++revision_;
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
           config_.type == MapType::IKDTree || config_.type == MapType::IVox || config_.type == MapType::C3PVoxelMap ||
           config_.type == MapType::OctVox || config_.type == MapType::RVoxelMap || config_.type == MapType::HVox ||
           config_.type == MapType::IOctree || config_.type == MapType::IGLIOPlane || config_.type == MapType::BIEVR;
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
    octvox_.reset();
    r_voxelmap_.reset();
    hvox_.reset();
    i_octree_.reset();
    igLio_.reset();
    bievr_.reset();

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
        options.use_optimized_ = config_.ivox_optimized;
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
    else if (config_.type == MapType::OctVox)
    {
        OctVoxBackend::Options options;
        options.resolution = static_cast<float>(config_.octvox_resolution);
        options.capacity   = std::max<std::size_t>(config_.octvox_capacity, 1);
        octvox_            = std::make_unique<OctVoxBackend>(options);
    }
    else if (config_.type == MapType::RVoxelMap)
    {
        r_voxelmap_ = std::make_unique<r_voxelmap_ns::VoxelMap>();
        r_voxelmap_->init(MakeRVoxelConfig(config_));
    }
    else if (config_.type == MapType::HVox)
    {
        hvox_ = std::make_unique<lio::VoxelMap>(static_cast<float>(config_.hvox_voxel_size));
        hvox_->SetHierarchyFactor(config_.hvox_hierarchy_factor);
        hvox_->SetPlanarityThreshold(static_cast<float>(config_.hvox_planarity_threshold));
        hvox_->SetMapBoxMultiplier(static_cast<float>(config_.hvox_box_multiplier));
    }
    else if (config_.type == MapType::IOctree)
    {
        i_octree_ = backend_i_octree::Create(config_);
    }
    else if (config_.type == MapType::IGLIOPlane)
    {
        igLio_ = backend_ig_lio::Create(config_);
    }
    else if (config_.type == MapType::BIEVR)
    {
        bievr_ = std::make_unique<bievr_native::Map>(backend_bievr::MakeConfig(config_));
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
    octvox_.reset();
    r_voxelmap_.reset();
    hvox_.reset();
    i_octree_.reset();
    igLio_.reset();
    bievr_.reset();

    map_points_.clear();
    initialized_ = false;
    window_      = MapWindow();
    ++revision_;
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
    result.point_cov   = match.point_cov;
    result.omega       = match.normal;
    result.omega_norm  = match.normal.norm();
    result.d           = match.d;
    result.distance    = match.normal.dot(match.point_world) + match.d;
    result.layer       = match.layer;
    return result;
}

/** @brief 调用共享 helper 按指定点图后端拟合平面。 */
bool MapManager::fit_point_plane(const MapPoint &query, const PointVector &neighbors, PlaneMatch &match,
                                 const MapType backend) const
{
    return backend_point_plane::FitPointPlane(query, neighbors, config_, backend, match);
}

/**
 * @brief Initializes the selected backend from common map points.
 * @param points Initial points in LiDAR and world frames.
 * @throws std::logic_error If the selected backend cannot be initialized.
 */
void MapManager::initialize(const MapPointList &points, const MapUpdateContext &context)
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
        initialize_voxel_map(native_points);
    }
    else if (config_.type == MapType::VoxelMapPlus)
    {
        std::vector<voxel_map_plus_ns::pointWithCov> native_points;
        native_points.reserve(points.size());
        for (const auto &point : points)
        {
            native_points.emplace_back(ToVoxelPlusPoint(point));
        }
        initialize_voxel_map_plus(native_points);
    }
    else if (config_.type == MapType::C3PVoxelMap)
    {
        initialize_c3p(points);
    }
    else if (config_.type == MapType::OctVox)
    {
        initialize_octvox(points);
    }
    else if (config_.type == MapType::RVoxelMap)
    {
        initialize_r_voxelmap(points);
    }
    else if (config_.type == MapType::HVox)
    {
        initialize_hvox(points, context);
    }
    else if (config_.type == MapType::IOctree)
    {
        initialize_i_octree(points);
    }
    else if (config_.type == MapType::IGLIOPlane)
    {
        InitializeIGLio(points);
    }
    else if (config_.type == MapType::BIEVR)
    {
        auto replacement = backend_bievr::Build(config_, points, context);
        clear();
        bievr_       = std::move(replacement);
        initialized_ = true;
        ++revision_;
    }
    else
    {
        initialize_point_backend(points);
    }
}

/** @brief 先构建替换树，再按 facade 原顺序清图并提交状态。 */
void MapManager::initialize_i_octree(const MapPointList &points)
{
    auto replacement = backend_i_octree::Build(config_, points);
    clear();
    i_octree_ = std::move(replacement);
    initialized_ = true;
    ++revision_;
}

/** @brief 更新现有 i-Octree；仅成功后递增地图 revision。 */
void MapManager::update_i_octree(const MapPointList &points)
{
    if (!initialized_ || !i_octree_)
    {
        initialize_i_octree(points);
        return;
    }
    backend_i_octree::Update(*i_octree_, config_, points);
    ++revision_;
}

/** @brief 完整构建替换地图后再清旧图，参数域拒绝不破坏已有地图。 */
void MapManager::InitializeIGLio(const MapPointList &points)
{
    auto replacement = backend_ig_lio::Build(config_, points);
    clear();
    igLio_       = std::move(replacement);
    initialized_ = true;
    ++revision_;
}

/** @brief 更新 iG-LIO 批次；资源异常可能部分提交，递增 revision 后向上传播。 */
void MapManager::UpdateIGLio(const MapPointList &points)
{
    if (!initialized_ || !igLio_)
    {
        InitializeIGLio(points);
        return;
    }
    try
    {
        backend_ig_lio::Update(*igLio_, points);
    } catch (const std::invalid_argument &)
    {
        throw;  // 全批输入预检发生在任何地图修改前。
    } catch (...)
    {
        ++revision_;  // 保守使缓存失效，不能把资源异常当成整批未发生。
        throw;
    }
    ++revision_;
}

void MapManager::initialize_hvox(const MapPointList &points, const MapUpdateContext &context)
{
    ValidateHVoxUpdate(points, context, config_);  // Before destroying an existing map.
    auto cloud = ToHVoxCloud(points);
    clear();
    configure_point_backends();
    hvox_->UpdateVoxelMap(cloud, *context.imu_position_world, config_.hvox_max_distance, true);
    initialized_ = true;
    ++revision_;
}

void MapManager::update_hvox(const MapPointList &points, const MapUpdateContext &context)
{
    ValidateHVoxUpdate(points, context, config_);  // Never mutate the map on rejected input.
    if (!initialized_)
    {
        initialize_hvox(points, context);
        return;
    }
    hvox_->UpdateVoxelMap(ToHVoxCloud(points), *context.imu_position_world, config_.hvox_max_distance, true);
    ++revision_;
}

void MapManager::initialize_r_voxelmap(const MapPointList &points)
{
    clear();
    configure_point_backends();
    r_voxelmap_->build(ToRVoxelPoints(points));
    r_voxelmap_->lru_cache_update();
    initialized_ = true;
}

void MapManager::update_r_voxelmap(const MapPointList &points)
{
    if (!initialized_)
    {
        initialize_r_voxelmap(points);
        return;
    }
    r_voxelmap_->update(ToRVoxelPoints(points));
    r_voxelmap_->lru_cache_update();
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

    map_points_  = points;
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

    c3p_map_ns::buildVoxelMap(config_.c3p_enable_voxel_merging, native_points, static_cast<float>(config_.voxel_size),
                              config_.max_layer, layer_point_size, static_cast<float>(config_.plane_threshold),
                              c3p_voxel_map_, c3p_plane_map_, c3p_merged_table_, config_.c3p_merge_theta_thresh,
                              config_.c3p_merge_dist_thresh, config_.c3p_merge_cov_min_eigen_val_thresh,
                              config_.c3p_merge_x_coord_diff_thresh, config_.c3p_merge_y_coord_diff_thresh);

    map_points_  = points;
    initialized_ = true;
}

/**
 * @brief Synchronizes the retained snapshot from OctVox's native sub-voxel map.
 */
void MapManager::refresh_octvox_snapshot()
{
    map_points_.clear();
    if (!octvox_)
    {
        return;
    }

    const OctVoxBackend::Points native_points = octvox_->points();
    map_points_.reserve(native_points.size());
    for (const OctVoxPoint &point : native_points)
    {
        MapPoint map_point;
        map_point.point_world = V3D(point.x(), point.y(), point.z());
        map_points_.emplace_back(map_point);
    }
}

/**
 * @brief Initializes Super-LIO's native OctVox backend.
 * @param points Initial world-frame points.
 * @throws std::logic_error If OctVox is not selected.
 */
void MapManager::initialize_octvox(const MapPointList &points)
{
    if (config_.type != MapType::OctVox)
    {
        throw std::logic_error("OctVox initialization requested for a non-OctVox backend");
    }

    clear();
    configure_point_backends();

    OctVoxBackend::Points native_points;
    native_points.reserve(points.size());
    for (const MapPoint &point : points)
    {
        native_points.emplace_back(point.point_world.cast<float>());
    }
    if (octvox_ && !native_points.empty())
    {
        octvox_->insert(native_points);
    }

    refresh_octvox_snapshot();
    initialized_ = true;
}

/**
 * @brief Initializes VoxelMap from native points.
 * @param points Native VoxelMap points.
 * @throws std::logic_error If VoxelMap is not selected.
 */
void MapManager::initialize_voxel_map(const std::vector<voxel_map_ns::pointWithCov> &points)
{
    require_supported_backend();
    if (config_.type != MapType::VoxelMap)
    {
        throw std::logic_error("VoxelMap points passed to a non-VoxelMap backend");
    }
    clear();
    voxel_map_ns::buildVoxelMap(points, config_.voxel_size, config_.max_layer, config_.layer_point_size,
                                config_.max_points_size, config_.max_cov_points_size, config_.plane_threshold,
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
void MapManager::initialize_voxel_map_plus(const std::vector<voxel_map_plus_ns::pointWithCov> &points)
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
void MapManager::update(const MapPointList &points, const std::uint32_t frame_number, const MapUpdateContext &context)
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
        update_voxel_map(native_points, frame_number);
    }
    else if (config_.type == MapType::VoxelMapPlus)
    {
        std::vector<voxel_map_plus_ns::pointWithCov> native_points;
        native_points.reserve(points.size());
        for (const auto &point : points)
        {
            native_points.emplace_back(ToVoxelPlusPoint(point));
        }
        update_voxel_map_plus(native_points, frame_number);
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
    else if (config_.type == MapType::OctVox)
    {
        if (!initialized_)
        {
            initialize_octvox(points);
        }
        else
        {
            update_octvox(points);
        }
    }
    else if (config_.type == MapType::RVoxelMap)
    {
        update_r_voxelmap(points);
    }
    else if (config_.type == MapType::HVox)
    {
        update_hvox(points, context);
    }
    else if (config_.type == MapType::IOctree)
    {
        update_i_octree(points);
    }
    else if (config_.type == MapType::IGLIOPlane)
    {
        UpdateIGLio(points);
    }
    else if (config_.type == MapType::BIEVR)
    {
        if (!initialized_ || !bievr_)
        {
            initialize(points, context);
            return;
        }
        try
        {
            backend_bievr::Update(*bievr_, points, context);
        } catch (const std::invalid_argument &)
        {
            throw;  // 全批参数预检不修改地图。
        } catch (...)
        {
            ++revision_;  // 资源异常可能已部分提交，使所有借用结果失效。
            throw;
        }
        ++revision_;
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
    const int nearest_count      = std::max(config_.nearest_point_count, NUM_MATCH_POINTS);
    const auto squared_distance  = [](const PointType &lhs, const PointType &rhs) {
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
        mid_point.x =
            static_cast<float>(std::floor(point_world.x / downsample_size) * downsample_size + 0.5 * downsample_size);
        mid_point.y =
            static_cast<float>(std::floor(point_world.y / downsample_size) * downsample_size + 0.5 * downsample_size);
        mid_point.z =
            static_cast<float>(std::floor(point_world.z / downsample_size) * downsample_size + 0.5 * downsample_size);

        const double point_to_center = squared_distance(point_world, mid_point);
        if (std::abs(points_near.front().x - mid_point.x) > 0.5 * downsample_size &&
            std::abs(points_near.front().y - mid_point.y) > 0.5 * downsample_size &&
            std::abs(points_near.front().z - mid_point.z) > 0.5 * downsample_size)
        {
            point_no_need_downsample.emplace_back(point_world);
            continue;
        }

        bool need_add                   = true;
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
    const int nearest_count      = std::max(config_.nearest_point_count, NUM_MATCH_POINTS);
    const auto squared_distance  = [](const PointType &lhs, const PointType &rhs) {
        const double dx = static_cast<double>(lhs.x) - rhs.x;
        const double dy = static_cast<double>(lhs.y) - rhs.y;
        const double dz = static_cast<double>(lhs.z) - rhs.z;
        return dx * dx + dy * dy + dz * dz;
    };

    for (const auto &point : points)
    {
        const PointType point_world = ToPointType(point.point_world);
        PointVector points_near;
        const bool has_near =
            ivox_ && ivox_->GetClosestPoint(point_world, points_near, nearest_count, config_.nearest_max_range);
        if (!has_near || points_near.empty())
        {
            points_to_add.emplace_back(point_world);
            continue;
        }

        PointType mid_point;
        mid_point.x =
            static_cast<float>(std::floor(point_world.x / downsample_size) * downsample_size + 0.5 * downsample_size);
        mid_point.y =
            static_cast<float>(std::floor(point_world.y / downsample_size) * downsample_size + 0.5 * downsample_size);
        mid_point.z =
            static_cast<float>(std::floor(point_world.z / downsample_size) * downsample_size + 0.5 * downsample_size);

        const double point_to_center = squared_distance(point_world, mid_point);
        if (std::abs(points_near.front().x - mid_point.x) > 0.5 * downsample_size &&
            std::abs(points_near.front().y - mid_point.y) > 0.5 * downsample_size &&
            std::abs(points_near.front().z - mid_point.z) > 0.5 * downsample_size)
        {
            point_no_need_downsample.emplace_back(point_world);
            continue;
        }

        bool need_add                   = true;
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

    c3p_map_ns::updateVoxelMap(config_.c3p_enable_voxel_merging, native_points, static_cast<float>(config_.voxel_size),
                               config_.max_layer, layer_point_size, static_cast<float>(config_.plane_threshold),
                               c3p_voxel_map_, c3p_plane_map_, c3p_merged_table_, frame_number,
                               config_.c3p_merge_theta_thresh, config_.c3p_merge_dist_thresh,
                               config_.c3p_merge_cov_min_eigen_val_thresh, config_.c3p_merge_x_coord_diff_thresh,
                               config_.c3p_merge_y_coord_diff_thresh);

    map_points_.insert(map_points_.end(), points.begin(), points.end());
}

/**
 * @brief Inserts one frame using Super-LIO's native OctVox LRU aggregation.
 * @param points New world-frame points.
 */
void MapManager::update_octvox(const MapPointList &points)
{
    if (!initialized_)
    {
        initialize_octvox(points);
        return;
    }
    if (!octvox_)
    {
        configure_point_backends();
    }

    OctVoxBackend::Points native_points;
    native_points.reserve(points.size());
    for (const MapPoint &point : points)
    {
        native_points.emplace_back(point.point_world.cast<float>());
    }
    if (octvox_ && !native_points.empty())
    {
        octvox_->insert(native_points);
    }
    refresh_octvox_snapshot();
}

/**
 * @brief Updates VoxelMap from native points.
 * @param points New native VoxelMap points.
 * @param frame_number Unused by VoxelMap; retained for API uniformity.
 */
void MapManager::update_voxel_map(const std::vector<voxel_map_ns::pointWithCov> &points,
                                  const std::uint32_t /*frame_number*/)
{
    require_supported_backend();
    if (config_.type != MapType::VoxelMap)
    {
        throw std::logic_error("VoxelMap points passed to a non-VoxelMap backend");
    }
    if (!initialized_)
    {
        initialize_voxel_map(points);
        return;
    }

    voxel_map_ns::updateVoxelMapOMP(points, config_.voxel_size, config_.max_layer, config_.layer_point_size,
                                    config_.max_points_size, config_.max_cov_points_size, config_.plane_threshold,
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
void MapManager::update_voxel_map_plus(const std::vector<voxel_map_plus_ns::pointWithCov> &points,
                                       const std::uint32_t /*frame_number*/)
{
    require_supported_backend();
    if (config_.type != MapType::VoxelMapPlus)
    {
        throw std::logic_error("VoxelMap++ points passed to a non-VoxelMap++ backend");
    }
    if (!initialized_)
    {
        initialize_voxel_map_plus(points);
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
    result.map_type    = MapType::VoxelMap;
    result.point       = match.point;
    result.point_world = match.point_world;
    result.normal      = match.normal;
    result.center      = match.center;
    result.plane_cov   = match.plane_cov;
    result.point_cov   = match.cov_world;
    result.d           = match.d;
    result.layer       = match.layer;
    result.omega       = match.normal;
    result.omega_norm  = match.normal.norm();
    result.distance    = match.normal.dot(match.point_world) + match.d;
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
    result.map_type    = MapType::VoxelMapPlus;
    result.point       = match.point;
    result.point_world = match.point_world;
    result.omega       = match.omega;
    result.omega_norm  = match.omega_norm;
    result.distance    = match.dist;
    result.plane_cov.setZero();
    result.plane_cov.block<3, 3>(0, 0) = match.plane_cov;
    result.point_cov                   = match.point_cov;
    result.main_direction              = match.main_direction;
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
void MapManager::search(const MapPointList &points, PlaneMatchList &matches, std::vector<V3D> &non_match)
{
    if (config_.type == MapType::BIEVR)
        throw std::logic_error("BIEVR requires typed height observations, not PlaneMatch");
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
    else if (config_.type == MapType::OctVox)
    {
        search_octvox(points, matches, non_match);
    }
    else if (config_.type == MapType::RVoxelMap)
    {
        search_r_voxelmap(points, matches, non_match);
    }
    else if (config_.type == MapType::HVox)
    {
        search_hvox(points, matches, non_match);
    }
    else if (config_.type == MapType::IOctree)
    {
        search_i_octree(points, matches, non_match);
    }
    else if (config_.type == MapType::IGLIOPlane)
    {
        backend_ig_lio::Search(igLio_.get(), config_, points, matches, non_match);
    }
    else
    {
        search_point_backend(points, matches, non_match);
    }
}

/** @brief 将 i-Octree 搜索委托给非拥有 adapter。 */
void MapManager::search_i_octree(const MapPointList &points, PlaneMatchList &matches, std::vector<V3D> &non_match)
{
    backend_i_octree::Search(i_octree_.get(), config_, points, matches, non_match);
}

void MapManager::search_hvox(const MapPointList &points, PlaneMatchList &matches, std::vector<V3D> &non_match)
{
    // Reject the whole observation before querying if any input is invalid.
    for (const auto &point : points)
    {
        if (!point.point_lidar.allFinite()) throw std::invalid_argument("hVox LiDAR point must be finite");
        ValidateHVoxCoordinate(point.point_world, config_);
    }
    if (!hvox_)
    {
        for (const auto &point : points) non_match.push_back(point.point_world);
        return;
    }
    for (const auto &point : points)
    {
        Eigen::Vector3f normal, center;
        float planarity = 0.0f;
        if (!hvox_->GetSurfelAtPoint(lio::Point3D::from_eigen(point.point_world), normal, center, planarity))
        {
            non_match.push_back(point.point_world);
            continue;
        }
        if (!normal.allFinite() || !center.allFinite() || !std::isfinite(planarity) || normal.squaredNorm() < 1e-12f)
            throw std::runtime_error("hVox produced a nonfinite or degenerate surfel");
        PlaneMatch match;
        match.map_type = MapType::HVox;
        match.point = point.point_lidar;
        match.point_world = point.point_world;
        // Preserve the native normal sign: the imported PKO is asymmetric.
        match.normal = normal.cast<double>();
        match.center = center.cast<double>();
        match.distance = match.normal.dot(match.point_world - match.center);
        if (!std::isfinite(match.distance)) throw std::runtime_error("hVox residual is nonfinite");
        matches.push_back(match);
    }
}

void MapManager::search_r_voxelmap(const MapPointList &points, PlaneMatchList &matches,
                                   std::vector<V3D> &non_match)
{
    matches.clear();
    non_match.clear();
    if (!r_voxelmap_)
    {
        for (const auto &point : points) non_match.push_back(point.point_world);
        return;
    }
    std::vector<r_voxelmap_ns::ptpl> native_matches;
    std::vector<int> useful_indices;
    r_voxelmap_->build_residual_omp(ToRVoxelPoints(points), config_.max_layer, native_matches, useful_indices,
                                     non_match);
    matches.reserve(native_matches.size());
    for (std::size_t i = 0; i < native_matches.size(); ++i)
    {
        const auto &native = native_matches[i];
        const MapPoint &point = points.at(static_cast<std::size_t>(useful_indices.at(i)));
        PlaneMatch match;
        match.map_type = MapType::RVoxelMap;
        match.point = point.point_lidar;
        match.point_world = point.point_world;
        match.point_cov = point.cov_world;
        match.lidar_cov = point.cov_lidar;
        match.normal = native.normal;
        match.center = native.center;
        match.plane_cov = native.plane_cov;
        match.d = native.d;
        match.distance = native.normal.dot(point.point_world) + native.d;
        match.layer = native.layer;
        matches.emplace_back(match);
    }
}

/**
 * @brief Dispatches point-map search to the selected backend.
 * @param points Query points in LiDAR and world frames.
 * @param matches Output common point-to-plane matches.
 * @param non_match Output world-frame queries without a match.
 */
void MapManager::search_point_backend(const MapPointList &points, PlaneMatchList &matches, std::vector<V3D> &non_match)
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
void MapManager::search_ikd_tree(const MapPointList &points, PlaneMatchList &matches, std::vector<V3D> &non_match)
{
    matches.clear();
    non_match.clear();
    matches.reserve(points.size());
    non_match.reserve(points.size());

    const int nearest_count = std::max(config_.nearest_point_count, NUM_MATCH_POINTS);
    const double max_range_squared = config_.nearest_max_range * config_.nearest_max_range;
    for (const auto &point : points)
    {
        PointVector neighbors;
        std::vector<float> point_search_sq_dist;
        if (ikd_tree_ && ikd_tree_->validnum() > 0)
        {
            ikd_tree_->Nearest_Search(ToPointType(point.point_world), nearest_count, neighbors, point_search_sq_dist);
        }

        if (neighbors.size() < NUM_MATCH_POINTS ||
            (point_search_sq_dist.size() >= NUM_MATCH_POINTS &&
             point_search_sq_dist[NUM_MATCH_POINTS - 1] > max_range_squared))
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
void MapManager::search_ivox(const MapPointList &points, PlaneMatchList &matches, std::vector<V3D> &non_match)
{
    matches.clear();
    non_match.clear();
    matches.reserve(points.size());
    non_match.reserve(points.size());

    const int nearest_count = std::max(config_.nearest_point_count, NUM_MATCH_POINTS);
    for (const auto &point : points)
    {
        PointVector neighbors;
        const bool found = ivox_ && ivox_->GetClosestPoint(ToPointType(point.point_world), neighbors, nearest_count,
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
void MapManager::search_c3p(const MapPointList &points, PlaneMatchList &matches, std::vector<V3D> &non_match)
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

    c3p_map_ns::BuildResidualListOMP(c3p_voxel_map_, config_.voxel_size, config_.sigma_num, config_.max_layer,
                                     native_points, native_matches, non_match);
    matches.reserve(native_matches.size());
    for (const auto &match : native_matches)
    {
        matches.emplace_back(FromC3PMatch(match));
    }
}

/**
 * @brief Searches OctVox with its native TopK and local-plane acceptance rules.
 *
 * The plane builder intentionally matches Super-LIO: up to five TopK points,
 * four-point fallback, 0.1 m neighbor-plane threshold, and the
 * range-versus-residual gate.  No covariance-derived weight is produced here.
 */
void MapManager::search_octvox(const MapPointList &points, PlaneMatchList &matches, std::vector<V3D> &non_match)
{
    matches.clear();
    non_match.clear();
    matches.reserve(points.size());
    non_match.reserve(points.size());

    const auto build_plane = [](const int count, const std::array<OctVoxPoint, 5> &neighbors, V3D &normal,
                                double &offset) {
        Eigen::Vector3d normal_unnormalized;
        if (count == 5)
        {
            Eigen::Matrix<double, 5, 3> system;
            Eigen::Matrix<double, 5, 1> rhs;
            for (int index = 0; index < 5; ++index)
            {
                system.row(index) = neighbors[static_cast<std::size_t>(index)].cast<double>();
                rhs(index)        = -1.0;
            }
            normal_unnormalized = system.colPivHouseholderQr().solve(rhs);
        }
        else
        {
            Eigen::Matrix<double, 4, 3> system;
            Eigen::Matrix<double, 4, 1> rhs;
            for (int index = 0; index < 4; ++index)
            {
                system.row(index) = neighbors[static_cast<std::size_t>(index)].cast<double>();
                rhs(index)        = -1.0;
            }
            normal_unnormalized = system.colPivHouseholderQr().solve(rhs);
        }

        const double norm = normal_unnormalized.norm();
        if (norm < 1e-6)
        {
            return false;
        }
        offset = 1.0 / norm;
        normal = normal_unnormalized * offset;
        for (int index = 0; index < count; ++index)
        {
            const double distance = normal.dot(neighbors[static_cast<std::size_t>(index)].cast<double>()) + offset;
            if (std::abs(distance) > 0.1)
            {
                return false;
            }
        }
        return true;
    };

    for (const MapPoint &point : points)
    {
        if (!octvox_)
        {
            non_match.emplace_back(point.point_world);
            continue;
        }

        OctVoxBackend::KNNHeapType top_k;
        top_k.reset();
        octvox_->getTopK(point.point_world.cast<float>(), top_k);
        if (top_k.count < 4)
        {
            non_match.emplace_back(point.point_world);
            continue;
        }

        V3D normal;
        double offset = 0.0;
        if (!build_plane(top_k.count, top_k.points_, normal, offset))
        {
            non_match.emplace_back(point.point_world);
            continue;
        }

        const double residual = normal.dot(point.point_world) + offset;
        const double range    = point.point_lidar.norm();
        if (!(range > 81.0 * residual * residual))
        {
            non_match.emplace_back(point.point_world);
            continue;
        }

        PlaneMatch match;
        match.map_type    = MapType::OctVox;
        match.point       = point.point_lidar;
        match.point_world = point.point_world;
        match.normal      = normal;
        match.omega       = normal;
        match.omega_norm  = 1.0;
        match.d           = offset;
        match.distance    = residual;
        match.center.setZero();
        for (int index = 0; index < top_k.count; ++index)
        {
            match.center += top_k.points_[static_cast<std::size_t>(index)].cast<double>();
        }
        match.center /= static_cast<double>(top_k.count);
        matches.emplace_back(match);
    }
}

/**
 * @brief Searches VoxelMap and returns native matches.
 * @param points Native VoxelMap query points.
 * @param matches Output native matches.
 * @param non_match Output world-frame queries without a match.
 */
void MapManager::search(const std::vector<voxel_map_ns::pointWithCov> &points, std::vector<voxel_map_ns::ptpl> &matches,
                        std::vector<V3D> &non_match)
{
    require_supported_backend();
    if (config_.type != MapType::VoxelMap)
    {
        throw std::logic_error("VoxelMap points passed to a non-VoxelMap backend");
    }
    voxel_map_ns::BuildResidualListOMP(voxel_map_, config_.voxel_size, config_.sigma_num, config_.max_layer, points,
                                       matches, non_match);
}

/**
 * @brief Searches VoxelMap++ and returns native matches.
 * @param points Native VoxelMap++ query points.
 * @param matches Output native matches.
 * @param non_match Output world-frame queries without a match.
 */
void MapManager::search(const std::vector<voxel_map_plus_ns::pointWithCov> &points,
                        std::vector<voxel_map_plus_ns::ptpl> &matches, std::vector<V3D> &non_match)
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
    return window.initialized && (point.array() >= window.min_bound.array()).all() &&
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
    if (config_.type == MapType::RVoxelMap)
    {
        throw std::logic_error("R-VoxelMap does not support geometric local-window deletion");
    }
    if (config_.type == MapType::HVox)
    {
        throw std::logic_error("hVox does not support public geometric local-window deletion");
    }
    if (config_.type == MapType::IGLIOPlane)
    {
        throw std::logic_error("ig_lio_plane does not support geometric local-window deletion; it uses native LRU");
    }
    if (config_.type == MapType::BIEVR) throw std::invalid_argument("BIEVR does not support a common geometric window");
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
    else if (config_.type == MapType::IOctree)
    {
        if (i_octree_)
        {
            static_cast<void>(backend_i_octree::Retain(*i_octree_, window));
            ++revision_;
        }
    }
    else if (config_.type == MapType::OctVox)
    {
        throw std::logic_error("OctVox does not support geometric local-window deletion");
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

std::uint64_t MapManager::revision() const
{
    return revision_;
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
    if (config_.type == MapType::OctVox)
    {
        return octvox_ ? octvox_->size() : 0;
    }
    if (config_.type == MapType::RVoxelMap)
    {
        return r_voxelmap_ ? r_voxelmap_->feat_map.size() : 0;
    }
    if (config_.type == MapType::HVox)
    {
        return hvox_ ? hvox_->GetVoxelCount() : 0;
    }
    if (config_.type == MapType::IOctree)
    {
        return backend_i_octree::Size(i_octree_.get());
    }
    if (config_.type == MapType::IGLIOPlane)
    {
        return backend_ig_lio::Size(igLio_.get());
    }
    if (config_.type == MapType::BIEVR) return bievr_ ? bievr_->Size() : 0;
    return 0;
}

/**
 * @brief Returns the number of points retained for snapshots and rebuilds.
 * @return Number of retained common points.
 */
size_t MapManager::point_count() const
{
    if (config_.type == MapType::BIEVR) return bievr_ ? bievr_->PointCount() : 0;
    if (config_.type == MapType::IGLIOPlane) return backend_ig_lio::PointCount(igLio_.get());
    if (config_.type == MapType::IOctree) return backend_i_octree::Size(i_octree_.get());
    if (config_.type == MapType::HVox) return hvox_ ? hvox_->GetPointCount() : 0;
    if (config_.type == MapType::RVoxelMap)
    {
        std::size_t count = 0;
        if (r_voxelmap_)
            for (const auto &entry : r_voxelmap_->feat_map) count += entry.second->point_count();
        return count;
    }
    return map_points_.size();
}

/**
 * @brief Returns a world-frame point-cloud snapshot of the selected map.
 * @return Shared point-cloud snapshot.
 */
PointCloudXYZI::Ptr MapManager::snapshot() const
{
    PointCloudXYZI::Ptr result(new PointCloudXYZI());
    if (config_.type == MapType::BIEVR)
    {
        if (bievr_)
            for (const auto &point : bievr_->Snapshot()) result->push_back(ToPointType(point));
        return result;
    }
    if (config_.type == MapType::IGLIOPlane)
    {
        const auto points = backend_ig_lio::Snapshot(igLio_.get());
        result->reserve(points.size());
        for (const auto &point : points)
        {
            result->push_back(ToPointType(point));
        }
        return result;
    }
    if (config_.type == MapType::IOctree)
    {
        if (i_octree_)
        {
            const auto points = backend_i_octree::Snapshot(i_octree_.get());
            result->reserve(points.size());
            for (const auto &point : points)
            {
                const V3D p(point.x, point.y, point.z);
                result->push_back(ToPointType(p));  // Native leaf stores XYZ only.
            }
        }
        return result;
    }
    if (config_.type == MapType::HVox)
    {
        if (hvox_)
        {
            const auto centers = hvox_->GetL0Centroids();
            result->reserve(centers.size());
            for (const auto &center : centers) result->push_back(ToPointType(center.cast<double>()));
        }
        return result;
    }
    if (config_.type == MapType::RVoxelMap)
    {
        if (r_voxelmap_)
        {
            std::vector<r_voxelmap_ns::pointWithCov> stored_points;
            for (const auto &entry : r_voxelmap_->feat_map) entry.second->append_points(stored_points);
            result->reserve(stored_points.size());
            for (const auto &point : stored_points)
                result->push_back(ToPointType(point.point, static_cast<float>(point.point.norm())));
        }
        return result;
    }
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
