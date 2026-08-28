/*
 * PV-LIO-PLUS local adaptation of Faster-LIO iVox.
 *
 * Upstream: https://github.com/gaoxiang12/faster-lio
 * Upstream authors: Chunge Bai, Tao Xiang, Yajie Chen, Haoqian Wang,
 * Fang Zhang, and Xiang Gao.
 * Original license: GPLv2. See LICENSE and THIRD_PARTY_NOTICES.md.
 *
 * Local modifications by PV-LIO-PLUS contributors:
 * - removed the glog dependency and use standard error reporting;
 * - validate non-positive KNN requests and use size-safe candidate handling;
 * - placed the source in the native backend include layout;
 * - added an opt-in, semantics-preserving storage/query optimization path.
 * Modified: 2026-08-27; license notice updated: 2026-08-16.
 */

//
// Created by xiang on 2021/9/16.
//

#ifndef FASTER_LIO_IVOX3D_H
#define FASTER_LIO_IVOX3D_H

#include <boost/container/small_vector.hpp>

#include <execution>
#include <cstddef>
#include <iostream>
#include <list>
#include <thread>

#include "eigen_types.h"
#include "ivox3d_node.hpp"

namespace faster_lio {

enum class IVoxNodeType {
    DEFAULT,  // linear ivox
    PHC,      // phc ivox
};

/// traits for NodeType
template <IVoxNodeType node_type, typename PointT, int dim>
struct IVoxNodeTypeTraits {};

template <typename PointT, int dim>
struct IVoxNodeTypeTraits<IVoxNodeType::DEFAULT, PointT, dim> {
    using NodeType = IVoxNode<PointT, dim>;
};

template <typename PointT, int dim>
struct IVoxNodeTypeTraits<IVoxNodeType::PHC, PointT, dim> {
    using NodeType = IVoxNodePhc<PointT, dim>;
};

template <int dim = 3, IVoxNodeType node_type = IVoxNodeType::DEFAULT, typename PointType = pcl::PointXYZ>
class IVox {
   public:
    using KeyType = Eigen::Matrix<int, dim, 1>;
    using PtType = Eigen::Matrix<float, dim, 1>;
    using NodeType = typename IVoxNodeTypeTraits<node_type, PointType, dim>::NodeType;
    using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
    using DistPoint = typename NodeType::DistPoint;

    enum class NearbyType {
        CENTER,  // center only
        NEARBY6,
        NEARBY18,
        NEARBY26,
    };

    struct Options {
        float resolution_ = 0.2;                        // ivox resolution
        float inv_resolution_ = 10.0;                   // inverse resolution
        NearbyType nearby_type_ = NearbyType::NEARBY6;  // nearby range
        std::size_t capacity_ = 1000000;                // capacity
        bool use_optimized_ = false;                    // preserve legacy behavior by default
    };

    /**
     * constructor
     * @param options  ivox options
     */
    explicit IVox(Options options) : options_(options) {
        options_.inv_resolution_ = 1.0 / options_.resolution_;
        GenerateNearbyGrids();
        if (options_.use_optimized_) {
            optimized_grids_map_.reserve(options_.capacity_);
        }
    }

    /**
     * add points
     * @param points_to_add
     */
    void AddPoints(const PointVector& points_to_add);

    /// get nn
    bool GetClosestPoint(const PointType& pt, PointType& closest_pt);

    /// get nn with condition
    bool GetClosestPoint(const PointType& pt, PointVector& closest_pt, int max_num = 5, double max_range = 5.0);

    /// get nn in cloud
    bool GetClosestPoint(const PointVector& cloud, PointVector& closest_cloud);

    /// get number of points
    size_t NumPoints() const;

    /// get number of valid grids
    size_t NumValidGrids() const;

    /// get statistics of the points
    std::vector<float> StatGridPoints() const;

   private:
    using GridValue = std::pair<KeyType, NodeType>;
    using GridCache = std::list<GridValue>;
    using GridIterator = typename GridCache::iterator;
    using LegacyGridMap = std::unordered_map<KeyType, GridIterator, hash_vec<dim>>;
    using OptimizedGridMap = std::unordered_map<KeyType, GridIterator, fast_hash_vec<dim>>;
    using InlineCandidates = boost::container::small_vector<DistPoint, 135>;

    /// generate the nearby grids according to the given options
    void GenerateNearbyGrids();

    /// position to grid
    KeyType Pos2Grid(const PtType& pt) const;

    bool GetClosestPointOptimized(const PointType& pt, PointType& closest_pt);

    bool GetClosestPointOptimized(const PointType& pt, PointVector& closest_pt, int max_num, double max_range);

    void AddPointsOptimized(const PointVector& points_to_add);

    Options options_;
    LegacyGridMap grids_map_;              // legacy voxel hash map
    GridCache grids_cache_;                 // legacy exact-LRU cache
    OptimizedGridMap optimized_grids_map_;  // reserved, full-width-hash map
    GridCache optimized_grids_cache_;       // optimized exact-LRU cache
    std::vector<KeyType> nearby_grids_;     // nearbys
};

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType& pt, PointType& closest_pt) {
    if (options_.use_optimized_) {
        return GetClosestPointOptimized(pt, closest_pt);
    }

    std::vector<DistPoint> candidates;
    auto key = Pos2Grid(ToEigen<float, dim>(pt));
    std::for_each(nearby_grids_.begin(), nearby_grids_.end(), [&key, &candidates, &pt, this](const KeyType& delta) {
        auto dkey = key + delta;
        auto iter = grids_map_.find(dkey);
        if (iter != grids_map_.end()) {
            DistPoint dist_point;
            bool found = iter->second->second.NNPoint(pt, dist_point);
            if (found) {
                candidates.emplace_back(dist_point);
            }
        }
    });

    if (candidates.empty()) {
        return false;
    }

    auto iter = std::min_element(candidates.begin(), candidates.end());
    closest_pt = iter->Get();
    return true;
}

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType& pt, PointVector& closest_pt, int max_num,
                                                      double max_range) {
    if (max_num <= 0) {
        return false;
    }
    if (options_.use_optimized_) {
        return GetClosestPointOptimized(pt, closest_pt, max_num, max_range);
    }

    const std::size_t max_num_size = static_cast<std::size_t>(max_num);
    std::vector<DistPoint> candidates;
    candidates.reserve(max_num * nearby_grids_.size());

    auto key = Pos2Grid(ToEigen<float, dim>(pt));

// #define INNER_TIMER
#ifdef INNER_TIMER
    static std::unordered_map<std::string, std::vector<int64_t>> stats;
    if (stats.empty()) {
        stats["knn"] = std::vector<int64_t>();
        stats["nth"] = std::vector<int64_t>();
    }
#endif

    for (const KeyType& delta : nearby_grids_) {
        auto dkey = key + delta;
        auto iter = grids_map_.find(dkey);
        if (iter != grids_map_.end()) {
#ifdef INNER_TIMER
            auto t1 = std::chrono::high_resolution_clock::now();
#endif
            iter->second->second.KNNPointByCondition(candidates, pt, max_num, max_range);
#ifdef INNER_TIMER
            auto t2 = std::chrono::high_resolution_clock::now();
            auto knn = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
            stats["knn"].emplace_back(knn);
#endif
        }
    }

    if (candidates.empty()) {
        return false;
    }

#ifdef INNER_TIMER
    auto t1 = std::chrono::high_resolution_clock::now();
#endif

    if (candidates.size() <= max_num_size) {
    } else {
        std::nth_element(candidates.begin(),
                         candidates.begin() + static_cast<std::ptrdiff_t>(max_num_size - 1),
                         candidates.end());
        candidates.resize(max_num_size);
    }
    std::nth_element(candidates.begin(), candidates.begin(), candidates.end());

#ifdef INNER_TIMER
    auto t2 = std::chrono::high_resolution_clock::now();
    auto nth = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
    stats["nth"].emplace_back(nth);

    constexpr int STAT_PERIOD = 100000;
    if (!stats["nth"].empty() && stats["nth"].size() % STAT_PERIOD == 0) {
        for (auto& it : stats) {
            const std::string& key = it.first;
            std::vector<int64_t>& stat = it.second;
            int64_t sum_ = std::accumulate(stat.begin(), stat.end(), 0);
            int64_t num_ = stat.size();
            stat.clear();
            std::cout << "inner_" << key << "(ns): sum=" << sum_ << " num=" << num_ << " ave=" << 1.0 * sum_ / num_
                      << " ave*n=" << 1.0 * sum_ / STAT_PERIOD << std::endl;
        }
    }
#endif

    closest_pt.clear();
    for (auto& it : candidates) {
        closest_pt.emplace_back(it.Get());
    }
    return closest_pt.empty() == false;
}

template <int dim, IVoxNodeType node_type, typename PointType>
size_t IVox<dim, node_type, PointType>::NumValidGrids() const {
    return options_.use_optimized_ ? optimized_grids_map_.size() : grids_map_.size();
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::GenerateNearbyGrids() {
    if (options_.nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY18) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0),   KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1),   KeyType(1, 1, 0),
                         KeyType(-1, 1, 0), KeyType(1, -1, 0), KeyType(-1, -1, 0), KeyType(1, 0, 1),
                         KeyType(-1, 0, 1), KeyType(1, 0, -1), KeyType(-1, 0, -1), KeyType(0, 1, 1),
                         KeyType(0, -1, 1), KeyType(0, 1, -1), KeyType(0, -1, -1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY26) {
        nearby_grids_ = {KeyType(0, 0, 0),   KeyType(-1, 0, 0),  KeyType(1, 0, 0),   KeyType(0, 1, 0),
                         KeyType(0, -1, 0),  KeyType(0, 0, -1),  KeyType(0, 0, 1),   KeyType(1, 1, 0),
                         KeyType(-1, 1, 0),  KeyType(1, -1, 0),  KeyType(-1, -1, 0), KeyType(1, 0, 1),
                         KeyType(-1, 0, 1),  KeyType(1, 0, -1),  KeyType(-1, 0, -1), KeyType(0, 1, 1),
                         KeyType(0, -1, 1),  KeyType(0, 1, -1),  KeyType(0, -1, -1), KeyType(1, 1, 1),
                         KeyType(-1, 1, 1),  KeyType(1, -1, 1),  KeyType(1, 1, -1),  KeyType(-1, -1, 1),
                         KeyType(-1, 1, -1), KeyType(1, -1, -1), KeyType(-1, -1, -1)};
    } else {
        std::cerr << "Unknown nearby_type!" << std::endl;
    }
}

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointVector& cloud, PointVector& closest_cloud) {
    std::vector<size_t> index(cloud.size());
    for (int i = 0; i < cloud.size(); ++i) {
        index[i] = i;
    }
    closest_cloud.resize(cloud.size());

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&cloud, &closest_cloud, this](size_t idx) {
        PointType pt;
        if (GetClosestPoint(cloud[idx], pt)) {
            closest_cloud[idx] = pt;
        } else {
            closest_cloud[idx] = PointType();
        }
    });
    return true;
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::AddPoints(const PointVector& points_to_add) {
    if (options_.use_optimized_) {
        AddPointsOptimized(points_to_add);
        return;
    }

    std::for_each(std::execution::unseq, points_to_add.begin(), points_to_add.end(), [this](const auto& pt) {
        auto key = Pos2Grid(ToEigen<float, dim>(pt));

        auto iter = grids_map_.find(key);
        if (iter == grids_map_.end()) {
            PointType center;
            center.getVector3fMap() = key.template cast<float>() * options_.resolution_;

            grids_cache_.push_front({key, NodeType(center, options_.resolution_)});
            grids_map_.insert({key, grids_cache_.begin()});

            grids_cache_.front().second.InsertPoint(pt);

            if (grids_map_.size() >= options_.capacity_) {
                grids_map_.erase(grids_cache_.back().first);
                grids_cache_.pop_back();
            }
        } else {
            iter->second->second.InsertPoint(pt);
            grids_cache_.splice(grids_cache_.begin(), grids_cache_, iter->second);
            grids_map_[key] = grids_cache_.begin();
        }
    });
}

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPointOptimized(const PointType& pt, PointType& closest_pt) {
    InlineCandidates candidates;
    const auto key = Pos2Grid(ToEigen<float, dim>(pt));
    for (const KeyType& delta : nearby_grids_) {
        const auto iter = optimized_grids_map_.find(key + delta);
        if (iter != optimized_grids_map_.end()) {
            DistPoint dist_point;
            if (iter->second->second.NNPoint(pt, dist_point)) {
                candidates.emplace_back(dist_point);
            }
        }
    }

    if (candidates.empty()) {
        return false;
    }

    auto iter = std::min_element(candidates.begin(), candidates.end());
    closest_pt = iter->Get();
    return true;
}

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPointOptimized(const PointType& pt, PointVector& closest_pt,
                                                               int max_num, double max_range) {
    const std::size_t max_num_size = static_cast<std::size_t>(max_num);
    InlineCandidates candidates;
    candidates.reserve(max_num_size * nearby_grids_.size());

    const auto key = Pos2Grid(ToEigen<float, dim>(pt));
    for (const KeyType& delta : nearby_grids_) {
        const auto iter = optimized_grids_map_.find(key + delta);
        if (iter != optimized_grids_map_.end()) {
            iter->second->second.KNNPointByCondition(candidates, pt, max_num, max_range);
        }
    }

    if (candidates.empty()) {
        return false;
    }

    if (candidates.size() > max_num_size) {
        std::nth_element(candidates.begin(), candidates.begin() + static_cast<std::ptrdiff_t>(max_num_size - 1),
                         candidates.end());
        candidates.resize(max_num_size);
    }
    std::nth_element(candidates.begin(), candidates.begin(), candidates.end());

    closest_pt.clear();
    for (auto& candidate : candidates) {
        closest_pt.emplace_back(candidate.Get());
    }
    return !closest_pt.empty();
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::AddPointsOptimized(const PointVector& points_to_add) {
    for (const auto& pt : points_to_add) {
        const auto key = Pos2Grid(ToEigen<float, dim>(pt));
        auto iter = optimized_grids_map_.find(key);
        if (iter == optimized_grids_map_.end()) {
            PointType center;
            center.getVector3fMap() = key.template cast<float>() * options_.resolution_;

            optimized_grids_cache_.push_front({key, NodeType(center, options_.resolution_)});
            optimized_grids_map_.insert({key, optimized_grids_cache_.begin()});
            optimized_grids_cache_.front().second.InsertPoint(pt);

            if (optimized_grids_map_.size() >= options_.capacity_) {
                optimized_grids_map_.erase(optimized_grids_cache_.back().first);
                optimized_grids_cache_.pop_back();
            }
        } else {
            const auto cache_iter = iter->second;
            cache_iter->second.InsertPoint(pt);
            if (cache_iter != optimized_grids_cache_.begin()) {
                optimized_grids_cache_.splice(optimized_grids_cache_.begin(), optimized_grids_cache_, cache_iter);
            }
        }
    }
}

template <int dim, IVoxNodeType node_type, typename PointType>
Eigen::Matrix<int, dim, 1> IVox<dim, node_type, PointType>::Pos2Grid(const IVox::PtType& pt) const {
    return (pt * options_.inv_resolution_).array().round().template cast<int>();
}

template <int dim, IVoxNodeType node_type, typename PointType>
std::vector<float> IVox<dim, node_type, PointType>::StatGridPoints() const {
    const GridCache& active_cache = options_.use_optimized_ ? optimized_grids_cache_ : grids_cache_;
    int num = active_cache.size(), valid_num = 0, max = 0, min = 100000000;
    int sum = 0, sum_square = 0;
    for (const auto& it : active_cache) {
        int s = it.second.Size();
        valid_num += s > 0;
        max = s > max ? s : max;
        min = s < min ? s : min;
        sum += s;
        sum_square += s * s;
    }
    float ave = float(sum) / num;
    float stddev = num > 1 ? sqrt((float(sum_square) - num * ave * ave) / (num - 1)) : 0;
    return std::vector<float>{valid_num, ave, max, min, stddev};
}

}  // namespace faster_lio

#endif
