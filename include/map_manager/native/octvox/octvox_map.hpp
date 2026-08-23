/*
 * Derived from Super-LIO's OctVoxMap/OctVoxMap.hpp.
 *
 * Super-LIO is distributed under GPL-3.0-or-later.  This isolated, minimal
 * adaptation retains the native sub-voxel aggregation and ordered TopK query
 * used by the Super-LIO point-to-plane observation.  The upstream file has no
 * per-file copyright header.
 */

#ifndef PV_LIO_PLUS_NATIVE_OCTVOX_MAP_HPP
#define PV_LIO_PLUS_NATIVE_OCTVOX_MAP_HPP

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <list>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "octvox_neighbors.hpp"

namespace pv_lio_plus
{
namespace octvox_ns
{
/**
 * @brief Fixed-size TopK accumulator used by Super-LIO's OctVox query.
 */
template <int K, typename Point>
class KNNHeap
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KNNHeap() : count(0), worst_(0), max_dist2_(0.0f)
    {
        std::memset(dist2_, 0, sizeof(dist2_));
    }

    void reset()
    {
        count      = 0;
        worst_     = 0;
        max_dist2_ = 0.0f;
        std::memset(dist2_, 0, sizeof(dist2_));
    }

    void try_insert(const float dist2, const Point &point)
    {
        const bool not_full      = count < K;
        const bool should_insert = not_full || dist2 < max_dist2_;
        if (!should_insert)
        {
            return;
        }

        const std::uint8_t insert_index = not_full ? count : worst_;
        dist2_[insert_index]            = dist2;
        points_[insert_index]           = point;
        if (not_full)
        {
            ++count;
            if (dist2 > max_dist2_)
            {
                max_dist2_ = dist2;
                worst_     = insert_index;
            }
        }
        else
        {
            update_worst_unrolled();
        }
    }

    std::uint8_t count;
    std::uint8_t worst_;
    float max_dist2_;
    float dist2_[K];
    std::array<Point, K> points_;

   private:
    void update_worst_unrolled()
    {
        static_assert(K == 5, "OctVox uses its native fixed five-neighbor heap");
        const float d0 = dist2_[0];
        const float d1 = dist2_[1];
        const float d2 = dist2_[2];
        const float d3 = dist2_[3];
        const float d4 = dist2_[4];

        const std::uint8_t index01 = d0 > d1 ? 0 : 1;
        const float max01          = d0 > d1 ? d0 : d1;
        const std::uint8_t index23 = d2 > d3 ? 2 : 3;
        const float max23          = d2 > d3 ? d2 : d3;
        const std::uint8_t index0123 = max01 > max23 ? index01 : index23;
        const float max0123          = max01 > max23 ? max01 : max23;
        worst_                       = max0123 > d4 ? index0123 : 4;
        max_dist2_                   = max0123 > d4 ? max0123 : d4;
    }
};

/**
 * @brief One native OctVox grid holding one aggregated point per sub-voxel.
 */
template <typename Point>
class OctVox
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    OctVox(const Point &point, const std::uint8_t local_index)
    {
        counts_.fill(kUninitialized);
        points_[local_index] = point;
        counts_[local_index] = 1;
    }

    void AddPoint(const Point &point, const std::uint8_t local_index)
    {
        std::uint8_t &count   = counts_[local_index];
        Point &stored_point   = points_[local_index];
        if (count == kUninitialized)
        {
            stored_point = point;
            count        = 1;
            return;
        }

        if (count >= kMaxPointsPerSubvoxel ||
            (point - stored_point).squaredNorm() > kDistanceThresholdSquared)
        {
            return;
        }

        stored_point = (stored_point * count + point) / static_cast<float>(count + 1);
        ++count;
    }

    bool getPoint(const std::uint8_t local_index, Point &point) const
    {
        if (counts_[local_index] == kUninitialized)
        {
            return false;
        }
        point = points_[local_index];
        return true;
    }

   private:
    static constexpr std::uint8_t kUninitialized         = 0x00;
    static constexpr std::uint8_t kMaxPointsPerSubvoxel  = 20;
    static constexpr float kDistanceThresholdSquared     = 0.1f * 0.1f;

    std::array<std::uint8_t, 8> counts_;
    std::array<Point, 8> points_;
};

/**
 * @brief Isolated Super-LIO OctVox local map.
 *
 * The map preserves its LRU grid capacity, octant-level aggregation, and
 * ordered group search.  It intentionally exposes only the operations needed
 * by PV-LIO-PLUS's MapManager.
 */
template <typename Point>
class OctVoxMap
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Points      = std::vector<Point, Eigen::aligned_allocator<Point>>;
    using KNNHeapType = KNNHeap<5, Point>;
    using OctVoxType  = OctVox<Point>;
    using Key         = Eigen::Vector3i;

    struct Options
    {
        float resolution     = 0.5f;
        std::size_t capacity = 1000000;
    };

    explicit OctVoxMap(const Options &options = Options())
    {
        SetOptions(options);
    }

    void SetOptions(const Options &options)
    {
        resolution_         = std::max(options.resolution, 1e-6f);
        capacity_           = std::max<std::size_t>(options.capacity, 1);
        inv_resolution_     = 1.0f / resolution_;
        sub_resolution_     = resolution_ / 2.0f;
        sub_inv_resolution_ = 1.0f / sub_resolution_;
    }

    void insert(const Points &cloud_world)
    {
        for (const Point &point : cloud_world)
        {
            const Key fine_key = (point * sub_inv_resolution_).array().floor().template cast<int>();
            Key key;
            key[0] = fine_key[0] >> 1;
            key[1] = fine_key[1] >> 1;
            key[2] = fine_key[2] >> 1;

            const std::uint8_t dx = static_cast<std::uint8_t>(fine_key[0] & 1);
            const std::uint8_t dy = static_cast<std::uint8_t>(fine_key[1] & 1);
            const std::uint8_t dz = static_cast<std::uint8_t>(fine_key[2] & 1);
            const std::uint8_t local_index = static_cast<std::uint8_t>((dz << 2) | (dy << 1) | dx);

            const auto grid = grids_.find(key);
            if (grid == grids_.end())
            {
                data_.emplace_front(std::piecewise_construct, std::forward_as_tuple(key),
                                    std::forward_as_tuple(point, local_index));
                grids_.emplace(key, data_.begin());
                if (data_.size() >= capacity_)
                {
                    grids_.erase(data_.back().first);
                    data_.pop_back();
                }
            }
            else
            {
                grid->second->second.AddPoint(point, local_index);
                data_.splice(data_.begin(), data_, grid->second);
            }
        }
    }

    void getTopK(const Point &point, KNNHeapType &top_k) const
    {
        const Key fine_key = (point * sub_inv_resolution_).array().floor().template cast<int>();
        Key key;
        key[0] = fine_key[0] >> 1;
        key[1] = fine_key[1] >> 1;
        key[2] = fine_key[2] >> 1;

        const int dx = fine_key[0] & 1;
        const int dy = fine_key[1] & 1;
        const int dz = fine_key[2] & 1;
        const int local_index = (dz << 2) | (dy << 1) | dx;
        const Key mirror_axis(1 - (dx << 1), 1 - (dy << 1), 1 - (dz << 1));

        std::array<const OctVoxType *, 8> nearby_voxels{};
        for (std::uint8_t index = 0; index < nearby_voxels.size(); ++index)
        {
            const Key neighbor_key = key + mirror_axis.cwiseProduct(HKNN_neighbor_voxel[index]);
            const auto grid        = grids_.find(neighbor_key);
            if (grid != grids_.end())
            {
                nearby_voxels[index] = &grid->second->second;
            }
        }

        Point sub_point;
        for (std::size_t group_index = 0; group_index + 1 < flat_search_order_offsets.size(); ++group_index)
        {
            const std::uint8_t *group_it = flat_search_order.data() + flat_search_order_offsets[group_index];
            const std::uint8_t *group_end =
                flat_search_order.data() + flat_search_order_offsets[group_index + 1];
            while (group_it < group_end)
            {
                const std::uint8_t neighbor_index = *group_it++;
                std::uint8_t data_size            = *group_it++;
                const OctVoxType *voxel           = nullptr;
                if (neighbor_index < nearby_voxels.size())
                {
                    voxel = nearby_voxels[neighbor_index];
                }
                else
                {
                    const Key neighbor_key =
                        key + mirror_axis.cwiseProduct(HKNN_neighbor_voxel[neighbor_index]);
                    const auto grid = grids_.find(neighbor_key);
                    if (grid != grids_.end())
                    {
                        voxel = &grid->second->second;
                    }
                }

                if (voxel == nullptr)
                {
                    group_it += data_size;
                    continue;
                }
                while (data_size--)
                {
                    const std::uint8_t sub_index =
                        static_cast<std::uint8_t>((*group_it++) ^ local_index);
                    if (voxel->getPoint(sub_index, sub_point))
                    {
                        top_k.try_insert((sub_point - point).squaredNorm(), sub_point);
                    }
                }
            }

            if (top_k.count == 5 && top_k.max_dist2_ < orders_min_dis2[group_index])
            {
                break;
            }
        }
    }

    void clear()
    {
        grids_.clear();
        data_.clear();
    }

    std::size_t size() const
    {
        return data_.size();
    }

    Points points() const
    {
        Points result;
        result.reserve(data_.size() * 8);
        Point point;
        for (const auto &voxel_pair : data_)
        {
            const OctVoxType &voxel = voxel_pair.second;
            for (std::uint8_t index = 0; index < 8; ++index)
            {
                if (voxel.getPoint(index, point))
                {
                    result.emplace_back(point);
                }
            }
        }
        return result;
    }

   private:
    struct KeyHash
    {
        std::size_t operator()(const Key &key) const
        {
            std::size_t hash = static_cast<std::size_t>(key[0]);
            hash ^= static_cast<std::size_t>(key[1]) * 0x9e3779b9U + (hash << 6U) + (hash >> 2U);
            hash ^= static_cast<std::size_t>(key[2]) * 0x85ebca6bU + (hash << 6U) + (hash >> 2U);
            return hash;
        }
    };

    struct KeyEqual
    {
        bool operator()(const Key &left, const Key &right) const
        {
            return left.x() == right.x() && left.y() == right.y() && left.z() == right.z();
        }
    };

    using DataValue = std::pair<Key, OctVoxType>;
    using DataList  = std::list<DataValue, Eigen::aligned_allocator<DataValue>>;
    using DataIter  = typename DataList::iterator;

    float resolution_         = 0.5f;
    float inv_resolution_     = 2.0f;
    float sub_resolution_     = 0.25f;
    float sub_inv_resolution_ = 4.0f;
    std::size_t capacity_     = 1000000;
    DataList data_;
    std::unordered_map<Key, DataIter, KeyHash, KeyEqual> grids_;
};

}  // namespace octvox_ns
}  // namespace pv_lio_plus

#endif  // PV_LIO_PLUS_NATIVE_OCTVOX_MAP_HPP
