// Copyright (c) 2023 Jun Zhu, Tsinghua University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
//
// PV-LIO-PLUS adaptation of i-Octree 5b640e05; see SOURCE.md.
#include "map_manager/native/i_octree/i_octree.hpp"

#include <algorithm>
#include <cfloat>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
#if defined(__x86_64__) || defined(__i386__)
#include <xmmintrin.h>
#endif

#ifdef __FAST_MATH__
#error "i-Octree binary64 distance kernel cannot be compiled with fast-math"
#endif
static_assert(FLT_EVAL_METHOD == 0, "i-Octree needs binary32/binary64 evaluation at standard precision");
static_assert(std::numeric_limits<float>::is_iec559 && std::numeric_limits<float>::digits == 24 && sizeof(float) == 4,
              "i-Octree needs IEEE binary32");
static_assert(std::numeric_limits<double>::is_iec559 && std::numeric_limits<double>::digits == 53 && sizeof(double) == 8,
              "i-Octree needs IEEE binary64");

namespace pv_lio_plus::i_octree_native
{
namespace
{
constexpr std::size_t kMaxSupportedDepth = 512;
constexpr double kInitialExtent = static_cast<double>(0.01f);
constexpr double kFloatLimit = static_cast<double>(std::numeric_limits<float>::max());

std::array<double, 3> values(const Point3d &p) { return {{p.x, p.y, p.z}}; }
std::array<float, 3> values(const Point3f &p) { return {{p.x, p.y, p.z}}; }
void validate_query(const Point3d &p)
{
    for (const double x : values(p))
        if (!std::isfinite(x) || std::abs(x) > kFloatLimit)
            throw std::invalid_argument("i-Octree query coordinates must be finite and within the binary32 range");
}

void validate_box(const Point3d &lo, const Point3d &hi)
{
    const auto a = values(lo), b = values(hi);
    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(a[i]) || !std::isfinite(b[i]) || a[i] > b[i])
            throw std::invalid_argument("i-Octree box endpoints must be finite and ordered");
}

std::vector<Point3f> quantize(const std::vector<Point3d> &points)
{
    std::vector<Point3f> out;
    out.reserve(points.size());
    for (const auto &p : points)
    {
        validate_query(p);
        out.push_back({static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)});
    }
    return out;
}

Aabb one_point_box(const Point3f &p) { return {p, p}; }
Aabb merge_box(const Aabb &a, const Aabb &b)
{
    return {{std::min(a.lo.x, b.lo.x), std::min(a.lo.y, b.lo.y), std::min(a.lo.z, b.lo.z)},
            {std::max(a.hi.x, b.hi.x), std::max(a.hi.y, b.hi.y), std::max(a.hi.z, b.hi.z)}};
}

// Exact expression order is the public binary64 distance contract.
double kernel(const std::array<double, 3> &p, const std::array<double, 3> &q)
{
    const double dx = std::abs(p[0] - q[0]);
    const double dy = std::abs(p[1] - q[1]);
    const double dz = std::abs(p[2] - q[2]);
    const double sx = dx * dx;
    const double sy = dy * dy;
    const double sz = dz * dz;
    return (sx + sy) + sz;
}

double point_distance(const Point3f &p, const Point3d &q)
{
    return kernel({{double(p.x), double(p.y), double(p.z)}}, values(q));
}

DistanceBounds bounds_distance(const Aabb &box, const Point3d &query)
{
    const auto lo = values(box.lo), hi = values(box.hi);
    const auto q = values(query);
    std::array<double, 3> near, far_diff;
    for (int i = 0; i < 3; ++i)
    {
        near[i] = std::max(double(lo[i]), std::min(q[i], double(hi[i])));
        const double dl = std::abs(double(lo[i]) - q[i]);
        const double dh = std::abs(double(hi[i]) - q[i]);
        far_diff[i] = std::max(dl, dh);
    }
    const double sx = far_diff[0] * far_diff[0];
    const double sy = far_diff[1] * far_diff[1];
    const double sz = far_diff[2] * far_diff[2];
    return {kernel(near, q), (sx + sy) + sz};
}

struct Geometry { std::array<double, 3> center; double extent; };
struct Node
{
    using Children = std::array<std::unique_ptr<Node>, 8>;
    Geometry geometry;
    Aabb bounds;
    std::size_t count = 0;
    std::size_t height = 0;
    bool leaf = true;
    std::vector<Point3f> points;  // One contiguous block per leaf.
    std::unique_ptr<Children> children;  // Allocated only for internal nodes.
};
using PointRefs = std::vector<const Point3f *>;

int code(const Point3f &p, const Geometry &g)
{
    return (double(p.x) > g.center[0] ? 1 : 0) |
           (double(p.y) > g.center[1] ? 2 : 0) |
           (double(p.z) > g.center[2] ? 4 : 0);
}
int code(const Point3d &p, const Geometry &g)
{
    return (p.x > g.center[0] ? 1 : 0) | (p.y > g.center[1] ? 2 : 0) | (p.z > g.center[2] ? 4 : 0);
}

Geometry child_geometry(const Geometry &g, int i)
{
    const double child_extent = g.extent * 0.5;
    if (!std::isfinite(child_extent) || child_extent <= 0 || child_extent >= g.extent)
        throw std::range_error("i-Octree child extent cannot decrease safely");
    Geometry result{{}, child_extent};
    for (int axis = 0; axis < 3; ++axis)
    {
        result.center[axis] = g.center[axis] + ((i & (1 << axis)) ? child_extent : -child_extent);
        if (!std::isfinite(result.center[axis])) throw std::range_error("i-Octree child center is nonfinite");
    }
    return result;
}

std::array<PointRefs, 8> divide(const PointRefs &points, const Geometry &g)
{
    std::array<PointRefs, 8> groups;
    for (const Point3f *p : points) groups[static_cast<std::size_t>(code(*p, g))].push_back(p);
    return groups;
}

void tighten(Node &node)
{
    if (node.leaf)
    {
        node.count = node.points.size();
        node.height = 0;
        if (node.points.empty()) throw std::logic_error("i-Octree must not retain an empty leaf");
        node.bounds = one_point_box(node.points.front());
        for (std::size_t i = 1; i < node.points.size(); ++i)
            node.bounds = merge_box(node.bounds, one_point_box(node.points[i]));
        return;
    }
    node.count = 0;
    node.height = 0;
    bool first = true;
    for (const auto &child : *node.children)
    {
        if (!child) continue;
        node.count += child->count;
        node.height = std::max(node.height, std::size_t(1) + child->height);
        node.bounds = first ? child->bounds : merge_box(node.bounds, child->bounds);
        first = false;
    }
    if (first) throw std::logic_error("i-Octree must not retain an empty internal node");
}

bool outside(const Aabb &b, const Point3d &lo, const Point3d &hi)
{
    return double(b.hi.x) < lo.x || double(b.lo.x) > hi.x ||
           double(b.hi.y) < lo.y || double(b.lo.y) > hi.y ||
           double(b.hi.z) < lo.z || double(b.lo.z) > hi.z;
}
bool inside(const Aabb &b, const Point3d &lo, const Point3d &hi)
{
    return double(b.lo.x) >= lo.x && double(b.hi.x) <= hi.x &&
           double(b.lo.y) >= lo.y && double(b.hi.y) <= hi.y &&
           double(b.lo.z) >= lo.z && double(b.hi.z) <= hi.z;
}
bool inside(const Point3f &p, const Point3d &lo, const Point3d &hi)
{
    return double(p.x) >= lo.x && double(p.x) <= hi.x &&
           double(p.y) >= lo.y && double(p.y) <= hi.y &&
           double(p.z) >= lo.z && double(p.z) <= hi.z;
}
}  // namespace

void require_numeric_environment()
{
    if (std::fegetround() != FE_TONEAREST)
        throw std::runtime_error("i-Octree requires round-to-nearest/ties-to-even");
#if defined(__x86_64__) || defined(__i386__)
    const unsigned int csr = _mm_getcsr();
    if ((csr & (0x40u | 0x8000u | 0x6000u)) != 0)
        throw std::runtime_error("i-Octree requires MXCSR round-to-nearest and DAZ/FTZ disabled");
#else
    throw std::runtime_error("i-Octree numeric environment is not verified on this architecture");
#endif
}

double distance_squared(const Point3f &point, const Point3d &query)
{
    require_numeric_environment();
    validate_query(query);
    for (float value : values(point))
        if (!std::isfinite(value)) throw std::invalid_argument("i-Octree point must be finite");
    return point_distance(point, query);
}

double radius_squared(double radius)
{
    require_numeric_environment();
    if (!std::isfinite(radius) || radius <= 0 || radius > 4.0 * kFloatLimit)
        throw std::invalid_argument("i-Octree radius is outside the supported domain");
    const double threshold = radius * radius;
    if (threshold <= 0.0 || !std::isfinite(threshold))
        throw std::invalid_argument("i-Octree radius squared is not finite and positive");
    return threshold;
}

DistanceBounds aabb_distance_bounds(const Aabb &box, const Point3d &query)
{
    require_numeric_environment();
    validate_query(query);
    const auto lo = values(box.lo), hi = values(box.hi);
    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(lo[i]) || !std::isfinite(hi[i]) || lo[i] > hi[i])
            throw std::invalid_argument("i-Octree AABB must be finite and ordered");
    return bounds_distance(box, query);
}

struct Octree::Impl
{
    explicit Impl(const Config &c) : config(c)
    {
        require_numeric_environment();
        if (config.bucket_size == 0 || config.bucket_size > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
            !std::isfinite(config.min_extent) || config.min_extent < double(std::numeric_limits<float>::denorm_min()) ||
            config.min_extent > kFloatLimit || config.max_depth == 0 || config.max_depth > kMaxSupportedDepth)
            throw std::invalid_argument("invalid i-Octree bucket size, minimum extent, or depth");
    }

    Config config;
    std::unique_ptr<Node> root;

    bool should_split(std::size_t count, const Geometry &g) const
    {
        return count > config.bucket_size && g.extent > 2.0 * config.min_extent;
    }

    void check_depth(std::size_t depth) const
    {
        if (depth > config.max_depth) throw std::range_error("i-Octree depth limit would be exceeded");
    }

    Geometry initial_geometry(const PointRefs &points) const
    {
        auto lo = values(*points.front()), hi = lo;
        for (const Point3f *p : points)
        {
            const auto v = values(*p);
            for (int axis = 0; axis < 3; ++axis)
            {
                lo[axis] = std::min(lo[axis], v[axis]);
                hi[axis] = std::max(hi[axis], v[axis]);
            }
        }
        Geometry g{{}, kInitialExtent};
        for (int axis = 0; axis < 3; ++axis)
        {
            const double span = double(hi[axis]) - double(lo[axis]);
            const double half = span * 0.5;
            g.center[axis] = double(lo[axis]) + half;
            g.extent = std::max(g.extent, half);
        }
        return g;
    }

    void preview_build(const Geometry &g, const PointRefs &points, std::size_t depth) const
    {
        check_depth(depth);
        if (!should_split(points.size(), g)) return;
        check_depth(depth + 1);
        const auto groups = divide(points, g);
        for (int i = 0; i < 8; ++i)
            if (!groups[static_cast<std::size_t>(i)].empty())
                preview_build(child_geometry(g, i), groups[static_cast<std::size_t>(i)], depth + 1);
    }

    void preview_update(const Node &node, const PointRefs &points, bool downsample, std::size_t depth) const
    {
        check_depth(depth);
        if (points.empty()) return;
        if (node.leaf)
        {
            if (node.points.size() > std::numeric_limits<std::size_t>::max() - points.size())
                throw std::length_error("i-Octree point count overflow");
            if (should_split(node.points.size() + points.size(), node.geometry))
            {
                PointRefs merged;
                merged.reserve(node.points.size() + points.size());
                for (const auto &point : node.points) merged.push_back(&point);
                merged.insert(merged.end(), points.begin(), points.end());
                preview_build(node.geometry, merged, depth);
            }
            // Otherwise the native leaf either accepts or rejects the whole batch.
            return;
        }
        const auto groups = divide(points, node.geometry);
        for (int i = 0; i < 8; ++i)
        {
            const auto &group = groups[static_cast<std::size_t>(i)];
            if (group.empty()) continue;
            const auto &child = (*node.children)[static_cast<std::size_t>(i)];
            if (child) preview_update(*child, group, downsample, depth + 1);
            else preview_build(child_geometry(node.geometry, i), group, depth + 1);
        }
    }

    std::unique_ptr<Node> build(const Geometry &g, const PointRefs &points) const
    {
        auto node = std::make_unique<Node>();
        node->geometry = g;
        if (should_split(points.size(), g))
        {
            node->leaf = false;
            node->children = std::make_unique<Node::Children>();
            const auto groups = divide(points, g);
            for (int i = 0; i < 8; ++i)
                if (!groups[static_cast<std::size_t>(i)].empty())
                    (*node->children)[static_cast<std::size_t>(i)] =
                        build(child_geometry(g, i), groups[static_cast<std::size_t>(i)]);
        }
        else
        {
            node->points.reserve(points.size());
            for (const Point3f *p : points) node->points.push_back(*p);
        }
        tighten(*node);
        return node;
    }

    struct Expansion { Geometry geometry; int old_child; };

    std::vector<Expansion> preview_expansions(const PointRefs &points) const
    {
        auto lo = values(*points.front()), hi = lo;
        for (const Point3f *p : points)
        {
            const auto v = values(*p);
            for (int i = 0; i < 3; ++i) { lo[i] = std::min(lo[i], v[i]); hi[i] = std::max(hi[i], v[i]); }
        }
        Geometry current = root->geometry;
        std::vector<Expansion> result;
        for (;;)
        {
            bool within = true;
            for (int i = 0; i < 3; ++i)
            {
                const double dlo = std::nextafter(std::abs(double(lo[i]) - current.center[i]),
                                                   std::numeric_limits<double>::infinity());
                const double dhi = std::nextafter(std::abs(double(hi[i]) - current.center[i]),
                                                   std::numeric_limits<double>::infinity());
                if (dlo > current.extent || dhi > current.extent) within = false;
            }
            if (within) break;
            if (result.size() >= config.max_depth || root->height > config.max_depth - result.size() - 1)
                throw std::range_error("i-Octree root expansion exceeds depth limit");
            Geometry parent{{}, current.extent * 2.0};
            if (!std::isfinite(parent.extent) || parent.extent <= current.extent)
                throw std::range_error("i-Octree root extent cannot increase safely");
            int old_child = 0;
            for (int i = 0; i < 3; ++i)
            {
                const double dlo = std::abs(double(lo[i]) - current.center[i]);
                const double dhi = std::abs(double(hi[i]) - current.center[i]);
                const double target = dhi >= dlo ? double(hi[i]) : double(lo[i]);
                parent.center[i] = current.center[i] + (target > current.center[i] ? current.extent : -current.extent);
                if (!std::isfinite(parent.center[i])) throw std::range_error("i-Octree parent center is nonfinite");
                if (current.center[i] > parent.center[i]) old_child |= (1 << i);
            }
            result.push_back({parent, old_child});
            current = parent;
        }
        return result;
    }

    void preview_virtual(const std::vector<Expansion> &expansions, int index, const Node &old_root,
                         const PointRefs &points, bool downsample, std::size_t depth) const
    {
        check_depth(depth);
        if (points.empty()) return;
        if (index < 0)
        {
            preview_update(old_root, points, downsample, depth);
            return;
        }
        const auto &expansion = expansions[static_cast<std::size_t>(index)];
        const auto groups = divide(points, expansion.geometry);
        for (int i = 0; i < 8; ++i)
        {
            const auto &group = groups[static_cast<std::size_t>(i)];
            if (group.empty()) continue;
            if (i == expansion.old_child)
                preview_virtual(expansions, index - 1, old_root, group, downsample, depth + 1);
            else
                preview_build(child_geometry(expansion.geometry, i), group, depth + 1);
        }
    }

    std::size_t insert(Node &node, const PointRefs &points, bool downsample) const
    {
        if (points.empty()) return 0;
        if (node.leaf)
        {
            if (should_split(node.points.size() + points.size(), node.geometry))
            {
                PointRefs merged;
                merged.reserve(node.points.size() + points.size());
                for (const auto &point : node.points) merged.push_back(&point);
                merged.insert(merged.end(), points.begin(), points.end());
                auto replacement = build(node.geometry, merged);
                node = std::move(*replacement);
                return points.size();
            }
            if (downsample && node.geometry.extent <= 2.0 * config.min_extent &&
                node.points.size() > config.bucket_size / 8)
                return 0;
            node.points.reserve(node.points.size() + points.size());
            for (const Point3f *point : points) node.points.push_back(*point);
            tighten(node);
            return points.size();
        }
        const auto groups = divide(points, node.geometry);
        std::size_t inserted = 0;
        try
        {
            for (int i = 0; i < 8; ++i)
            {
                const auto &group = groups[static_cast<std::size_t>(i)];
                if (group.empty()) continue;
                auto &child = (*node.children)[static_cast<std::size_t>(i)];
                if (child) inserted += insert(*child, group, downsample);
                else
                {
                    child = build(child_geometry(node.geometry, i), group);
                    inserted += group.size();
                }
            }
        }
        catch (...)
        {
            tighten(node);  // A partial allocation failure must not stale the data AABB/count.
            throw;
        }
        tighten(node);
        return inserted;
    }

    void initialize(const std::vector<Point3f> &points)
    {
        if (points.empty()) { root.reset(); return; }
        PointRefs refs;
        refs.reserve(points.size());
        for (const auto &point : points) refs.push_back(&point);
        const Geometry geometry = initial_geometry(refs);
        preview_build(geometry, refs, 0);
        auto fresh = build(geometry, refs);
        root = std::move(fresh);
    }

    void update(const std::vector<Point3f> &points, bool downsample)
    {
        if (points.empty()) return;
        if (!root) { initialize(points); return; }
        if (root->count > std::numeric_limits<std::size_t>::max() - points.size())
            throw std::length_error("i-Octree point count overflow");
        PointRefs refs;
        refs.reserve(points.size());
        for (const auto &point : points) refs.push_back(&point);
        const auto expansions = preview_expansions(refs);
        preview_virtual(expansions, static_cast<int>(expansions.size()) - 1, *root, refs, downsample, 0);

        // Allocate all parents before changing root, so failed allocation cannot
        // destroy or half-expand the previous map.
        std::vector<std::unique_ptr<Node>> parents;
        parents.reserve(expansions.size());
        for (const auto &expansion : expansions)
        {
            auto parent = std::make_unique<Node>();
            parent->geometry = expansion.geometry;
            parent->leaf = false;
            parent->children = std::make_unique<Node::Children>();
            parents.push_back(std::move(parent));
        }
        auto current = std::move(root);
        for (std::size_t i = 0; i < expansions.size(); ++i)
        {
            (*parents[i]->children)[static_cast<std::size_t>(expansions[i].old_child)] = std::move(current);
            tighten(*parents[i]);
            current = std::move(parents[i]);
        }
        root = std::move(current);
        insert(*root, refs, downsample);
    }

    static void append(const Node &node, std::vector<Point3f> &out)
    {
        if (node.leaf) { out.insert(out.end(), node.points.begin(), node.points.end()); return; }
        for (const auto &child : *node.children) if (child) append(*child, out);
    }

    static void append_neighbors(const Node &node, const Point3d &query, std::vector<Neighbor> &out)
    {
        if (node.leaf)
        {
            for (const auto &point : node.points) out.push_back({point, point_distance(point, query)});
            return;
        }
        for (const auto &child : *node.children) if (child) append_neighbors(*child, query, out);
    }

    static void radius_search(const Node &node, const Point3d &query, double threshold, std::vector<Neighbor> &out)
    {
        const DistanceBounds distances = bounds_distance(node.bounds, query);
        if (distances.lower >= threshold) return;
        if (distances.upper < threshold) { append_neighbors(node, query, out); return; }
        if (node.leaf)
        {
            for (const auto &point : node.points)
            {
                const double distance = point_distance(point, query);
                if (distance < threshold) out.push_back({point, distance});
            }
            return;
        }
        for (const auto &child : *node.children) if (child) radius_search(*child, query, threshold, out);
    }

    struct FartherFirst
    {
        bool operator()(const Neighbor &a, const Neighbor &b) const { return a.distance_squared < b.distance_squared; }
    };
    using Heap = std::priority_queue<Neighbor, std::vector<Neighbor>, FartherFirst>;

    static void knn_search(const Node &node, const Point3d &query, std::size_t k, Heap &heap)
    {
        if (heap.size() == k && bounds_distance(node.bounds, query).lower > heap.top().distance_squared) return;
        if (node.leaf)
        {
            for (const auto &point : node.points)
            {
                const double distance = point_distance(point, query);
                if (heap.size() < k) heap.push({point, distance});
                else if (distance < heap.top().distance_squared)
                {
                    heap.pop();
                    heap.push({point, distance});
                }
            }
            return;
        }
        const int first = code(query, node.geometry);
        if ((*node.children)[static_cast<std::size_t>(first)])
            knn_search(*(*node.children)[static_cast<std::size_t>(first)], query, k, heap);
        for (int i = 0; i < 8; ++i)
            if (i != first && (*node.children)[static_cast<std::size_t>(i)])
                knn_search(*(*node.children)[static_cast<std::size_t>(i)], query, k, heap);
    }

    static std::size_t erase(std::unique_ptr<Node> &node, const Point3d &lo, const Point3d &hi, bool keep_inside)
    {
        if (!node) return 0;
        const bool disjoint = outside(node->bounds, lo, hi);
        const bool enclosed = inside(node->bounds, lo, hi);
        if ((!keep_inside && disjoint) || (keep_inside && enclosed)) return 0;
        if ((!keep_inside && enclosed) || (keep_inside && disjoint))
        {
            const std::size_t count = node->count;
            node.reset();
            return count;
        }
        if (node->leaf)
        {
            const auto original = node->points.size();
            node->points.erase(std::remove_if(node->points.begin(), node->points.end(),
                               [&](const Point3f &p) { return inside(p, lo, hi) != keep_inside; }),
                               node->points.end());
            const auto removed = original - node->points.size();
            if (node->points.empty()) node.reset();
            else tighten(*node);
            return removed;
        }
        std::size_t removed = 0;
        for (auto &child : *node->children) removed += erase(child, lo, hi, keep_inside);
        const bool any = std::any_of(node->children->begin(), node->children->end(),
                                     [](const std::unique_ptr<Node> &p) { return bool(p); });
        if (!any) node.reset();
        else tighten(*node);
        return removed;
    }

    static int inspect(const Node &node, std::vector<DebugNode> &out)
    {
        if (out.size() >= static_cast<std::size_t>(std::numeric_limits<int>::max()))
            throw std::length_error("i-Octree debug node indices exceed int range");
        const int index = static_cast<int>(out.size());
        out.emplace_back();
        out[static_cast<std::size_t>(index)].bounds = node.bounds;
        out[static_cast<std::size_t>(index)].routing_center =
            {node.geometry.center[0], node.geometry.center[1], node.geometry.center[2]};
        out[static_cast<std::size_t>(index)].routing_extent = node.geometry.extent;
        if (node.leaf) out[static_cast<std::size_t>(index)].points = node.points;
        else
            for (int i = 0; i < 8; ++i)
                if ((*node.children)[static_cast<std::size_t>(i)])
                {
                    const int child = inspect(*(*node.children)[static_cast<std::size_t>(i)], out);
                    out[static_cast<std::size_t>(index)].children[static_cast<std::size_t>(i)] = child;
                }
        return index;
    }
};

Octree::Octree(const Config &config) : impl_(std::make_unique<Impl>(config)) {}
Octree::~Octree() = default;
Octree::Octree(Octree &&) noexcept = default;
Octree &Octree::operator=(Octree &&) noexcept = default;

void Octree::initialize(const std::vector<Point3d> &points)
{
    require_numeric_environment();
    const auto converted = quantize(points);
    impl_->initialize(converted);
}
void Octree::update(const std::vector<Point3d> &points, bool downsample)
{
    require_numeric_environment();
    const auto converted = quantize(points);
    impl_->update(converted, downsample);
}
void Octree::clear() { impl_->root.reset(); }

std::size_t Octree::knn(const Point3d &query, int k, std::vector<Neighbor> &out) const
{
    out.clear();
    require_numeric_environment();
    validate_query(query);
    if (k < 0) throw std::invalid_argument("i-Octree k must be nonnegative");
    if (k == 0 || !impl_->root) return 0;
    const std::size_t count = std::min(static_cast<std::size_t>(k), impl_->root->count);
    Impl::Heap heap;
    Impl::knn_search(*impl_->root, query, count, heap);
    out.reserve(heap.size());
    while (!heap.empty()) { out.push_back(heap.top()); heap.pop(); }
    std::reverse(out.begin(), out.end());
    return out.size();
}

std::size_t Octree::radius(const Point3d &query, double radius_value, std::vector<Neighbor> &out) const
{
    out.clear();
    require_numeric_environment();
    validate_query(query);
    const double threshold = radius_squared(radius_value);
    if (impl_->root) Impl::radius_search(*impl_->root, query, threshold, out);
    return out.size();
}

std::size_t Octree::erase_box(const Point3d &lo, const Point3d &hi)
{
    require_numeric_environment();
    validate_box(lo, hi);
    return Impl::erase(impl_->root, lo, hi, false);
}
std::size_t Octree::retain_box(const Point3d &lo, const Point3d &hi)
{
    require_numeric_environment();
    validate_box(lo, hi);
    return Impl::erase(impl_->root, lo, hi, true);
}
std::size_t Octree::size() const { return impl_->root ? impl_->root->count : 0; }
std::vector<Point3f> Octree::snapshot() const
{
    std::vector<Point3f> points;
    if (impl_->root) { points.reserve(impl_->root->count); Impl::append(*impl_->root, points); }
    return points;
}
std::vector<DebugNode> Octree::debug_nodes() const
{
    std::vector<DebugNode> nodes;
    if (impl_->root) Impl::inspect(*impl_->root, nodes);
    return nodes;
}
}  // namespace pv_lio_plus::i_octree_native
