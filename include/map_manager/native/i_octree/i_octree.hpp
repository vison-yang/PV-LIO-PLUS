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
// PV-LIO-PLUS adaptation: source and behavior differences are in SOURCE.md.
#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <vector>

// Private native/test interface; no raw leaf pointers escape an operation.
namespace pv_lio_plus::i_octree_native
{
struct Point3f { float x = 0, y = 0, z = 0; };
struct Point3d { double x = 0, y = 0, z = 0; };
struct Aabb { Point3f lo, hi; };
struct Neighbor { Point3f point; double distance_squared = 0; };
struct DistanceBounds { double lower = 0, upper = 0; };
struct Config
{
    std::size_t bucket_size = 32;
    double min_extent = 0.01;
    std::size_t max_depth = 512;
};

// Nodes are copied in depth-first order; root index is 0, missing child is -1.
// Only leaves carry points. This seam is for structural/independent oracles.
struct DebugNode
{
    Aabb bounds;
    Point3d routing_center;
    double routing_extent = 0;
    std::array<int, 8> children{{-1, -1, -1, -1, -1, -1, -1, -1}};
    std::vector<Point3f> points;
};

// These functions check the calling thread's IEEE mode; invalid input throws.
void require_numeric_environment();
double distance_squared(const Point3f &point, const Point3d &query);
double radius_squared(double radius);
DistanceBounds aabb_distance_bounds(const Aabb &box, const Point3d &query);

class Octree
{
   public:
    explicit Octree(const Config &config = Config{});
    ~Octree();
    Octree(const Octree &) = delete;
    Octree &operator=(const Octree &) = delete;
    Octree(Octree &&) noexcept;
    Octree &operator=(Octree &&) noexcept;

    // Input is validated then quantized once to float using RN/ties-to-even.
    // Invalid/domain/depth failures throw before modifying the old tree.
    void initialize(const std::vector<Point3d> &points);
    void update(const std::vector<Point3d> &points, bool downsample);
    void clear();

    // Outputs are cleared on entry. k=0 yields empty, k<0 throws. Distances are
    // double squared metres, nearest-first for kNN; radius is strictly < r^2.
    std::size_t knn(const Point3d &query, int k, std::vector<Neighbor> &out) const;
    std::size_t radius(const Point3d &query, double radius, std::vector<Neighbor> &out) const;

    // Physical deletion only; boxes and retained windows are closed.
    std::size_t erase_box(const Point3d &lo, const Point3d &hi);
    std::size_t retain_box(const Point3d &lo, const Point3d &hi);
    std::size_t size() const;
    std::vector<Point3f> snapshot() const;
    std::vector<DebugNode> debug_nodes() const;

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace pv_lio_plus::i_octree_native
