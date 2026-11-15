/* Copyright (C) 2026 PV-LIO-PLUS contributors. SPDX-License-Identifier: GPL-2.0-or-later */
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cfenv>
#include <cmath>
#include <limits>
#include <random>
#include <tuple>
#include <vector>
#if defined(__x86_64__) || defined(__i386__)
#include <xmmintrin.h>
#endif
#include "map_manager/native/i_octree/i_octree.hpp"
#ifndef I_OCTREE_NATIVE_ONLY
#include "map_manager/map_manager.cpp"
#endif

namespace {
namespace io = pv_lio_plus::i_octree_native;
using io::Point3d;
using io::Point3f;
using io::Neighbor;
using io::Octree;

Point3f AsFloat(const Point3d &p) { return {float(p.x), float(p.y), float(p.z)}; }
auto Key(const Point3f &p) { return std::make_tuple(p.x, p.y, p.z); }
void SortPoints(std::vector<Point3f> &p) {
    std::sort(p.begin(),p.end(),[](const Point3f &a,const Point3f &b){return Key(a)<Key(b);});
}
void ExpectSamePoints(std::vector<Point3f> actual, std::vector<Point3f> expected) {
    SortPoints(actual); SortPoints(expected);
    ASSERT_EQ(actual.size(),expected.size());
    for (std::size_t i=0; i<actual.size(); ++i) EXPECT_EQ(Key(actual[i]),Key(expected[i])) << i;
}
void ExpectSameNodes(const std::vector<io::DebugNode> &a,const std::vector<io::DebugNode> &b) {
    ASSERT_EQ(a.size(),b.size());
    for (std::size_t i=0; i<a.size(); ++i) {
        EXPECT_EQ(Key(a[i].bounds.lo),Key(b[i].bounds.lo));
        EXPECT_EQ(Key(a[i].bounds.hi),Key(b[i].bounds.hi));
        EXPECT_EQ(a[i].children,b[i].children);
        EXPECT_DOUBLE_EQ(a[i].routing_center.x,b[i].routing_center.x);
        EXPECT_DOUBLE_EQ(a[i].routing_center.y,b[i].routing_center.y);
        EXPECT_DOUBLE_EQ(a[i].routing_center.z,b[i].routing_center.z);
        EXPECT_DOUBLE_EQ(a[i].routing_extent,b[i].routing_extent);
        ASSERT_EQ(a[i].points.size(),b[i].points.size());
        for (std::size_t j=0; j<a[i].points.size(); ++j) EXPECT_EQ(Key(a[i].points[j]),Key(b[i].points[j]));
    }
}
// Deliberately separate scalar reference for the frozen binary64 operation order.
double RefDistance(const Point3f &p, const Point3d &q) {
    const double dx=std::abs(double(p.x)-q.x), dy=std::abs(double(p.y)-q.y), dz=std::abs(double(p.z)-q.z);
    const double sx=dx*dx, sy=dy*dy, sz=dz*dz;
    return (sx+sy)+sz;
}
bool Inside(const Point3f &p,const Point3d &lo,const Point3d &hi) {
    return p.x>=lo.x && p.x<=hi.x && p.y>=lo.y && p.y<=hi.y && p.z>=lo.z && p.z<=hi.z;
}
std::vector<Point3f> AuditNode(const std::vector<io::DebugNode> &nodes, int index,
                               std::vector<bool> &visited, const Point3d &query) {
    EXPECT_GE(index,0); EXPECT_LT(static_cast<std::size_t>(index),nodes.size());
    if (index<0 || static_cast<std::size_t>(index)>=nodes.size()) return {};
    EXPECT_FALSE(visited[static_cast<std::size_t>(index)]);
    visited[static_cast<std::size_t>(index)]=true;
    const auto &node=nodes[static_cast<std::size_t>(index)];
    std::vector<Point3f> points=node.points;
    bool child_exists=false;
    for (int child:node.children) if (child>=0) {
        child_exists=true;
        auto child_points=AuditNode(nodes,child,visited,query);
        points.insert(points.end(),child_points.begin(),child_points.end());
    }
    EXPECT_FALSE(child_exists && !node.points.empty());
    EXPECT_FALSE(points.empty());
    if (points.empty()) return points;
    Point3f lo=points.front(),hi=lo;
    const auto bounds=io::aabb_distance_bounds(node.bounds,query);
    for (const auto &p:points) {
        lo.x=std::min(lo.x,p.x); lo.y=std::min(lo.y,p.y); lo.z=std::min(lo.z,p.z);
        hi.x=std::max(hi.x,p.x); hi.y=std::max(hi.y,p.y); hi.z=std::max(hi.z,p.z);
        const double dist=RefDistance(p,query);
        EXPECT_LE(bounds.lower,dist);
        EXPECT_GE(bounds.upper,dist);
    }
    EXPECT_EQ(Key(lo),Key(node.bounds.lo)); EXPECT_EQ(Key(hi),Key(node.bounds.hi));
    return points;
}
void Audit(const Octree &tree,const std::vector<Point3f> &expected,Point3d query={0,0,0}) {
    EXPECT_EQ(tree.size(),expected.size());
    ExpectSamePoints(tree.snapshot(),expected);
    const auto nodes=tree.debug_nodes();
    if (expected.empty()) { EXPECT_TRUE(nodes.empty()); return; }
    ASSERT_FALSE(nodes.empty());
    std::vector<bool> visited(nodes.size(),false);
    ExpectSamePoints(AuditNode(nodes,0,visited,query),expected);
    for (bool seen:visited) EXPECT_TRUE(seen);
}
void CompareQueries(const Octree &tree,const std::vector<Point3f> &expected,Point3d q,int k,double r) {
    std::vector<Neighbor> found{{{99,99,99},-1}};
    EXPECT_EQ(tree.knn(q,k,found),std::min(expected.size(),static_cast<std::size_t>(k)));
    std::vector<double> expected_dist;
    for (const auto &p:expected) expected_dist.push_back(RefDistance(p,q));
    std::sort(expected_dist.begin(),expected_dist.end());
    ASSERT_EQ(found.size(),std::min(expected.size(),static_cast<std::size_t>(k)));
    for (std::size_t i=0; i<found.size(); ++i) {
        EXPECT_DOUBLE_EQ(found[i].distance_squared,expected_dist[i]);
        EXPECT_DOUBLE_EQ(found[i].distance_squared,RefDistance(found[i].point,q));
    }
    const auto radius_count=tree.radius(q,r,found);
    EXPECT_EQ(radius_count,found.size());
    std::vector<Point3f> actual,reference;
    for (const auto &n:found) { EXPECT_LT(n.distance_squared,r*r); actual.push_back(n.point); }
    for (const auto &p:expected) if (RefDistance(p,q)<r*r) reference.push_back(p);
    ExpectSamePoints(actual,reference);
}

TEST(IOctreeNative, EmptyCountsAndStableOutputs) {
    Octree tree;
    std::vector<Neighbor> result{{{1,2,3},42}};
    EXPECT_EQ(tree.knn({0,0,0},5,result),0u); EXPECT_TRUE(result.empty());
    result.push_back({{1,2,3},42});
    EXPECT_EQ(tree.radius({0,0,0},1,result),0u); EXPECT_TRUE(result.empty());
    tree.initialize({}); EXPECT_EQ(tree.size(),0u);
    tree.update({},true); EXPECT_TRUE(tree.snapshot().empty());
    tree.update({{1,2,3}},true); EXPECT_EQ(tree.size(),1u);
    EXPECT_EQ(tree.knn({0,0,0},7,result),1u); EXPECT_EQ(result.size(),1u);
    EXPECT_EQ(tree.knn({0,0,0},0,result),0u); EXPECT_TRUE(result.empty());
    result.push_back({{1,2,3},42});
    EXPECT_THROW(tree.knn({0,0,0},-1,result),std::invalid_argument); EXPECT_TRUE(result.empty());
    tree.clear(); tree.clear(); Audit(tree,{});
}

TEST(IOctreeNative, BoundingCounterexampleDoubleQueryAndStrictRadius) {
    Octree tree;
    tree.initialize({{16777216.,0,0},{16777218.,0,0}});
    std::vector<Neighbor> found;
    ASSERT_EQ(tree.radius({16777216.,0,0},1.9,found),1u);
    EXPECT_EQ(found[0].point.x,16777216.f);
    EXPECT_EQ(tree.radius({16777216.,0,0},2.,found),1u); // strict equality excludes second
    EXPECT_EQ(tree.knn({16777216.,0,0},9,found),2u);
    tree.initialize({{0,0,0},{2,2,0}});
    const Point3d q{1.+std::ldexp(1.,-26),1.-std::ldexp(1.,-27),0};
    ASSERT_EQ(tree.knn(q,1,found),1u);
    EXPECT_EQ(Key(found[0].point),Key(Point3f{2,2,0}));
    EXPECT_DOUBLE_EQ(found[0].distance_squared,RefDistance(found[0].point,q));
}

TEST(IOctreeNative, IncrementsDeletionAndRandomBruteForce) {
    io::Config config; config.bucket_size=3; config.min_extent=.01;
    Octree tree(config);
    std::vector<Point3f> reference;
    std::mt19937 gen(20261115);
    std::uniform_int_distribution<int> draw(-32,32);
    for (int step=0; step<30; ++step) {
        std::vector<Point3d> batch;
        for (int i=0; i<5; ++i) {
            Point3d p{draw(gen)*.25,draw(gen)*.25,draw(gen)*.25};
            batch.push_back(p); reference.push_back(AsFloat(p));
        }
        tree.update(batch,false);
        if (step%7==6) {
            const Point3d lo{-1,-2,-3},hi{2,1,3};
            const auto before=reference.size();
            reference.erase(std::remove_if(reference.begin(),reference.end(),[&](const Point3f &p){return Inside(p,lo,hi);}),reference.end());
            EXPECT_EQ(tree.erase_box(lo,hi),before-reference.size());
        }
        const Point3d q{draw(gen)*.25,draw(gen)*.25,draw(gen)*.25};
        Audit(tree,reference,q);
        CompareQueries(tree,reference,q,7,4.5);
    }
    const Point3d lo{-4,-4,-4},hi{4,4,4};
    const auto before=reference.size();
    reference.erase(std::remove_if(reference.begin(),reference.end(),[&](const Point3f &p){return !Inside(p,lo,hi);}),reference.end());
    EXPECT_EQ(tree.retain_box(lo,hi),before-reference.size());
    Audit(tree,reference);
}

TEST(IOctreeNative, ClosedDeletionRetainAndBatchDownsampling) {
    io::Config config; config.bucket_size=8; config.min_extent=1.;
    Octree tree(config);
    tree.initialize({{.1,.1,.1},{.3,.1,.1},{.5,.1,.1}});
    const auto prior=tree.snapshot();
    tree.update({{.2,.1,.1},{.4,.1,.1}},true);  // old size 3 > bucket/8
    Audit(tree,prior);
    tree.update({{.2,.1,.1},{.4,.1,.1}},false);
    EXPECT_EQ(tree.size(),5u);
    tree.initialize({{-1,0,0},{0,0,0},{1,0,0},{2,0,0}});
    EXPECT_EQ(tree.retain_box({-1,0,0},{1,0,0}),1u);
    Audit(tree,{{-1,0,0},{0,0,0},{1,0,0}});
    EXPECT_EQ(tree.erase_box({0,0,0},{1,0,0}),2u);
    Audit(tree,{{-1,0,0}});
    EXPECT_EQ(tree.erase_box({-1,0,0},{-1,0,0}),1u);
    Audit(tree,{});
    tree.update({{-2,-3,-4}},false);
    Audit(tree,{{-2,-3,-4}});
}

TEST(IOctreeNative, DownsamplingKeepsNativeWholeBatchAndNewSubtreeRules) {
    io::Config config; config.bucket_size=8; config.min_extent=1.;
    Octree tree(config);
    tree.initialize({{.2,.2,.2}});
    const std::vector<Point3d> batch(20,Point3d{.2,.2,.2});
    tree.update(batch,true);  // existing size == bucket/8: all 20 are accepted
    EXPECT_EQ(tree.size(),21u);
    tree.update({{.2,.2,.2}},true);  // prior size now exceeds threshold
    EXPECT_EQ(tree.size(),21u);
    config.bucket_size=1; config.min_extent=.01;
    Octree growing(config);
    growing.initialize({{-4,0,0},{4,0,0}});
    growing.update(std::vector<Point3d>(10,Point3d{100,0,0}),true);
    EXPECT_EQ(growing.size(),12u);  // newly created subtree is built without the rejection rule
}

TEST(IOctreeNative, RejectionsPreserveExistingTree) {
    io::Config config; config.bucket_size=1; config.min_extent=.01; config.max_depth=1;
    Octree tree(config); tree.initialize({{0,0,0}});
    const auto before=tree.snapshot();
    const auto before_nodes=tree.debug_nodes();
    EXPECT_THROW(tree.initialize({{-1,0,0},{1,0,0},{1,0,0}}),std::range_error);
    Audit(tree,before);
    ExpectSameNodes(tree.debug_nodes(),before_nodes);
    EXPECT_THROW(tree.update({{1,0,0}},false),std::range_error);
    Audit(tree,before);
    ExpectSameNodes(tree.debug_nodes(),before_nodes);
    const double nan=std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(tree.update({{0,0,0},{nan,0,0}},false),std::invalid_argument);
    EXPECT_THROW(tree.initialize({{std::numeric_limits<double>::max(),0,0}}),std::invalid_argument);
    EXPECT_THROW(tree.erase_box({1,0,0},{0,0,0}),std::invalid_argument);
    Audit(tree,before);
    ExpectSameNodes(tree.debug_nodes(),before_nodes);
    std::vector<Neighbor> out{{{1,2,3},1}};
    EXPECT_THROW(tree.knn({nan,0,0},1,out),std::invalid_argument); EXPECT_TRUE(out.empty());
    EXPECT_THROW(tree.radius({0,0,0},std::numeric_limits<double>::denorm_min(),out),std::invalid_argument);
}

TEST(IOctreeNative, AllowedRootExpansionThenOverdeepSplitRejectsWholeBatch) {
    io::Config config; config.bucket_size=1; config.min_extent=.001; config.max_depth=2;
    Octree control(config), tree(config);
    control.initialize({{0,0,0}});
    tree.initialize({{0,0,0}});
    const auto before=tree.snapshot();
    const auto before_nodes=tree.debug_nodes();
    ASSERT_EQ(before_nodes.size(),1u);
    ASSERT_NO_THROW(control.update({{.04,0,0}},false));
    ASSERT_EQ(control.size(),2u);
    ASSERT_GT(control.debug_nodes()[0].routing_extent,before_nodes[0].routing_extent);
    // Expansion fits the budget by itself; the second point routes to the old
    // leaf and would require a split one level beyond that budget.
    EXPECT_THROW(tree.update({{.04,0,0},{0,0,0}},false),std::range_error);
    Audit(tree,before);
    ExpectSameNodes(tree.debug_nodes(),before_nodes);
}

TEST(IOctreeNative, AllowedExpansionThenNewSubtreeOverdeepRejectsWholeBatch) {
    io::Config config; config.bucket_size=1; config.min_extent=1e-8; config.max_depth=1;
    Octree control(config), tree(config);
    control.initialize({{0,0,0}});
    tree.initialize({{0,0,0}});
    const auto before=tree.snapshot();
    const auto before_nodes=tree.debug_nodes();
    ASSERT_EQ(before_nodes.size(),1u);
    ASSERT_NO_THROW(control.update({{.02,0,0}},false));
    ASSERT_EQ(control.size(),2u);
    ASSERT_GT(control.debug_nodes()[0].routing_extent,before_nodes[0].routing_extent);
    // The expansion alone fits, but two points in the new child require an
    // additional split beyond the allowed depth.
    EXPECT_THROW(tree.update({{.02,0,0},{.02,0,0}},false),std::range_error);
    Audit(tree,before);
    ExpectSameNodes(tree.debug_nodes(),before_nodes);
}

TEST(IOctreeNative, ExtremeInputsAndFloatingPointMode) {
    Octree tree;
    const float f=std::numeric_limits<float>::max();
    const float tiny=std::numeric_limits<float>::denorm_min();
    tree.initialize({{-double(f),0,0},{double(f),0,0},{double(tiny),0,0}});
    Audit(tree,{{-f,0,0},{f,0,0},{tiny,0,0}});
    const auto before_nodes=tree.debug_nodes();
    std::vector<Neighbor> out;
    ASSERT_EQ(tree.knn({double(f),0,0},3,out),3u);
    EXPECT_DOUBLE_EQ(out[0].distance_squared,0.);
    EXPECT_TRUE(std::isfinite(out[2].distance_squared));
    const int old_round=std::fegetround();
    if (std::fesetround(FE_DOWNWARD)==0) {
        EXPECT_THROW(tree.update({{0,0,0}},false),std::runtime_error);
        std::fesetround(old_round);
    }
#if defined(__x86_64__) || defined(__i386__)
    const unsigned int old_csr=_mm_getcsr();
    _mm_setcsr(old_csr | 0x8000u);
    EXPECT_THROW(tree.knn({0,0,0},1,out),std::runtime_error);
    _mm_setcsr(old_csr);
    _mm_setcsr(old_csr | 0x40u);
    EXPECT_THROW(tree.update({{0,0,0}},false),std::runtime_error);
    _mm_setcsr(old_csr);
#endif
    EXPECT_EQ(tree.size(),3u);
    ExpectSameNodes(tree.debug_nodes(),before_nodes);
}

TEST(IOctreeNative, LongFiniteDuplicatePathKeepsAllPoints) {
    io::Config config;
    config.bucket_size=1;
    config.min_extent=double(std::numeric_limits<float>::denorm_min());
    Octree tree(config);
    const double far=double(std::numeric_limits<float>::max());
    tree.initialize({{0,0,0},{0,0,0},{far,0,0}});
    EXPECT_EQ(tree.size(),3u);
    EXPECT_GT(tree.debug_nodes().size(),200u);
    std::vector<Neighbor> out;
    ASSERT_EQ(tree.knn({0,0,0},4,out),3u);
    EXPECT_DOUBLE_EQ(out[0].distance_squared,0.);
    EXPECT_DOUBLE_EQ(out[1].distance_squared,0.);
    EXPECT_TRUE(std::isfinite(out[2].distance_squared));
}

#ifndef I_OCTREE_NATIVE_ONLY
using namespace pv_lio_plus;
MapPoint M(double x,double y,double z) {
    MapPoint p; p.point_world=V3D(x,y,z); p.point_lidar=V3D(1,2,3); p.cov_world=M3D::Identity()*.01; return p;
}
PlaneMatchList SearchMap(MapManager &map,const MapPoint &p) {
    PlaneMatchList matches; std::vector<V3D> nonmatch; map.search({p},matches,nonmatch); return matches;
}
TEST(IOctreeAdapter, LifecycleRangeDegeneracyAndWindow) {
    MapManagerConfig config; config.type=MapType::IOctree; config.nearest_max_range=5.;
    MapManager map(config);
    EXPECT_EQ(ParseMapType("i-octree"),MapType::IOctree);
    EXPECT_STREQ(MapTypeName(MapType::IOctree),"i_octree");
    map.initialize({}); EXPECT_TRUE(map.initialized()); EXPECT_EQ(map.size(),0u);
    MapPointList plane{M(15,10,1),M(5,10,1),M(10,15,1),M(10,5,1),M(13,14,1)};
    map.initialize(plane);
    ASSERT_EQ(map.size(),5u);
    EXPECT_EQ(SearchMap(map,M(10,10,1)).size(),1u);  // fifth neighbor on 5 m boundary
    auto invalid=M(10,10,1); invalid.point_world.x()=std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(SearchMap(map,invalid).empty());
    auto invalid_cov=M(10,10,1); invalid_cov.cov_world(0,0)=std::numeric_limits<double>::infinity();
    EXPECT_TRUE(SearchMap(map,invalid_cov).empty());
    const auto before_revision=map.revision();
    auto bad_config=config; bad_config.i_octree_bucket_size=0;
    EXPECT_THROW(map.configure(bad_config),std::invalid_argument);
    bad_config=config; bad_config.i_octree_min_extent=0.;
    EXPECT_THROW(map.configure(bad_config),std::invalid_argument);
    bad_config=config; bad_config.nearest_max_range=std::numeric_limits<double>::infinity();
    EXPECT_THROW(map.configure(bad_config),std::invalid_argument);
    EXPECT_THROW(map.initialize({invalid}),std::invalid_argument);
    EXPECT_THROW(map.update({M(0,0,0),invalid},1),std::invalid_argument);
    EXPECT_EQ(map.size(),5u);
    EXPECT_EQ(map.point_count(),5u);
    EXPECT_EQ(map.revision(),before_revision);
    MapWindow w; w.initialized=true; w.min_bound=V3D(5,5,1); w.max_bound=V3D(15,15,1);
    map.erase_outside_window(w); EXPECT_EQ(map.size(),5u);
    w.max_bound.x()=std::nextafter(15.,0.); map.erase_outside_window(w); EXPECT_EQ(map.size(),4u);
    EXPECT_TRUE(SearchMap(map,M(10,10,1)).empty());
    map.initialize({M(0,0,0),M(0,0,0),M(0,0,0),M(0,0,0),M(0,0,0)});
    EXPECT_TRUE(SearchMap(map,M(0,0,0)).empty());
    map.clear(); EXPECT_FALSE(map.initialized()); EXPECT_TRUE(map.snapshot()->empty());
    map.update(plane,2); EXPECT_EQ(map.size(),5u);
}
#endif
}  // namespace

int main(int argc,char **argv) { testing::InitGoogleTest(&argc,argv); return RUN_ALL_TESTS(); }
