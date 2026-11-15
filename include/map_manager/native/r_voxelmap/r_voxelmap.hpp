/*
 * R-VoxelMap core extracted from author revision 50ffbbab (see SOURCE.md).
 * Local safety and integration changes are documented alongside this file.
 */
#ifndef PV_LIO_PLUS_R_VOXELMAP_H
#define PV_LIO_PLUS_R_VOXELMAP_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <limits>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace r_voxelmap_ns {
using std::unordered_map;
using std::unordered_set;
using std::vector;
static int plane_id = 0;
// R-VoxelMap config
typedef struct voxel_map_config {
  float voxel_size;
  int max_layer;
  std::vector<int> layer_point_size;
  float planer_threshold;
  int stop_reinit_threshold;
  int reinit_threshold;
  std::vector<int> reinit_size_vec;
  int update_size_threshold;
  double update_inlier_distance_threshold;
  int max_points_size;
  int max_cov_points_size;
  int max_root_points_size;
  int lru_capacity;

  int valid_check_max_layer;
  int valid_check_min_points_size;
  int valid_check_resolution;

  int ransac_max_iter;
  int ransac_sample_num;
  double ransac_inlier_distance_threshold;
  double ransac_isplane_p_threshold;
  int sample_seed;

  std::vector<int> iter2layer_;
  int direct_match_layer;

  bool pub_cache_en;
  bool pub_grid_map_en;
  bool pub_noupdate_voxel_en;
  bool pub_effect_en;

  int pub_max_voxel_layer;
} voxel_map_config;

// a point to plane matching structure
typedef struct ptpl {
  Eigen::Vector3d normal;
  Eigen::Vector3d center;
  Eigen::Matrix<double, 6, 6> plane_cov;
  double d;
  int layer;
} ptpl;

// 3D point with covariance
typedef struct pointWithCov {
  Eigen::Vector3d point;
  Eigen::Matrix3d cov;
} pointWithCov;

typedef struct Plane {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Vector3d y_normal = Eigen::Vector3d::Zero();
  Eigen::Vector3d x_normal = Eigen::Vector3d::Zero();
  Eigen::Matrix3d sum_ppt = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  Eigen::Matrix<double, 6, 6> plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
  float radius = 0;
  float min_eigen_value = 1;
  float mid_eigen_value = 1;
  float max_eigen_value = 1;
  float d = 0;
  int points_size = 0;

  bool is_plane = false;
  bool is_init = false;
  int id = 0;
  // is_update and last_update_points_size are only for publish plane
  bool is_update = false;
  bool cov_need_update = false;
  int last_update_points_size = 0;
  bool update_enable = true;
} Plane;

class VOXEL_LOC {
public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

struct VoxelHash {
  std::size_t operator()(const VOXEL_LOC &v) const noexcept {
    std::size_t h = std::hash<int64_t>{}(v.x);
    h ^= std::hash<int64_t>{}(v.y) + 0x9e3779b9U + (h << 6) + (h >> 2);
    h ^= std::hash<int64_t>{}(v.z) + 0x9e3779b9U + (h << 6) + (h >> 2);
    return h;
  }
};
struct PairHash {
  std::size_t operator()(const std::pair<int, int> &p) const noexcept {
    auto h = std::hash<int>{}(p.first);
    return h ^ (std::hash<int>{}(p.second) + 0x9e3779b9U + (h << 6) + (h >> 2));
  }
};

// Floor produces the same cell on both sides of a negative integer boundary.
inline bool voxel_location(const Eigen::Vector3d &point, double size, VOXEL_LOC &loc) {
  if (!point.allFinite() || !std::isfinite(size) || size <= 0.0) return false;
  int64_t key[3];
  const double limit = std::ldexp(1.0, 63);
  for (int axis = 0; axis < 3; ++axis) {
    const double cell = std::floor(point[axis] / size);
    if (!std::isfinite(cell) || cell < -limit || cell >= limit) return false;
    key[axis] = static_cast<int64_t>(cell);
  }
  loc = VOXEL_LOC(key[0], key[1], key[2]);
  return true;
}

// Eigen returns ascending eigenvalues. The plane normal and its covariance
// are undefined when its smallest eigenvalue has no resolvable separation.
inline bool valid_plane_spectrum(const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> &solver,
                                 const Eigen::Vector3d &center) {
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
      !solver.eigenvectors().allFinite() || !center.allFinite()) return false;
  const Eigen::Vector3d &values = solver.eigenvalues();
  // Native covariance subtracts E[p]E[p]^T from E[pp^T]; its rounding
  // uncertainty also grows with the world-coordinate magnitude.
  const double scale = std::max(values.cwiseAbs().maxCoeff(), center.squaredNorm());
  if (!std::isfinite(scale)) return false;
  const double gap_01 = values[1] - values[0];
  const double gap_02 = values[2] - values[0];
  const double tolerance = 64.0 * std::numeric_limits<double>::epsilon() * scale;
  return values[2] >= 0.0 && std::isfinite(gap_01) && std::isfinite(gap_02) &&
         gap_01 > tolerance && gap_02 > tolerance;
}

class OctoTree {
public:
  std::list<pointWithCov> plane_points_; // all plane points in an octo tree
  std::list<pointWithCov> not_plane_points_; // all not plane points in an octo tree
  std::list<pointWithCov> new_points_;  // new points in an octo tree

  Plane *plane_ptr_;
  OctoTree *root;
  int max_layer_;
  bool indoor_mode_;
  int layer_;
  int octo_state_; // 0 is end of tree, 1 is not
  OctoTree *leaves_[8];
  double voxel_center_[3]; // x, y, z
  std::vector<int> layer_point_size_;
  float quater_length_;
  float planer_threshold_;
  int init_size_threshold_;
  int update_size_threshold_;

  int all_points_num_root_;
  int all_points_num_;
  int new_points_num_;

  int max_points_size_;
  int max_cov_points_size_;
  bool init_octo_;
  bool update_cov_enable_;
  bool update_enable_;
  bool root_update_enable_;

  bool stop_reinit_;
  int reinit_state_;

  // config
  voxel_map_config config_;

  OctoTree(int layer, const voxel_map_config& config)
      : layer_(layer), config_(config) {
    max_layer_ = config.max_layer;
    layer_point_size_ = config.layer_point_size;
    max_points_size_ = config.max_points_size;
    max_cov_points_size_ = config.max_cov_points_size;
    planer_threshold_ = config.planer_threshold;
    update_size_threshold_ = config.update_size_threshold;

    new_points_.clear();
    plane_points_.clear();
    not_plane_points_.clear();
    octo_state_ = 0;
    new_points_num_ = 0;
    all_points_num_ = 0;
    all_points_num_root_ = 0;
    init_octo_ = false;
    root_update_enable_ = true;
    update_enable_ = true;
    update_cov_enable_ = true;
    init_size_threshold_ = layer_point_size_[layer_];
    stop_reinit_ = false;
    reinit_state_ = 0;

    for (int i = 0; i < 8; i++) {
      leaves_[i] = nullptr;
    }
    plane_ptr_ = new Plane;
  }

  ~OctoTree() {
    delete plane_ptr_;
    for (int i = 0; i < 8; i++) {
      if (leaves_[i] != nullptr) {
        delete leaves_[i];
      }
    }
  }

  void reset_degenerate_plane() {
    plane_ptr_->is_plane = false;
    plane_ptr_->is_init = true;
    plane_ptr_->cov_need_update = false;
    plane_ptr_->center.setZero();
    plane_ptr_->normal.setZero();
    plane_ptr_->x_normal.setZero();
    plane_ptr_->y_normal.setZero();
    plane_ptr_->sum_ppt.setZero();
    plane_ptr_->covariance.setZero();
    plane_ptr_->plane_cov.setZero();
    plane_ptr_->points_size = 0;
    plane_ptr_->last_update_points_size = 0;
    plane_ptr_->radius = 0;
    plane_ptr_->d = 0;
  }

  // Preserve every point on a failed/degenerate plane fit for the native
  // outlier path to retain or recursively distribute.
  void reject_degenerate_plane() {
    reset_degenerate_plane();
    not_plane_points_.splice(not_plane_points_.end(), plane_points_);
    not_plane_points_.splice(not_plane_points_.end(), new_points_);
  }

  void rebuild_degenerate_plane() {
    new_points_.splice(new_points_.end(), plane_points_);
    new_points_.splice(new_points_.end(), not_plane_points_);
    reset_degenerate_plane();
    reinit_octo_tree();
  }

  // using new_points_ to init plane
  void init_plane_ransac() {
    Plane *plane = plane_ptr_;
    all_points_num_ = new_points_.size();

    // It is not ideal to call srand here,
    // However, placing it at the beginning of the program still results in random outcomes, making reproduction difficult.
    srand(config_.sample_seed);
    //* RANSAC
    if(!ransac()){
      reject_degenerate_plane();
      return;
    }

    plane->plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
    plane->sum_ppt = Eigen::Matrix3d::Zero();
    plane->covariance = Eigen::Matrix3d::Zero();
    plane->center = Eigen::Vector3d::Zero();
    plane->normal = Eigen::Vector3d::Zero();
    plane->points_size = 0;
    plane->radius = 0;

    // Calculate plane parameters using the plane_points_ list
    for (const auto &pv : plane_points_) {
      plane->sum_ppt += pv.point * pv.point.transpose();
      plane->center += pv.point;
      plane->points_size++;
    }

    if (plane->points_size < 3) {
      reject_degenerate_plane();
      return;
    }
    plane->center = plane->center / plane->points_size;
    plane->covariance = plane->sum_ppt / plane->points_size - plane->center * plane->center.transpose();
    if (!plane->covariance.allFinite()) {
      reject_degenerate_plane();
      return;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(plane->covariance);
    if (!valid_plane_spectrum(es, plane->center)) {
      reject_degenerate_plane();
      return;
    }
    Eigen::Matrix3d evecs = es.eigenvectors();
    Eigen::Vector3d evals = es.eigenvalues();
    const int evalsMin = 0;
    const int evalsMid = 1;
    const int evalsMax = 2;

    //* plane valid check
    plane->normal = evecs.real().col(evalsMin);
    plane->y_normal = evecs.real().col(evalsMid);
    plane->x_normal = plane->y_normal.cross(plane->normal);

    if(layer_ <= config_.valid_check_max_layer && plane_points_.size() >= config_.valid_check_min_points_size){
      if(!plane_valid_check()){
        plane->points_size = 0;
        plane->is_init = true;
        plane->is_plane = false;
        return;
      }
    }

    // new_points_ temporary storage of outlier points after valid check
    if(new_points_.size() > 0){
      plane->center = plane->center * plane->points_size;
      for(auto iter = new_points_.begin(); iter != new_points_.end(); iter++){
        Eigen::Vector3d point = iter->point;
        plane->sum_ppt -= point * point.transpose();
        plane->center -= point;
      }
      plane->points_size -= new_points_.size();
      if (plane->points_size < 3) {
        reject_degenerate_plane();
        return;
      }
      plane->center = plane->center / plane->points_size;
      plane->covariance = plane->sum_ppt / plane->points_size - plane->center * plane->center.transpose();
      if (!plane->covariance.allFinite()) {
        reject_degenerate_plane();
        return;
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> refined(plane->covariance);
      if (!valid_plane_spectrum(refined, plane->center)) {
        reject_degenerate_plane();
        return;
      }
      evecs = refined.eigenvectors();
      evals = refined.eigenvalues();

      not_plane_points_.splice(not_plane_points_.end(), new_points_);
    }

    if(evals(evalsMin) < planer_threshold_){
      // plane covariance calculation
      Eigen::Matrix3d J_Q;
      J_Q << 1.0 / plane->points_size, 0, 0, 0, 1.0 / plane->points_size, 0, 0, 0,
          1.0 / plane->points_size;
      // Calculate plane covariance using the plane_points_ list
      for (const auto &pv : plane_points_) {
        Eigen::Matrix<double, 6, 3> J;
        Eigen::Matrix3d F;
        for (int m = 0; m < 3; m++) {
          if (m != (int)evalsMin) {
            Eigen::Matrix<double, 1, 3> F_m =
                (pv.point - plane->center).transpose() /
                ((plane->points_size) * (evals[evalsMin] - evals[m])) *
                (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() +
                 evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
            F.row(m) = F_m;
          } else {
            Eigen::Matrix<double, 1, 3> F_m;
            F_m << 0, 0, 0;
            F.row(m) = F_m;
          }
        }
        J.block<3, 3>(0, 0) = evecs * F;
        J.block<3, 3>(3, 0) = J_Q;
        plane->plane_cov += J * pv.cov * J.transpose();
      }

      if (!plane->plane_cov.allFinite()) {
        reject_degenerate_plane();
        return;
      }

      plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
          evecs.real()(2, evalsMin);
      plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid),
          evecs.real()(2, evalsMid);
      plane->x_normal = plane->y_normal.cross(plane->normal);
      plane->min_eigen_value = evals(evalsMin);
      plane->mid_eigen_value = evals(evalsMid);
      plane->max_eigen_value = evals(evalsMax);
      plane->radius = sqrt(evals(evalsMax));
      plane->d = -(plane->normal(0) * plane->center(0) +
                   plane->normal(1) * plane->center(1) +
                   plane->normal(2) * plane->center(2));
      plane->is_plane = true;
      if (plane->last_update_points_size == 0) {
        plane->last_update_points_size = plane->points_size;
        plane->is_update = true;
      } else if (plane->points_size - plane->last_update_points_size > 100) {
        plane->last_update_points_size = plane->points_size;
        plane->is_update = true;
      }

      if (!plane->is_init) {
        plane->id = plane_id;
        plane_id++;
        plane->is_init = true;
      }

    }
    else {
      if (!plane->is_init) {
        plane->id = plane_id;
        plane_id++;
        plane->is_init = true;
      }
      if (plane->last_update_points_size == 0) {
        plane->last_update_points_size = plane->points_size;
        plane->is_update = true;
      } else if (plane->points_size - plane->last_update_points_size > 100) {
        plane->last_update_points_size = plane->points_size;
        plane->is_update = true;
      }
      not_plane_points_.splice(not_plane_points_.end(), plane_points_);
      plane->is_plane = false;
    }
  }

  void update_plane_cov(){
    Plane *plane = plane_ptr_;
    if (!plane->covariance.allFinite() || plane->points_size < 3) {
      rebuild_degenerate_plane();
      return;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(plane->covariance);
    if (!valid_plane_spectrum(es, plane->center)) {
      rebuild_degenerate_plane();
      return;
    }
    Eigen::Matrix3d evecs = es.eigenvectors();
    Eigen::Vector3d evals = es.eigenvalues();
    constexpr int evalsMin = 0;


    // plane covariance calculation
    Eigen::Matrix3d J_Q;
    J_Q << 1.0 / plane->points_size, 0, 0,
            0, 1.0 / plane->points_size, 0,
            0, 0, 1.0 / plane->points_size;

    plane->plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
    int plane_points_num = 0;
    for (auto &pv : plane_points_) {
      plane_points_num++;
      Eigen::Matrix<double, 6, 3> J;
      Eigen::Matrix3d F = Eigen::Matrix3d::Zero();

      const Eigen::Vector3d point_centered = pv.point - plane->center;
      for (int m = 0; m < 3; m++) {
        if (m != (int)evalsMin) {
          const Eigen::Vector3d evec_m = evecs.col(m);
          F.row(m) = 1.0 / (plane->points_size * (evals[evalsMin] - evals[m])) * point_centered.transpose() *
                      (evec_m * evecs.col(evalsMin).transpose() + evecs.col(evalsMin) * evec_m.transpose());
        }
      }
      J.block<3, 3>(0, 0) = evecs * F;
      J.block<3, 3>(3, 0) = J_Q;
      plane->plane_cov.noalias() += J * pv.cov * J.transpose();
    }
    if (!plane->plane_cov.allFinite()) {
      rebuild_degenerate_plane();
      return;
    }
    plane->cov_need_update = false;
    plane->is_update = true;
    return;
  }

  bool check_and_update(const pointWithCov &pv){
    Plane *plane = plane_ptr_;
    int curr_points_num = plane->points_size;
    // plane_points_num_ > plane->points_size 由于在check_update成功后会直接给plane_points_num_++，但这里要用的还是旧的plane->points_size

    Eigen::Vector3d curr_mean = plane->center;
    Eigen::Matrix3d curr_ppt = plane->sum_ppt;

    Eigen::Vector3d p_vec(pv.point[0], pv.point[1], pv.point[2]);
    Eigen::Vector3d new_mean = (curr_mean * curr_points_num + p_vec) / (curr_points_num + 1);

    // update ppt and cov
    Eigen::Matrix3d new_ppt = curr_ppt + p_vec * p_vec.transpose();
    Eigen::Matrix3d new_cov = new_ppt / (curr_points_num + 1) - new_mean * new_mean.transpose();

    // calculate new cov eigenvalues
    if (!new_cov.allFinite()) return false;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(new_cov);
    if (!valid_plane_spectrum(es, new_mean)) return false;
    Eigen::Vector3d eigen_values = es.eigenvalues();

    // check if min eigenvalue is still less than threshold
    double min_eigen_value = eigen_values(0);
    if(min_eigen_value >= config_.planer_threshold){
      return false;
    }
    else{
      plane->center = new_mean;
      plane->sum_ppt = new_ppt;
      plane->covariance = new_cov;
      plane->points_size = curr_points_num + 1;

      Eigen::Matrix3d evecs = es.eigenvectors();
      plane->normal << evecs.real()(0, 0), evecs.real()(1, 0), evecs.real()(2, 0);
      plane->y_normal << evecs.real()(0, 1), evecs.real()(1, 1), evecs.real()(2, 1);
      plane->x_normal = plane->y_normal.cross(plane->normal);

      plane->min_eigen_value = eigen_values(0);
      plane->mid_eigen_value = eigen_values(1);
      plane->max_eigen_value = eigen_values(2);
      plane->radius = sqrt(eigen_values(2));
      plane->d = -plane->normal.dot(plane->center);
      plane->is_update = true;
      plane->cov_need_update = true;
      return true;
    }
  }

  void init_octo_tree() {
    all_points_num_ = new_points_.size();
    if (all_points_num_ > init_size_threshold_) {
      init_plane_ransac();

      if(not_plane_points_.size() > 0){
        octo_state_ = 1;
        cut_octo_tree();
      }
      else{
        octo_state_ = 0;
      }
      init_octo_ = true;
    }
    new_points_num_ = 0;
  }

  void reinit_octo_tree(){
    collect_points(new_points_, layer_);

    // test
    // for(int i = 0; i < 8; ++i){
    //   if(leaves_[i] != nullptr){
    //     leaves_[i]->check_remain_points();
    //   }
    // }

    for(int i = 0; i < 8; ++i){
      if(leaves_[i] != nullptr){
        delete leaves_[i];
        leaves_[i] = nullptr;
      }
    }

    init_octo_ = false;
    plane_ptr_->is_init = false;

    init_octo_tree();
    return;
  }

  void cut_octo_tree() {
    if (layer_ >= max_layer_) {
      octo_state_ = 0;
      return;
    }
    for (auto it = not_plane_points_.begin(); it != not_plane_points_.end();) {
      auto &pv = *it;
      int xyz[3] = {0, 0, 0};
      if (pv.point[0] > voxel_center_[0]) {
        xyz[0] = 1;
      }
      if (pv.point[1] > voxel_center_[1]) {
        xyz[1] = 1;
      }
      if (pv.point[2] > voxel_center_[2]) {
        xyz[2] = 1;
      }
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
      if (leaves_[leafnum] == nullptr) {
        leaves_[leafnum] = new OctoTree(layer_ + 1, config_);
        leaves_[leafnum]->root = root;
        leaves_[leafnum]->layer_point_size_ = layer_point_size_;
        leaves_[leafnum]->voxel_center_[0] =
            voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
        leaves_[leafnum]->voxel_center_[1] =
            voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
        leaves_[leafnum]->voxel_center_[2] =
            voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
        leaves_[leafnum]->quater_length_ = quater_length_ / 2;
      }

      auto move_it = it++;
      leaves_[leafnum]->new_points_.splice(leaves_[leafnum]->new_points_.end(), not_plane_points_, move_it);
    }
    for (uint i = 0; i < 8; i++) {
      if (leaves_[i] != nullptr) {
        leaves_[i]->init_octo_tree();
      }
    }
  }

  void update_octo_tree() {
    if(!init_octo_){
      init_octo_tree();
      return;
    }

    // old reinit version
    // if(!stop_reinit_ && all_points_num_root_/config_.reinit_threshold > reinit_state_){
    //   reinit_octo_tree();
    //   reinit_state_ = all_points_num_root_/config_.reinit_threshold;
    //   if(all_points_num_root_ > config_.stop_reinit_threshold){
    //     stop_reinit_ = true;
    //   }
    //   return;
    // }

    // new reinit version
    if (!stop_reinit_ && reinit_state_ < static_cast<int>(config_.reinit_size_vec.size()) &&
        all_points_num_root_ > config_.reinit_size_vec[reinit_state_]) {
      reinit_octo_tree();
      while (reinit_state_ < static_cast<int>(config_.reinit_size_vec.size()) &&
             all_points_num_root_ > config_.reinit_size_vec[reinit_state_]) {
        ++reinit_state_;
      }
      stop_reinit_ = reinit_state_ == static_cast<int>(config_.reinit_size_vec.size());
      return;
    }

    unordered_set<OctoTree*> update_cache;
    for(auto &pv: new_points_){
        OctoTree* voxel = push_to_subvoxel(pv);
        if(voxel != nullptr){
          update_cache.insert(voxel);
        }
    }
    new_points_.clear();

    // A node may rebuild itself and delete descendants. Capture all depths
    // before updating anything, then process descendants before ancestors so
    // no unprocessed cached pointer can have been deleted. This relies on a
    // unique parent and strictly increasing layer; any future upward rebuild
    // or cross-tree deletion must revisit this ordering.
    std::vector<std::pair<int, OctoTree *>> ordered_updates;
    ordered_updates.reserve(update_cache.size());
    for (OctoTree *voxel : update_cache) ordered_updates.emplace_back(voxel->layer_, voxel);
    std::sort(ordered_updates.begin(), ordered_updates.end(),
              [](const auto &a, const auto &b) { return a.first > b.first; });
    for (const auto &update : ordered_updates) {
      OctoTree *voxel = update.second;
      voxel->update_single_voxel();
      if(voxel->plane_ptr_->is_plane && voxel->root->stop_reinit_ && voxel->plane_points_.size() > config_.max_points_size){
        voxel->update_enable_ = false;
      }
    }

    return;
  }

  void update_single_voxel(){
    if(!init_octo_){
      init_octo_tree();
      return;
    }

    if(new_points_num_ < update_size_threshold_){
      return;
    }

    if(plane_ptr_->is_plane && plane_ptr_->cov_need_update){
      update_plane_cov();
    }

    if(octo_state_ == 1 && !plane_ptr_->is_plane){
      ROS_ERROR("in update : octo_state_ == 1 && !plane_ptr_->is_plane");
    }


    if(octo_state_ == 0 && 1.0 * plane_points_.size() / (plane_points_.size() + not_plane_points_.size()) < config_.ransac_isplane_p_threshold){
      new_points_.splice(new_points_.end(), plane_points_);
      new_points_.splice(new_points_.end(), not_plane_points_);
      init_octo_tree();
    }

    new_points_num_ = 0;
  }

  void build_single_residual(const pointWithCov &pv, bool &is_sucess,
                            double &prob, ptpl &single_ptpl) {
    double radius_k = 3;
    Eigen::Vector3d p_w = pv.point;
    if (plane_ptr_->is_plane && plane_ptr_->normal.allFinite() && plane_ptr_->center.allFinite() &&
        plane_ptr_->plane_cov.allFinite() && std::isfinite(plane_ptr_->d) && std::isfinite(plane_ptr_->radius)) {
      Plane &plane = *plane_ptr_;
      // distance to plane: Ax+By+Cz+D
      float dis_to_plane =
          fabs(plane.normal(0) * p_w(0) + plane.normal(1) * p_w(1) +
              plane.normal(2) * p_w(2) + plane.d);

      float dis_to_center =
          (plane.center(0) - p_w(0)) * (plane.center(0) - p_w(0)) +
          (plane.center(1) - p_w(1)) * (plane.center(1) - p_w(1)) +
          (plane.center(2) - p_w(2)) * (plane.center(2) - p_w(2));

      float range_dis = sqrt(std::max(0.0f, dis_to_center - dis_to_plane * dis_to_plane));

      if (range_dis <= radius_k * plane.radius) {
        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq.block<1, 3>(0, 0) = p_w - plane.center;
        J_nq.block<1, 3>(0, 3) = -plane.normal;
        double sigma_l = J_nq * plane.plane_cov * J_nq.transpose();
        sigma_l += plane.normal.transpose() * pv.cov * plane.normal;
        if (std::isfinite(sigma_l) && sigma_l > 0.0 && dis_to_plane < 3.0 * sqrt(sigma_l)) {
          double this_prob = 1.0 / (sqrt(sigma_l)) *
                            exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
          if (std::isfinite(this_prob) && this_prob > prob) {
            is_sucess = true;
            prob = this_prob;
            single_ptpl.plane_cov = plane.plane_cov;
            single_ptpl.normal = plane.normal;
            single_ptpl.center = plane.center;
            single_ptpl.d = plane.d;
            single_ptpl.layer = layer_;
          }
        }
      }
    }

    if (layer_ < max_layer_ && octo_state_ == 1) {
      for (size_t leafnum = 0; leafnum < 8; leafnum++) {
        if (leaves_[leafnum] != nullptr) {
          leaves_[leafnum]->build_single_residual(pv, is_sucess, prob, single_ptpl);
        }
      }
    }
  }

  // Expose the points still owned by this node and its descendants.
  void append_points(std::vector<pointWithCov> &out) const {
    out.insert(out.end(), plane_points_.begin(), plane_points_.end());
    out.insert(out.end(), not_plane_points_.begin(), not_plane_points_.end());
    out.insert(out.end(), new_points_.begin(), new_points_.end());
    for (const OctoTree *leaf : leaves_) {
      if (leaf != nullptr) leaf->append_points(out);
    }
  }

  std::size_t point_count() const {
    std::size_t count = plane_points_.size() + not_plane_points_.size() + new_points_.size();
    for (const OctoTree *leaf : leaves_) {
      if (leaf != nullptr) count += leaf->point_count();
    }
    return count;
  }

private:
  bool ransac(){
    Eigen::Vector3d best_normal, best_center;
    int max_inliers = 0;
    int sample_num = config_.ransac_sample_num;

    // create iterator vector for random access
    std::vector<std::list<pointWithCov>::iterator> iter_vec;
    iter_vec.reserve(new_points_.size());
    for(auto it = new_points_.begin(); it != new_points_.end(); ++it){
      iter_vec.push_back(it);
    }
    all_points_num_ = iter_vec.size();

    if(sample_num < 3){
      ROS_ERROR("ransac sample num is less than 3, set sample num to 3");
      sample_num = 3;
      config_.ransac_sample_num = sample_num;
    }
    if(sample_num > all_points_num_){
      ROS_WARN("ransac sample num is greater than all points num %d, set sample num to all points num", all_points_num_);
      sample_num = all_points_num_;
    }

    // RANSAC
    for(int i = 0; i < config_.ransac_max_iter; ++i){
      std::vector<Eigen::Vector3d> sample_points(sample_num);
      std::unordered_set<int> sampled_set;
      int cnt = 0;
      while(cnt < sample_num){
        int sample = rand() % all_points_num_;
        if(sampled_set.find(sample) == sampled_set.end()){
          sample_points[cnt] = iter_vec[sample]->point;
          sampled_set.insert(sample);
          cnt++;
        }
      }

      Eigen::Vector3d center = Eigen::Vector3d::Zero();
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      for(int j = 0; j < sample_num; j++){
        center += sample_points[j];
        cov += sample_points[j] * sample_points[j].transpose();
      }
      center = center / sample_num;
      cov = cov / sample_num - center * center.transpose();

      if (!cov.allFinite()) continue;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
      if (!valid_plane_spectrum(es, center)) continue;
      Eigen::Vector3d normal = es.eigenvectors().col(0);
      normal.normalize();

      int inliers = 0;
      for(int k = 0; k < all_points_num_; k++){
        Eigen::Vector3d current_point = iter_vec[k]->point;
        double distance = abs((current_point - center).dot(normal));
        if(distance < config_.ransac_inlier_distance_threshold){
          inliers++;
        }
      }

      if(inliers > max_inliers){
        max_inliers = inliers;
        best_normal = normal;
        best_center = center;
      }
    }

    // check if is plane
    if(1.0f * max_inliers/all_points_num_ > config_.ransac_isplane_p_threshold){
      for(int i = 0; i < all_points_num_; i++){
        auto& iter = iter_vec[i];
        double distance = abs((iter->point - best_center).dot(best_normal));
        if (distance < config_.ransac_inlier_distance_threshold) {
          plane_points_.splice(plane_points_.end(), new_points_, iter);
        } else {
          not_plane_points_.splice(not_plane_points_.end(), new_points_, iter);
        }
      }
      return true;
    }
    else{
      not_plane_points_.splice(not_plane_points_.end(), new_points_);
      return false;
    }
  }

  bool plane_valid_check() {
    Plane *plane = plane_ptr_;

    // project points to plane
    // !note: the valid_check_resolution must not be 0
    double resolution = quater_length_ * 4 / config_.valid_check_resolution;
    Eigen::Vector3d x_axis = plane->x_normal;
    Eigen::Vector3d y_axis = plane->y_normal;
    Eigen::Vector3d origin = plane->center;

    std::unordered_map<std::pair<int, int>, std::vector<std::list<pointWithCov>::iterator>, PairHash> grid_map; // 位置->点云idx

    // iterate all points
    for (auto iter = plane_points_.begin(); iter != plane_points_.end(); iter++) {
      const Eigen::Vector3d& point = iter->point;

      // calculate point projection to plane
      Eigen::Vector3d relative_point = point - origin;
      double x_coord = relative_point.dot(x_axis);
      double y_coord = relative_point.dot(y_axis);

      int grid_x = static_cast<int>(std::floor(x_coord / resolution));
      int grid_y = static_cast<int>(std::floor(y_coord / resolution));

      grid_map[{grid_x, grid_y}].push_back(iter);
    }

    // use DFS to cluster points
    std::unordered_map<std::pair<int, int>, bool, PairHash> visited;
    std::vector<std::vector<std::pair<int, int>>> clusters;

    // DFS
    std::function<void(int, int, std::vector<std::pair<int, int>>&)> dfs = [&](int x, int y, std::vector<std::pair<int, int>>& cluster) {
      auto key = std::make_pair(x, y);
      if (visited[key] || grid_map.find(key) == grid_map.end()) {
          return;
      }

      visited[key] = true;
      cluster.push_back(key);

      // check 4 neighbors
      dfs(x+1, y, cluster);
      dfs(x-1, y, cluster);
      dfs(x, y+1, cluster);
      dfs(x, y-1, cluster);
    };

    for (const auto& grid_pair : grid_map) {
      auto key = grid_pair.first;
      if (!visited[key]) {
        std::vector<std::pair<int, int>> cluster;
        dfs(key.first, key.second, cluster);
        clusters.push_back(cluster);
      }
    }

    // find the largest cluster
    int max_cluster_size = 0;
    int max_cluster_idx = -1;
    for (size_t i = 0; i < clusters.size(); i++) {
      int cluster_size = 0;
      for(auto grid_pair : clusters[i]){
        cluster_size += grid_map[grid_pair].size();
      }
      if (cluster_size > max_cluster_size) {
          max_cluster_size = cluster_size;
          max_cluster_idx = i;
      }
    }

    // check if the largest cluster satisfies the threshold
    if (1.0 * max_cluster_size / all_points_num_ > config_.ransac_isplane_p_threshold) {
      for(int i = 0; i < clusters.size(); i++){
        if(i == max_cluster_idx){
          continue;
        }
        for(auto grid_pair : clusters[i]){
          for(auto iter : grid_map[grid_pair]){
            new_points_.splice(new_points_.end(), plane_points_, iter);
          }
        }
      }
      return true;
    }
    else{
      not_plane_points_.splice(not_plane_points_.end(), plane_points_);
      return false;
    }
  }

  void collect_points(std::list<pointWithCov> &collect_list, int collect_layer){
    if(plane_ptr_->is_plane){
      collect_list.splice(collect_list.end(), plane_points_);
    }

    if(octo_state_ == 0){
      collect_list.splice(collect_list.end(), not_plane_points_);
    }

    if(!init_octo_ && layer_ != collect_layer){
      collect_list.splice(collect_list.end(), new_points_);
    }

    if(octo_state_ == 1){
      for(int i = 0; i < 8; i++){
        if(leaves_[i] != nullptr){
          leaves_[i]->collect_points(collect_list, collect_layer);
        }
      }
    }
    return;
  }

  void check_remain_points(){
    if(!plane_points_.empty() || !not_plane_points_.empty() || !new_points_.empty()){
      std::cout << "============ error =============" << std::endl;
      std::cout << "layer_: " << layer_ << std::endl;
      std::cout << "octo_state_: " << octo_state_ << std::endl;
      std::cout << "plane_ptr_->is_plane: " << plane_ptr_->is_plane << std::endl;
    }
    if(!plane_points_.empty()){
      std::cout << "plane_points_.size(): " << plane_points_.size() << std::endl;
    }
    if(!not_plane_points_.empty()){
      std::cout << "not_plane_points_.size(): " << not_plane_points_.size() << std::endl;
    }
    if(!new_points_.empty()){
      std::cout << "new_points_.size(): " << new_points_.size() << std::endl;
    }

    if(octo_state_ == 1){
      for(int i = 0; i < 8; i++){
        if(leaves_[i] != nullptr){
          leaves_[i]->check_remain_points();
        }
      }
    }
    return;
  }

  OctoTree* push_to_subvoxel(const pointWithCov &pv){
    vector<OctoTree *> subvoxel_candidates = find_subvoxel_candidates(pv);
    double min_residual = 1000000;
    OctoTree *best_voxel = nullptr;
    for (auto curr_voxel : subvoxel_candidates) {
      if(curr_voxel->plane_ptr_->is_plane){
        Plane *plane = curr_voxel->plane_ptr_;
        double residual = abs(plane->normal.dot(pv.point) + plane->d);
        double distance2 = (pv.point - plane->center).squaredNorm();
        double range_dis = sqrt(distance2 - residual * residual);
        if(range_dis < 3 * plane->radius){
          if(residual < 3 * sqrt(plane->min_eigen_value) && residual < min_residual){
            min_residual = residual;
            best_voxel = curr_voxel;
          }
        }
      }
    }

    if(best_voxel != nullptr && min_residual < config_.update_inlier_distance_threshold){
      if(!best_voxel->update_enable_){
        root->all_points_num_root_ -= 1;
        return nullptr;
      }
      if(best_voxel->check_and_update(pv)){
        best_voxel->plane_points_.push_back(pv);
        best_voxel->new_points_num_ ++;
        return best_voxel;
      }
    }

    auto last_voxel = subvoxel_candidates.back();
    if(last_voxel->octo_state_ == 0){
      best_voxel = last_voxel;
      if(best_voxel -> init_octo_){
        best_voxel->not_plane_points_.push_back(pv);
        best_voxel->new_points_num_ ++;
      }
      else{
        best_voxel->new_points_.push_back(pv);
      }
    }
    else{
      int xyz[3] = {0, 0, 0};
      if (pv.point[0] > last_voxel->voxel_center_[0]) {
        xyz[0] = 1;
      }
      if (pv.point[1] > last_voxel->voxel_center_[1]) {
        xyz[1] = 1;
      }
      if (pv.point[2] > last_voxel->voxel_center_[2]) {
        xyz[2] = 1;
      }
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

      last_voxel->leaves_[leafnum] = new OctoTree(last_voxel->layer_ + 1, last_voxel->config_);
      last_voxel->leaves_[leafnum]->root = root;
      last_voxel->leaves_[leafnum]->voxel_center_[0] =
          last_voxel->voxel_center_[0] + (2 * xyz[0] - 1) * last_voxel->quater_length_;
      last_voxel->leaves_[leafnum]->voxel_center_[1] =
          last_voxel->voxel_center_[1] + (2 * xyz[1] - 1) * last_voxel->quater_length_;
      last_voxel->leaves_[leafnum]->voxel_center_[2] =
          last_voxel->voxel_center_[2] + (2 * xyz[2] - 1) * last_voxel->quater_length_;
      last_voxel->leaves_[leafnum]->quater_length_ = last_voxel->quater_length_ / 2;

      last_voxel->leaves_[leafnum]->new_points_.push_back(pv);
      best_voxel = last_voxel->leaves_[leafnum];
    }

    return best_voxel;
  }

  vector<OctoTree *> find_subvoxel_candidates(const pointWithCov &pv) {
    vector<OctoTree *> subvoxel_candidates;
    OctoTree *curr = this;
    subvoxel_candidates.push_back(curr);
    while (curr->octo_state_ != 0) {
      int xyz[3] = {0, 0, 0};
      if (pv.point[0] > curr->voxel_center_[0]) {
        xyz[0] = 1;
      }
      if (pv.point[1] > curr->voxel_center_[1]) {
        xyz[1] = 1;
      }
      if (pv.point[2] > curr->voxel_center_[2]) {
        xyz[2] = 1;
      }
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
      if (curr->leaves_[leafnum] == nullptr) {
        break;
      }
      curr = curr->leaves_[leafnum];
      subvoxel_candidates.push_back(curr);
    }
    return subvoxel_candidates;
  }

  void cout_log(){
    std::cout << "[layer_ : " << layer_ ;
    std::cout << " octo_state_ : " << octo_state_ << " ]:";
    if(plane_ptr_->is_plane){
      std::cout << "is plane : " << plane_points_.size() << " not plane : " << not_plane_points_.size() << std::endl;
    }
    else if(init_octo_){
      std::cout << "not plane : " << not_plane_points_.size() << std::endl;
    }
    else{
      std::cout << "new points : " << new_points_.size() << std::endl;
    }

    if(octo_state_ == 1){
      for(int i = 0; i < 8; i++){
        if(leaves_[i] != nullptr){
          leaves_[i]->cout_log();
        }
      }
    }
  }
};

// R-VoxelMap class
class VoxelMap {
public:
  std::unordered_map<VOXEL_LOC, OctoTree *, VoxelHash> feat_map;
  std::unordered_map<VOXEL_LOC, std::list<VOXEL_LOC>::iterator, VoxelHash> loc2lru;
  std::list<VOXEL_LOC> lru_cache;

  voxel_map_config config;
  float voxel_size;
  int lru_capacity;
  std::vector<int> iter2layer_;

  VoxelMap() {}

  VoxelMap(const voxel_map_config& _config) : config(_config) {}

  void init(const voxel_map_config& _config) {
    config = _config;
    voxel_size = config.voxel_size;
    lru_capacity = config.lru_capacity;
    iter2layer_ = config.iter2layer_;
  }

  ~VoxelMap() {
    for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter) {
      delete iter->second;
    }
  }

  void build(const std::vector<pointWithCov> &input_points) {
    uint plsize = input_points.size();
    for (uint i = 0; i < plsize; i++) {
      const pointWithCov p_v = input_points[i];
      VOXEL_LOC position;
      if (!p_v.cov.allFinite() || !voxel_location(p_v.point, voxel_size, position)) continue;
      auto iter = feat_map.find(position);
      if (iter != feat_map.end()) {
        iter->second->new_points_.push_back(p_v);
        iter->second->all_points_num_root_ ++;
      } else {
        OctoTree *octo_tree = new OctoTree(0, config);
        feat_map[position] = octo_tree;
        feat_map[position]->root = octo_tree;
        feat_map[position]->quater_length_ = voxel_size / 4;
        feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
        feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
        feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
        feat_map[position]->new_points_.push_back(p_v);
        feat_map[position]->all_points_num_root_ ++;
        feat_map[position]->layer_point_size_ = config.layer_point_size;
      }
    }
    for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter) {
      iter->second->init_octo_tree();
      if(iter->second->all_points_num_root_ > config.max_root_points_size){
        iter->second->root_update_enable_ = false;
      }
      lru_cache.push_front(iter->first);
      loc2lru[iter->first] = lru_cache.begin();
    }
  }

  void update(const std::vector<pointWithCov> &input_points) {
    unordered_map<VOXEL_LOC, OctoTree *, VoxelHash> update_cache;
    uint plsize = input_points.size();
    for (uint i = 0; i < plsize; i++) {
      const pointWithCov p_v = input_points[i];
      VOXEL_LOC position;
      if (!p_v.cov.allFinite() || !voxel_location(p_v.point, voxel_size, position)) continue;
      auto iter = feat_map.find(position);
      if (iter != feat_map.end()) {
        if(iter->second->root_update_enable_){
          iter->second->new_points_.push_back(p_v);
          iter->second->all_points_num_root_ ++;
        }
      } else {
        OctoTree *octo_tree = new OctoTree(0, config);
        feat_map[position] = octo_tree;
        feat_map[position]->root = octo_tree;
        feat_map[position]->quater_length_ = voxel_size / 4;
        feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
        feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
        feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
        feat_map[position]->new_points_.push_back(p_v);
        feat_map[position]->all_points_num_root_ ++;
      }
      update_cache[position] = feat_map[position];
    }

    for (auto& pair: update_cache) {
      pair.second->update_octo_tree();

      if(pair.second->all_points_num_root_ > config.max_root_points_size){
        pair.second->root_update_enable_ = false;
      }
      // update lru cache
      if(loc2lru.find(pair.first) != loc2lru.end()){
        lru_cache.splice(lru_cache.begin(), lru_cache, loc2lru[pair.first]);
        loc2lru[pair.first] = lru_cache.begin();
      }
      else{
        lru_cache.push_front(pair.first);
        loc2lru[pair.first] = lru_cache.begin();
      }
    }
  }

  void lru_cache_update() {
    while (lru_cache.size() > lru_capacity) {
      auto last = lru_cache.back();
      lru_cache.pop_back();

      loc2lru.erase(last);
      delete feat_map[last];
      feat_map.erase(last);
    }
  }

  // Matching is serial: the author's vector<bool> parallel writes race on
  // shared packed words. useful_index maps each match to its original query.
  void build_residual_omp(const std::vector<pointWithCov> &pv_list,
                          const int /*layer*/, std::vector<ptpl> &ptpl_list,
                          std::vector<int> &useful_index,
                          std::vector<Eigen::Vector3d> &non_match) {
    ptpl_list.clear();
    useful_index.clear();
    non_match.clear();
    for (std::size_t i = 0; i < pv_list.size(); ++i) {
      const pointWithCov &pv = pv_list[i];
      VOXEL_LOC position;
      if (!pv.cov.allFinite() || !voxel_location(pv.point, voxel_size, position)) {
        non_match.push_back(pv.point);
        continue;
      }
      const auto iter = feat_map.find(position);
      bool success = false;
      ptpl match;
      double probability = 0.0;
      if (iter != feat_map.end()) {
        OctoTree *node = iter->second;
        node->build_single_residual(pv, success, probability, match);
        if (!success) {
          VOXEL_LOC neighbor = position;
          int64_t *indices[3] = {&neighbor.x, &neighbor.y, &neighbor.z};
          bool in_range = true;
          for (int axis = 0; axis < 3; ++axis) {
            int step = 0;
            if (pv.point[axis] > node->voxel_center_[axis] + node->quater_length_) step = 1;
            else if (pv.point[axis] < node->voxel_center_[axis] - node->quater_length_) step = -1;
            if ((step > 0 && *indices[axis] == std::numeric_limits<int64_t>::max()) ||
                (step < 0 && *indices[axis] == std::numeric_limits<int64_t>::min())) {
              in_range = false;
              break;
            }
            *indices[axis] += step;
          }
          if (in_range) {
            const auto next = feat_map.find(neighbor);
            if (next != feat_map.end()) next->second->build_single_residual(pv, success, probability, match);
          }
        }
      }
      if (success) {
        ptpl_list.push_back(match);
        useful_index.push_back(static_cast<int>(i));
      } else {
        non_match.push_back(pv.point);
      }
    }
  }


};
}  // namespace r_voxelmap_ns
#endif
