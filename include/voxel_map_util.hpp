#ifndef VOXEL_MAP_UTIL_HPP
#define VOXEL_MAP_UTIL_HPP
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "common_lib.h"
#include "omp.h"
// #include <execution>
#include <openssl/md5.h>
#include <pcl/common/io.h>
#include <rosbag/bag.h>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <unordered_map>

#ifndef HASH_P
#define HASH_P 116101
#endif
#ifndef MAX_N
#define MAX_N 10000000000
#endif

namespace voxel_map_ns
{
    static int plane_id = 0;

    // a point to plane matching structure
    typedef struct ptpl
    {
        Eigen::Vector3d point;
        Eigen::Vector3d point_world;
        Eigen::Vector3d normal;
        Eigen::Vector3d center;
        Eigen::Matrix<double, 6, 6> plane_cov;
        double d;
        int layer;
        Eigen::Matrix3d cov_lidar;
        Eigen::Matrix3d cov_world;
    } ptpl;

    // 3D point with covariance
    typedef struct pointWithCov
    {
        Eigen::Vector3d point_lidar;
        Eigen::Vector3d point_world;
        Eigen::Matrix3d cov_lidar;
        Eigen::Matrix3d cov_world;
    } pointWithCov;

    typedef struct Plane
    {
        Eigen::Vector3d center;
        Eigen::Vector3d normal;
        Eigen::Vector3d y_normal;
        Eigen::Vector3d x_normal;
        Eigen::Matrix3d covariance;  //< 点集的协方差阵
        Eigen::Matrix<double, 6, 6> plane_cov;
        float radius          = 0;  // 平面方向上的半径
        float min_eigen_value = 1;
        float mid_eigen_value = 1;
        float max_eigen_value = 1;
        float d               = 0;
        int points_size       = 0;

        bool is_plane = false;
        bool is_init  = false;
        int id;
        // is_update and last_update_points_size are only for publish plane
        bool is_update              = false;
        int last_update_points_size = 0;
        bool update_enable          = true;
    } Plane;

    class VOXEL_LOC
    {
    public:
        int64_t x, y, z;

        VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
            : x(vx), y(vy), z(vz) {}

        bool operator==(const VOXEL_LOC &other) const
        {
            return (x == other.x && y == other.y && z == other.z);
        }
    };
}

// Hash value
namespace std
{
    template <>
    struct hash<voxel_map_ns::VOXEL_LOC>
    {
        int64_t operator()(const voxel_map_ns::VOXEL_LOC &s) const
        {
            using std::hash;
            using std::size_t;
            return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
        }
    };
}  // namespace std

namespace voxel_map_ns
{
    class OctoTree
    {
    public:
        std::vector<pointWithCov> temp_points_;  // all points in an octo tree
        std::vector<pointWithCov> new_points_;   // new points in an octo tree
        Plane *plane_ptr_;
        int max_layer_;  // 4
        bool indoor_mode_;
        int layer_;
        int octo_state_;  // 0 is end of tree, 1 is not
        OctoTree *leaves_[8];
        double voxel_center_[3];  // x, y, z
        std::vector<int> layer_point_size_;
        float quater_length_;
        int max_plane_update_threshold_;
        int update_size_threshold_;
        int all_points_num_;
        int new_points_num_;
        int max_points_size_;
        int max_cov_points_size_;
        float planer_threshold_;
        bool init_octo_;
        bool update_cov_enable_;  // true: enough points, no need to update after
        bool update_enable_;      // true: enough points, no need to update after
        OctoTree(int max_layer, int layer, std::vector<int> layer_point_size,
                 int max_point_size, int max_cov_points_size, float planer_threshold)
            : max_layer_(max_layer), layer_(layer), layer_point_size_(layer_point_size), max_points_size_(max_point_size), max_cov_points_size_(max_cov_points_size), planer_threshold_(planer_threshold)
        {
            temp_points_.clear();
            octo_state_     = 0;
            new_points_num_ = 0;
            all_points_num_ = 0;
            // when new points num > 5, do a update
            update_size_threshold_      = 5;
            init_octo_                  = false;
            update_enable_              = true;
            update_cov_enable_          = true;
            max_plane_update_threshold_ = layer_point_size_[layer_];
            for (int i = 0; i < 8; i++)
            {
                leaves_[i] = nullptr;
            }
            plane_ptr_ = new Plane;
        }

        /**
         * @brief check is plane , calc plane parameters including plane covariance
         *        ? 要求至少五个点
         * @param points
         * @param plane
         */
        void init_plane(const std::vector<pointWithCov> &points, Plane *plane)
        {
            //! 点集的均值和协方差，与平面方程无关
            //? 但是为什么计算时不考虑各点的协方差阵
            plane->plane_cov   = Eigen::Matrix<double, 6, 6>::Zero();
            plane->covariance  = Eigen::Matrix3d::Zero();
            plane->center      = Eigen::Vector3d::Zero();
            plane->normal      = Eigen::Vector3d::Zero();
            plane->points_size = points.size();
            plane->radius      = 0;
            for (auto pv : points)
            {
                // note: modified 修改 voxelmap 变量调用混乱的情况
                plane->covariance += pv.point_world * pv.point_world.transpose();
                plane->center += pv.point_world;
                //? 以下为考虑点协方差阵的公式
                // plane->center += pv.point * pv.cov.inverse();
                // plane->covariance += pv.cov.inverse();
            }
            plane->center /= plane->points_size;
            plane->covariance /= plane->points_size;
            plane->covariance -= plane->center * plane->center.transpose();
            //? 以下为考虑点协方差阵的公式
            // plane->convariance = plane->convariance.inverse();
            // plane->center *= plane->convariance;

            //* 点集协方差阵特征值分解
            Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance);
            Eigen::Matrix3cd evecs    = es.eigenvectors();
            Eigen::Vector3cd evals    = es.eigenvalues();
            Eigen::Vector3d evalsReal = evals.real();
            Eigen::Matrix3f::Index evalsMin, evalsMax;
            evalsReal.minCoeff(&evalsMin);
            evalsReal.maxCoeff(&evalsMax);
            int evalsMid = 3 - evalsMin - evalsMax;
            // Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
            // Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
            // Eigen::Vector3d evecMax = evecs.real().col(evalsMax);

            //* -------------------------------------------------------------------------- */
            //! modified: 从下面 if-else 中提取的公共部分
            // note: 这里只是表示平面参数被首次计算，不代表这是一个平面
            if (!plane->is_init)
            {
                plane->id = plane_id;  //< 索引
                plane_id++;
                plane->is_init = true;
            }

            // points_size = points->size()
            //? 足够多的点意味着平面参数的变化是一个明显的 update？
            if (plane->last_update_points_size == 0 || plane->points_size - plane->last_update_points_size > 100)
            {
                plane->last_update_points_size = plane->points_size;
                plane->is_update               = true;
            }

            //* 记录三个法向量及其对应特征值，按特征值大小（与方差大小成正比）排序，特征值最小的是平面的法向
            plane->normal          = evecs.real().col(evalsMin);
            plane->y_normal        = evecs.real().col(evalsMid);
            plane->x_normal        = evecs.real().col(evalsMax);
            plane->min_eigen_value = evalsReal(evalsMin);
            plane->mid_eigen_value = evalsReal(evalsMid);
            plane->max_eigen_value = evalsReal(evalsMax);
            plane->radius          = sqrt(evalsReal(evalsMax));            //< 方差最大的方向，标准差表征平面的水平范围
            plane->d               = -(plane->normal.dot(plane->center));  //< 平面方程的常数项
            //* -------------------------------------------------------------------------- */

            //* 若方差足够小，表示这是一个平面，综合各点的球坐标系协方差，精确计算平面协方差
            // note: 由于采用点法式（中心点+法向量确定平面），因此采用 6*6 的协方差阵
            // note: 左上右下分别为法向和平面位置不确定性，右上左下为法向与位置相关不确定性
            // note: 实际三维空间中，一个平面的协方差阵最小可采用 3*3，因为自由度为 3
            if (evalsReal(evalsMin) < planer_threshold_)
            {
                Eigen::Matrix3d J_Q;
                J_Q << 1.0 / plane->points_size, 0, 0, 0,
                    1.0 / plane->points_size, 0, 0, 0,
                    1.0 / plane->points_size;

                for (size_t i = 0; i < points.size(); i++)
                {
                    Eigen::Matrix<double, 6, 3> J;
                    Eigen::Matrix3d F;
                    for (int m = 0; m < 3; m++)
                    {
                        if (m != (int)evalsMin)
                        {
                            // note: modified 修改 voxelmap 变量调用混乱的情况
                            F.row(m) = (points[i].point_world - plane->center).transpose()
                                       / ((plane->points_size) * (evalsReal[evalsMin] - evalsReal[m]))
                                       * (evecs.real().col(m) * evecs.real().col(evalsMin).transpose()
                                          + evecs.real().col(evalsMin) * evecs.real().col(m).transpose());  //? 不懂
                        }
                        else
                        {
                            F.row(m) << 0, 0, 0;
                        }
                    }
                    J.block<3, 3>(0, 0) = evecs.real() * F;
                    J.block<3, 3>(3, 0) = J_Q;
                    plane->plane_cov += J * points[i].cov_world * J.transpose();
                }

                plane->is_plane = true;  //! 确定是平面
            }
            else
            {
                plane->is_plane = false;
            }
        }

        // only updaye plane normal, center and radius with new points
        void update_plane(const std::vector<pointWithCov> &points, Plane *plane)
        {
            // Eigen::Matrix3d old_covariance = plane->covariance;
            // Eigen::Vector3d old_center = plane->center;

            Eigen::Matrix3d sum_ppt = (plane->covariance + plane->center * plane->center.transpose())
                                      * plane->points_size;
            Eigen::Vector3d sum_p = plane->center * plane->points_size;
            for (auto pv : points)
            {
                // note: modified 修改 voxelmap 变量调用混乱的情况
                plane->covariance += pv.point_world * pv.point_world.transpose();
                plane->center += pv.point_world;
            }
            plane->points_size = plane->points_size + points.size();
            plane->center      = sum_p / plane->points_size;
            plane->covariance  = sum_ppt / plane->points_size - plane->center * plane->center.transpose();

            Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance);
            Eigen::Matrix3cd evecs    = es.eigenvectors();
            Eigen::Vector3cd evals    = es.eigenvalues();
            Eigen::Vector3d evalsReal = evals.real();
            Eigen::Matrix3d::Index evalsMin, evalsMax;
            evalsReal.minCoeff(&evalsMin);
            evalsReal.maxCoeff(&evalsMax);
            int evalsMid = 3 - evalsMin - evalsMax;
            // Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
            // Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
            // Eigen::Vector3d evecMax = evecs.real().col(evalsMax);

            //* -------------------------------------------------------------------------- */
            //! modified: 从下面的 if-else 提取公共部分
            plane->normal          = evecs.real().col(evalsMin);
            plane->y_normal        = evecs.real().col(evalsMid);
            plane->x_normal        = evecs.real().col(evalsMax);
            plane->min_eigen_value = evalsReal(evalsMin);
            plane->mid_eigen_value = evalsReal(evalsMid);
            plane->max_eigen_value = evalsReal(evalsMax);
            plane->radius          = sqrt(evalsReal(evalsMax));
            plane->d               = -(plane->normal.dot(plane->center));  //< 平面方程的常数项
            plane->is_update       = true;
            //* -------------------------------------------------------------------------- */

            if (evalsReal(evalsMin) < planer_threshold_)
            {
                plane->is_plane = true;
            }
            else
            {
                plane->is_plane = false;
            }
        }

        /**
         * @brief
         */
        void init_octo_tree()
        {
            if (temp_points_.size() > (size_t)max_plane_update_threshold_)  //< 5
            {
                // 1.计算平面
                init_plane(temp_points_, plane_ptr_);

                // 2.满足平面要求
                if (plane_ptr_->is_plane == true)
                {
                    octo_state_ = 0;
                    if (temp_points_.size() > (size_t)max_cov_points_size_)  //< 1000
                    {
                        update_cov_enable_ = false;
                    }
                    if (temp_points_.size() > (size_t)max_points_size_)  //< 1000
                    {
                        update_enable_ = false;
                    }
                }
                // 3. 不满足平面要求，划分子树继续尝试构建平面
                else
                {
                    octo_state_ = 1;  // not end of leave
                    cut_octo_tree();
                }

                init_octo_      = true;
                new_points_num_ = 0;
            }
        }

        /**
         * @brief
         */
        void cut_octo_tree()
        {
            // 层数限制
            if (layer_ >= max_layer_)
            {
                octo_state_ = 0;
                return;
            }

            //* 1. 将各点分配到新的八叉树叶子节点中
            for (size_t i = 0; i < temp_points_.size(); i++)
            {
                // 根据与中心点三轴坐标的大小关系计算该点在八叉树划分时的位置
                // note: modified 修改 voxelmap 变量调用混乱的情况
                int xyz[3] = {0, 0, 0};
                if (temp_points_[i].point_world[0] > voxel_center_[0])
                {
                    xyz[0] = 1;
                }
                if (temp_points_[i].point_world[1] > voxel_center_[1])
                {
                    xyz[1] = 1;
                }
                if (temp_points_[i].point_world[2] > voxel_center_[2])
                {
                    xyz[2] = 1;
                }
                int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

                // 将该点记录到对应的新叶子节点
                if (leaves_[leafnum] == nullptr)
                {
                    leaves_[leafnum] = new OctoTree(max_layer_, layer_ + 1, layer_point_size_, max_points_size_,
                                                    max_cov_points_size_, planer_threshold_);

                    leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
                    leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
                    leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
                    leaves_[leafnum]->quater_length_   = quater_length_ / 2;
                }
                leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
                leaves_[leafnum]->new_points_num_++;
            }

            //* 2. leaves_[i]->init_octo_plane()
            for (uint i = 0; i < 8; i++)
            {
                if (leaves_[i] != nullptr)
                {
                    if (leaves_[i]->temp_points_.size() > (size_t)leaves_[i]->max_plane_update_threshold_)  //< 5
                    {
                        init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);

                        if (leaves_[i]->plane_ptr_->is_plane)
                        {
                            leaves_[i]->octo_state_ = 0;
                        }
                        else
                        {
                            leaves_[i]->octo_state_ = 1;
                            leaves_[i]->cut_octo_tree();
                        }
                        leaves_[i]->init_octo_      = true;
                        leaves_[i]->new_points_num_ = 0;
                    }
                }
            }
        }

        /**
         * @brief voxel_map_utils.cpp的内部函数，两处调用：UpdateVoxelMap()、函数内递归
         * @param pv
         */
        void UpdateOctoTree(const pointWithCov &pv)
        {
            if (!init_octo_)
            {
                new_points_num_++;
                all_points_num_++;
                temp_points_.push_back(pv);
                if (temp_points_.size() > (size_t)max_plane_update_threshold_)
                {
                    init_octo_tree();
                }
            }
            else
            {
                if (plane_ptr_->is_plane)
                {
                    if (update_enable_)
                    {
                        new_points_num_++;
                        all_points_num_++;
                        if (update_cov_enable_)
                        {
                            temp_points_.push_back(pv);
                        }
                        else
                        {
                            new_points_.push_back(pv);
                        }
                        if (new_points_num_ > update_size_threshold_)
                        {
                            if (update_cov_enable_)
                            {
                                init_plane(temp_points_, plane_ptr_);
                            }
                            new_points_num_ = 0;
                        }
                        if (all_points_num_ >= max_cov_points_size_)
                        {
                            update_cov_enable_ = false;
                            std::vector<pointWithCov>().swap(temp_points_);
                        }
                        if (all_points_num_ >= max_points_size_)
                        {
                            update_enable_            = false;
                            plane_ptr_->update_enable = false;
                            std::vector<pointWithCov>().swap(new_points_);
                        }
                    }
                    else
                    {
                        return;
                    }
                }
                else
                {
                    if (layer_ < max_layer_)
                    {
                        if (temp_points_.size() != 0)
                        {
                            std::vector<pointWithCov>().swap(temp_points_);
                        }
                        if (new_points_.size() != 0)
                        {
                            std::vector<pointWithCov>().swap(new_points_);
                        }

                        // note: modified 修改 voxelmap 变量调用混乱的情况
                        int xyz[3] = {0, 0, 0};
                        if (pv.point_world[0] > voxel_center_[0])
                        {
                            xyz[0] = 1;
                        }
                        if (pv.point_world[1] > voxel_center_[1])
                        {
                            xyz[1] = 1;
                        }
                        if (pv.point_world[2] > voxel_center_[2])
                        {
                            xyz[2] = 1;
                        }

                        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
                        if (leaves_[leafnum] != nullptr)
                        {
                            leaves_[leafnum]->UpdateOctoTree(pv);
                        }
                        else
                        {
                            leaves_[leafnum]                    = new OctoTree(max_layer_, layer_ + 1, layer_point_size_, max_points_size_,
                                                                               max_cov_points_size_, planer_threshold_);
                            leaves_[leafnum]->layer_point_size_ = layer_point_size_;
                            leaves_[leafnum]->voxel_center_[0]  = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
                            leaves_[leafnum]->voxel_center_[1]  = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
                            leaves_[leafnum]->voxel_center_[2]  = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
                            leaves_[leafnum]->quater_length_    = quater_length_ / 2;
                            leaves_[leafnum]->UpdateOctoTree(pv);
                        }
                    }
                    else
                    {
                        if (update_enable_)
                        {
                            new_points_num_++;
                            all_points_num_++;
                            if (update_cov_enable_)
                            {
                                temp_points_.push_back(pv);
                            }
                            else
                            {
                                new_points_.push_back(pv);
                            }
                            if (new_points_num_ > update_size_threshold_)
                            {
                                if (update_cov_enable_)
                                {
                                    init_plane(temp_points_, plane_ptr_);
                                }
                                else
                                {
                                    update_plane(new_points_, plane_ptr_);
                                    new_points_.clear();
                                }
                                new_points_num_ = 0;
                            }
                            if (all_points_num_ >= max_cov_points_size_)
                            {
                                update_cov_enable_ = false;
                                std::vector<pointWithCov>().swap(temp_points_);
                            }
                            if (all_points_num_ >= max_points_size_)
                            {
                                update_enable_            = false;
                                plane_ptr_->update_enable = false;
                                std::vector<pointWithCov>().swap(new_points_);
                            }
                        }
                    }
                }
            }
        }
    };

    void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g,
                uint8_t &b)
    {
        r = 255;
        g = 255;
        b = 255;

        if (v < vmin)
        {
            v = vmin;
        }

        if (v > vmax)
        {
            v = vmax;
        }

        double dr, dg, db;

        if (v < 0.1242)
        {
            db = 0.504 + ((1. - 0.504) / 0.1242) * v;
            dg = dr = 0.;
        }
        else if (v < 0.3747)
        {
            db = 1.;
            dr = 0.;
            dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
        }
        else if (v < 0.6253)
        {
            db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
            dg = 1.;
            dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
        }
        else if (v < 0.8758)
        {
            db = 0.;
            dr = 1.;
            dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
        }
        else
        {
            db = 0.;
            dg = 0.;
            dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
        }

        r = (uint8_t)(255 * dr);
        g = (uint8_t)(255 * dg);
        b = (uint8_t)(255 * db);
    }

    /**
     * @brief 构建 voxelmap，流程与 STD 中 init_voxel_map() 基本一致
     * @param input_points
     * @param voxel_size // 0.25
     * @param max_layer
     * @param layer_point_size
     * @param max_points_size
     * @param max_cov_points_size
     * @param planer_threshold
     * @param feat_map //! 待构建的 voxelmap 对象
     */
    void buildVoxelMap(const std::vector<pointWithCov> &input_points,
                       const float voxel_size, const int max_layer,
                       const std::vector<int> &layer_point_size,
                       const int max_points_size, const int max_cov_points_size,
                       const float planer_threshold,
                       std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map)
    {
        //* 1. 将点分配到各体素位置对应的八叉树对象中
        for (uint i = 0; i < input_points.size(); i++)
        {
            // 计算体素位置
            // const pointWithCov p_v = input_points[i];
            const pointWithCov &p_v = input_points[i];
            float loc_xyz[3];
            for (int j = 0; j < 3; j++)
            {
                // note: modified 这是 voxel_map 本身的变量使用混乱，
                // note: 但是由于 voxelMapping.cpp 中调用时仔细审查，未对程序造成影响，这里修改回来
                loc_xyz[j] = p_v.point_world[j] / voxel_size;
                if (loc_xyz[j] < 0)
                {
                    loc_xyz[j] -= 1.0;
                }
            }
            VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

            // 将点记录到对应体素位置的八叉树中
            auto iter = feat_map.find(position);
            if (iter != feat_map.end())
            {
                feat_map[position]->temp_points_.push_back(p_v);
                feat_map[position]->new_points_num_++;
            }
            else
            {
                OctoTree *octo_tree                  = new OctoTree(max_layer, 0, layer_point_size, max_points_size,
                                                                    max_cov_points_size, planer_threshold);
                feat_map[position]                   = octo_tree;
                feat_map[position]->quater_length_   = voxel_size / 4;  //< 0.25 / 4，记录一个 1/4 长度，方便后续运算
                feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
                feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
                feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
                feat_map[position]->temp_points_.push_back(p_v);
                feat_map[position]->new_points_num_++;
                feat_map[position]->layer_point_size_ = layer_point_size;  //? 重复
            }
        }

        //* 2. 初始化各八叉树对象，主要是计算平面参数
        for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter)
        {
            iter->second->init_octo_tree();
        }
    }

    void updateVoxelMap(const std::vector<pointWithCov> &input_points,
                        const float voxel_size, const int max_layer,
                        const std::vector<int> &layer_point_size,
                        const int max_points_size, const int max_cov_points_size,
                        const float planer_threshold,
                        std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map)
    {
        uint plsize = input_points.size();
        for (uint i = 0; i < plsize; i++)
        {
            // 计算体素位置
            // const pointWithCov p_v = input_points[i];
            const pointWithCov &p_v = input_points[i];
            float loc_xyz[3];
            for (int j = 0; j < 3; j++)
            {
                // note: modified 这是 voxel_map 本身的变量使用混乱，
                // note: 但是由于 voxelMapping.cpp 中调用时仔细审查，未对程序造成影响，这里修改回来
                loc_xyz[j] = p_v.point_world[j] / voxel_size;
                if (loc_xyz[j] < 0)
                {
                    loc_xyz[j] -= 1.0;
                }
            }
            VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
            auto iter = feat_map.find(position);
            // 如果点的位置已经存在voxel 那么就更新点的位置 否则创建新的voxel
            if (iter != feat_map.end())
            {
                feat_map[position]->UpdateOctoTree(p_v);
            }
            else
            {
                OctoTree *octo_tree                  = new OctoTree(max_layer, 0, layer_point_size, max_points_size,
                                                                    max_cov_points_size, planer_threshold);
                feat_map[position]                   = octo_tree;
                feat_map[position]->quater_length_   = voxel_size / 4;
                feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
                feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
                feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
                feat_map[position]->UpdateOctoTree(p_v);
            }
        }
    }

    void updateVoxelMapOMP(const std::vector<pointWithCov> &input_points,
                           const float voxel_size, const int max_layer,
                           const std::vector<int> &layer_point_size,
                           const int max_points_size, const int max_cov_points_size,
                           const float planer_threshold,
                           std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map)
    {
        std::unordered_map<VOXEL_LOC, vector<pointWithCov>> position_index_map;
        int insert_count = 0, update_count = 0;
        uint plsize = input_points.size();

        // double t_update_start = omp_get_wtime();
        for (uint i = 0; i < plsize; i++)
        {
            // 计算体素位置
            const pointWithCov &p_v = input_points[i];
            float loc_xyz[3];
            for (int j = 0; j < 3; j++)
            {
                // note: modified 这是 voxel_map 本身的变量使用混乱，
                // note: 但是由于 voxelMapping.cpp 中调用时仔细审查，未对程序造成影响，这里修改回来
                loc_xyz[j] = p_v.point_world[j] / voxel_size;
                if (loc_xyz[j] < 0)
                {
                    loc_xyz[j] -= 1.0;
                }
            }
            VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
            auto iter = feat_map.find(position);
            // 如果点的位置已经存在voxel 那么就更新点的位置 否则创建新的voxel
            if (iter != feat_map.end())
            {
                // 更新的点总是很多 先缓存 再延迟并行更新
                update_count++;
                position_index_map[position].push_back(p_v);
            }
            else
            {
                // 插入的点总是少的 直接单线程插入
                // 保存position位置对应的点
                insert_count++;
                OctoTree *octo_tree                  = new OctoTree(max_layer, 0, layer_point_size, max_points_size,
                                                                    max_cov_points_size, planer_threshold);
                feat_map[position]                   = octo_tree;
                feat_map[position]->quater_length_   = voxel_size / 4;
                feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
                feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
                feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
                feat_map[position]->UpdateOctoTree(p_v);
            }
        }
        // double t_update_end = omp_get_wtime();
        // std::printf("Insert & store time:  %.4fs\n", t_update_end - t_update_start);
        // t_update_start = omp_get_wtime();
        // 并行延迟更新
#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for default(none) shared(position_index_map, feat_map)
#endif
        for (size_t b = 0; b < position_index_map.bucket_count(); b++)
        {
            // 先遍历bucket 理想情况下bucket一般只有一个元素 这样还是相当于完全并行的遍历position_index_map
            // XXX 需要确定最坏情况下bucket的元素数量
            for (auto bi = position_index_map.begin(b); bi != position_index_map.end(b); bi++)
            {
                VOXEL_LOC position = bi->first;
                for (const pointWithCov &p_v : bi->second)
                {
                    feat_map[position]->UpdateOctoTree(p_v);
                }
            }
        }
        // t_update_end = omp_get_wtime();

        // std::printf("Update:  %.4fs\n", t_update_end - t_update_start);
        // std::printf("Insert: %d  Update: %d \n", insert_count, update_count);
    }

    /**
     * @brief
     * @param pv
     * @param current_octo
     * @param current_layer
     * @param max_layer
     * @param sigma_num // 3.0
     * @param is_sucess
     * @param prob
     * @param single_ptpl
     */
    void build_single_residual(const pointWithCov &pv, const OctoTree *current_octo,
                               const int current_layer, const int max_layer,
                               const double sigma_num, bool &is_sucess,
                               double &prob, ptpl &single_ptpl)
    {
        const double radius_k      = 3;
        const Eigen::Vector3d &p_w = pv.point_world;  // note: modified

        //* 如果当前voxel有平面，则构建 voxel block
        //! 否则，在递归搜索当前 voxel 的 leaves 直到找到平面，即递归调用此函数
        // XXX 如果不是平面是不是可以在构建的时候直接剪掉？
        if (current_octo->plane_ptr_->is_plane)
        {
            Plane &plane = *current_octo->plane_ptr_;

            float dis_to_plane  = fabs(p_w.dot(plane.normal) + plane.d);  //< 点到该平面的距离
            float dis_to_center = (p_w - plane.center).squaredNorm();     //< 点到该平面点集中心的距离

            //* range_dis 越大，说明该点虽然靠近平面，但是离估计该平面的点集的中心很远
            //* 在这些远点处可能不满足该平面的假设，因此不能使用这个点与平面计算残差
            // 此处筛选是因为将点划分进voxel时只用了第一层voxel
            // 这个voxel可能比较大，遍历到的这个子voxel时，距离点可能还比较远
            //? modified 不清除原因，调试发现存在 dis_to_center < dis_to_plane * dis_to_plane，这里添加条件判断
            float range_dis = dis_to_center - dis_to_plane * dis_to_plane;
            if (range_dis < 0)
            {
                return;
            }
            range_dis = sqrt(range_dis);  //< 沿平面方向与中心点的距离，勾股定理

            if (range_dis <= radius_k * plane.radius)
            {
                //* 点面距离的方差
                // 这里采用点法式模型 dis = nT * (p - p0) 进行协方差传播推导，
                // 同时点 pv 与该平面相互独立，即 cov(p, p0) = 0，cov(p, n) = 0
                Eigen::Matrix<double, 1, 6> J_nq;
                J_nq.block<1, 3>(0, 0) = p_w - plane.center;
                J_nq.block<1, 3>(0, 3) = -plane.normal;
                double sigma_l         = J_nq * plane.plane_cov * J_nq.transpose();
                sigma_l += plane.normal.transpose() * pv.cov_world * plane.normal;

                //* 只选择与平面的距离在 3倍 sigma 之内的匹配
                if (dis_to_plane < sigma_num * sqrt(sigma_l))
                {
                    // is_sucess = true; //? 这一行修改到下面的 if 语句执行部分，否则逻辑上会返回 false positive

                    //* 求对应正态分布的概率密度值，即该点属于当前平面的可能性，注意分布的均值为0
                    // 如果进入递归过程，这里记录最大概率的 ptpl 匹配，但如果第一个节点就有平面，就不会递归，可能返回空
                    //? 但是少了 1. / sqrt(2 * M_PI)，因为常值所以无所谓？
                    // HACK 这里比fast lio和任何loam系的都要clever得多
                    double this_prob = 1.0 / (sqrt(sigma_l)) * exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
                    if (this_prob > prob)
                    {
                        is_sucess               = true;  // note: modified
                        prob                    = this_prob;
                        single_ptpl.point       = pv.point_lidar;
                        single_ptpl.point_world = pv.point_world;
                        single_ptpl.plane_cov   = plane.plane_cov;
                        single_ptpl.normal      = plane.normal;
                        single_ptpl.center      = plane.center;
                        single_ptpl.d           = plane.d;
                        single_ptpl.layer       = current_layer;
                        single_ptpl.cov_lidar   = pv.cov_lidar;
                        single_ptpl.cov_world   = pv.cov_world;
                    }
                }
            }
        }
        else
        {
            //* 如果当前 voxel 没有平面，且层数未达上限，继续往下递归
            if (current_layer < max_layer)
            {
                //* 遍历当前节点的所有叶子
                for (size_t leafnum = 0; leafnum < 8; leafnum++)
                {
                    if (current_octo->leaves_[leafnum] != nullptr)
                    {
                        build_single_residual(pv, current_octo->leaves_[leafnum], current_layer + 1, max_layer,
                                              sigma_num, is_sucess, prob, single_ptpl);
                    }
                }
            }
        }

        return;
    }

    void GetUpdatePlane(const OctoTree *current_octo, const int pub_max_voxel_layer,
                        std::vector<Plane> &plane_list)
    {
        if (current_octo->layer_ > pub_max_voxel_layer)
        {
            return;
        }
        if (current_octo->plane_ptr_->is_update)
        {
            plane_list.push_back(*current_octo->plane_ptr_);
        }
        if (current_octo->layer_ < current_octo->max_layer_)
        {
            if (!current_octo->plane_ptr_->is_plane)
            {
                for (size_t i = 0; i < 8; i++)
                {
                    if (current_octo->leaves_[i] != nullptr)
                    {
                        GetUpdatePlane(current_octo->leaves_[i], pub_max_voxel_layer,
                                       plane_list);
                    }
                }
            }
        }
        return;
    }

    /**
     * @brief 查找最近点并计算残差
     * @param voxel_map
     * @param voxel_size
     * @param sigma_num // 3.0
     * @param max_layer // 4
     * @param pv_list
     * @param ptpl_list
     * @param non_match
     */
    void BuildResidualListOMP(const unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                              const double voxel_size, const double sigma_num,
                              const int max_layer,
                              const std::vector<pointWithCov> &pv_list,
                              std::vector<ptpl> &ptpl_list,
                              std::vector<Eigen::Vector3d> &non_match)
    {
        // init
        ptpl_list.clear();
        ptpl_list.reserve(pv_list.size());
        std::vector<ptpl> all_ptpl_list(pv_list.size());
        std::vector<bool> useful_ptpl(pv_list.size(), false);

#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
        // note: 这个文章在实现的时候 第一层voxel并没有严格作为根节点，而是现有一个层次的结构，这样方便管理
        //* 逐点计算
        for (size_t i = 0; i < pv_list.size(); ++i)
        {
            //* 计算体素位置
            const pointWithCov &pv = pv_list[i];
            float loc_xyz[3];
            for (int j = 0; j < 3; j++)
            {
                loc_xyz[j] = pv.point_world[j] / voxel_size;
                if (loc_xyz[j] < 0)
                {
                    loc_xyz[j] -= 1.0;
                }
            }
            VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
            auto iter = voxel_map.find(position);

            //* 如果存在 octotree 对象
            if (iter != voxel_map.end())
            {
                OctoTree *current_octo = iter->second;
                ptpl single_ptpl;
                bool is_sucess = false;
                double prob    = 0;

                //* 1.构建点面匹配对，返回值是single_ptpl，包含了点和面的所有信息
                build_single_residual(pv, current_octo, 0, max_layer, sigma_num,
                                      is_sucess, prob, single_ptpl);

                //* 如果不成功,根据当前点偏离voxel的程度,查找临近的voxel，再尝试一次构建
                // note: 这里是为了处理点落在两个voxel边界的情况，可能真实匹配的平面在临近的voxel中
                if (!is_sucess)
                {
                    // 在中心点一侧偏离超过 1/4 体素长度（quater_length_），有可能是在 voxel 边缘，
                    // 对这个方向的 voxel_loc 偏离 1 位，综合三个方向，查找到最终的邻近 voxel
                    VOXEL_LOC near_position = position;
                    if (loc_xyz[0] > (current_octo->voxel_center_[0] + current_octo->quater_length_))
                    {
                        near_position.x = near_position.x + 1;
                    }
                    else if (loc_xyz[0] < (current_octo->voxel_center_[0] - current_octo->quater_length_))
                    {
                        near_position.x = near_position.x - 1;
                    }
                    if (loc_xyz[1] > (current_octo->voxel_center_[1] + current_octo->quater_length_))
                    {
                        near_position.y = near_position.y + 1;
                    }
                    else if (loc_xyz[1] < (current_octo->voxel_center_[1] - current_octo->quater_length_))
                    {
                        near_position.y = near_position.y - 1;
                    }
                    if (loc_xyz[2] > (current_octo->voxel_center_[2] + current_octo->quater_length_))
                    {
                        near_position.z = near_position.z + 1;
                    }
                    else if (loc_xyz[2] < (current_octo->voxel_center_[2] - current_octo->quater_length_))
                    {
                        near_position.z = near_position.z - 1;
                    }

                    // 邻近体素位置存在 OctoTree 对象
                    auto iter_near = voxel_map.find(near_position);
                    if (iter_near != voxel_map.end())
                    {
                        build_single_residual(pv, iter_near->second, 0, max_layer, sigma_num,
                                              is_sucess, prob, single_ptpl);
                    }
                }

                //* 查找匹配成功，记录匹配结果到 all_ptpl_list 的对应位置中
                if (is_sucess)
                {
                    useful_ptpl[i]   = true;
                    all_ptpl_list[i] = single_ptpl;
                }
                else
                {
                    useful_ptpl[i] = false;
                }
            }
        }

        //* 从 all_ptpl_list 依次记录到最终的 ptpl_list
        for (size_t i = 0; i < useful_ptpl.size(); i++)
        {
            if (useful_ptpl[i])
            {
                ptpl_list.push_back(all_ptpl_list[i]);
            }
        }
    }

    void BuildResidualListNormal(const unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                                 const double voxel_size, const double sigma_num, const int max_layer,
                                 const std::vector<pointWithCov> &pv_list, std::vector<ptpl> &ptpl_list,
                                 std::vector<Eigen::Vector3d> &non_match)
    {
        ptpl_list.clear();
        std::vector<size_t> index(pv_list.size());
        for (size_t i = 0; i < pv_list.size(); ++i)
        {
            pointWithCov pv = pv_list[i];
            float loc_xyz[3];
            for (int j = 0; j < 3; j++)
            {
                loc_xyz[j] = pv.point_world[j] / voxel_size;
                if (loc_xyz[j] < 0)
                {
                    loc_xyz[j] -= 1.0;
                }
            }
            VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
            auto iter = voxel_map.find(position);
            if (iter != voxel_map.end())
            {
                OctoTree *current_octo = iter->second;
                ptpl single_ptpl;
                bool is_sucess = false;
                double prob    = 0;
                build_single_residual(pv, current_octo, 0, max_layer, sigma_num,
                                      is_sucess, prob, single_ptpl);

                if (!is_sucess)
                {
                    VOXEL_LOC near_position = position;
                    if (loc_xyz[0] > (current_octo->voxel_center_[0] + current_octo->quater_length_))
                    {
                        near_position.x = near_position.x + 1;
                    }
                    else if (loc_xyz[0] < (current_octo->voxel_center_[0] - current_octo->quater_length_))
                    {
                        near_position.x = near_position.x - 1;
                    }
                    if (loc_xyz[1] > (current_octo->voxel_center_[1] + current_octo->quater_length_))
                    {
                        near_position.y = near_position.y + 1;
                    }
                    else if (loc_xyz[1] < (current_octo->voxel_center_[1] - current_octo->quater_length_))
                    {
                        near_position.y = near_position.y - 1;
                    }
                    if (loc_xyz[2] > (current_octo->voxel_center_[2] + current_octo->quater_length_))
                    {
                        near_position.z = near_position.z + 1;
                    }
                    else if (loc_xyz[2] < (current_octo->voxel_center_[2] - current_octo->quater_length_))
                    {
                        near_position.z = near_position.z - 1;
                    }
                    auto iter_near = voxel_map.find(near_position);
                    if (iter_near != voxel_map.end())
                    {
                        build_single_residual(pv, iter_near->second, 0, max_layer, sigma_num,
                                              is_sucess, prob, single_ptpl);
                    }
                }
                if (is_sucess)
                {
                    ptpl_list.push_back(single_ptpl);
                }
                else
                {
                    non_match.push_back(pv.point_world);
                }
            }
        }
    }

    void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec,
                         const Eigen::Vector3d &z_vec, geometry_msgs::Quaternion &q)
    {
        Eigen::Matrix3d rot;
        rot << x_vec(0), x_vec(1), x_vec(2), y_vec(0), y_vec(1), y_vec(2), z_vec(0),
            z_vec(1), z_vec(2);
        Eigen::Matrix3d rotation = rot.transpose();
        Eigen::Quaterniond eq(rotation);
        q.w = eq.w();
        q.x = eq.x();
        q.y = eq.y();
        q.z = eq.z();
    }

    void CalcQuation(const Eigen::Vector3d &vec, const int axis, geometry_msgs::Quaternion &q)
    {
        Eigen::Vector3d x_body = vec;
        Eigen::Vector3d y_body(1, 1, 0);
        if (x_body(2) != 0)
        {
            y_body(2) = -(y_body(0) * x_body(0) + y_body(1) * x_body(1)) / x_body(2);
        }
        else
        {
            if (x_body(1) != 0)
            {
                y_body(1) = -(y_body(0) * x_body(0)) / x_body(1);
            }
            else
            {
                y_body(0) = 0;
            }
        }
        y_body.normalize();
        Eigen::Vector3d z_body = x_body.cross(y_body);
        Eigen::Matrix3d rot;

        rot << x_body(0), x_body(1), x_body(2), y_body(0), y_body(1), y_body(2),
            z_body(0), z_body(1), z_body(2);
        Eigen::Matrix3d rotation = rot.transpose();
        if (axis == 2)
        {
            Eigen::Matrix3d rot_inc;
            rot_inc << 0, 0, 1, 0, 1, 0, -1, 0, 0;
            rotation = rotation * rot_inc;
        }
        Eigen::Quaterniond eq(rotation);
        q.w = eq.w();
        q.x = eq.x();
        q.y = eq.y();
        q.z = eq.z();
    }

    void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub,
                        const std::string plane_ns, const Plane &single_plane,
                        const float alpha, const Eigen::Vector3d rgb)
    {
        visualization_msgs::Marker plane;
        plane.header.frame_id = "camera_init";
        plane.header.stamp    = ros::Time();
        plane.ns              = plane_ns;
        plane.id              = single_plane.id;
        plane.type            = visualization_msgs::Marker::CYLINDER;
        plane.action          = visualization_msgs::Marker::ADD;
        plane.pose.position.x = single_plane.center[0];
        plane.pose.position.y = single_plane.center[1];
        plane.pose.position.z = single_plane.center[2];
        geometry_msgs::Quaternion q;
        CalcVectQuation(single_plane.x_normal, single_plane.y_normal,
                        single_plane.normal, q);
        plane.pose.orientation = q;
        plane.scale.x          = 3 * sqrt(single_plane.max_eigen_value);
        plane.scale.y          = 3 * sqrt(single_plane.mid_eigen_value);
        plane.scale.z          = 2 * sqrt(single_plane.min_eigen_value);
        plane.color.a          = alpha;
        plane.color.r          = rgb(0);
        plane.color.g          = rgb(1);
        plane.color.b          = rgb(2);
        plane.lifetime         = ros::Duration();
        plane_pub.markers.push_back(plane);
    }

    void pubNoPlaneMap(const std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map,
                       const ros::Publisher &plane_map_pub)
    {
        // int id = 0;
        ros::Rate loop(500);
        float use_alpha = 0.8;
        visualization_msgs::MarkerArray voxel_plane;
        voxel_plane.markers.reserve(1000000);
        for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++)
        {
            if (!iter->second->plane_ptr_->is_plane)
            {
                for (uint i = 0; i < 8; i++)
                {
                    if (iter->second->leaves_[i] != nullptr)
                    {
                        OctoTree *temp_octo_tree = iter->second->leaves_[i];
                        if (!temp_octo_tree->plane_ptr_->is_plane)
                        {
                            for (uint j = 0; j < 8; j++)
                            {
                                if (temp_octo_tree->leaves_[j] != nullptr)
                                {
                                    if (!temp_octo_tree->leaves_[j]->plane_ptr_->is_plane)
                                    {
                                        Eigen::Vector3d plane_rgb(1, 1, 1);
                                        pubSinglePlane(voxel_plane, "no_plane",
                                                       *(temp_octo_tree->leaves_[j]->plane_ptr_), use_alpha, plane_rgb);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        plane_map_pub.publish(voxel_plane);
        loop.sleep();
    }

    void pubVoxelMap(const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                     const int pub_max_voxel_layer, const ros::Publisher &plane_map_pub)
    {
        double max_trace = 0.25;
        double pow_num   = 0.2;
        ros::Rate loop(500);
        float use_alpha = 0.8;
        visualization_msgs::MarkerArray voxel_plane;
        voxel_plane.markers.reserve(1000000);
        std::vector<Plane> pub_plane_list;
        for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++)
        {
            GetUpdatePlane(iter->second, pub_max_voxel_layer, pub_plane_list);
        }
        for (size_t i = 0; i < pub_plane_list.size(); i++)
        {
            V3D plane_cov = pub_plane_list[i].plane_cov.block<3, 3>(0, 0).diagonal();
            double trace  = plane_cov.sum();
            if (trace >= max_trace)
            {
                trace = max_trace;
            }
            trace = trace * (1.0 / max_trace);
            trace = pow(trace, pow_num);
            uint8_t r, g, b;
            mapJet(trace, 0, 1, r, g, b);
            Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
            double alpha;
            if (pub_plane_list[i].is_plane)
            {
                alpha = use_alpha;
            }
            else
            {
                alpha = 0;
            }
            pubSinglePlane(voxel_plane, "plane", pub_plane_list[i], alpha, plane_rgb);
        }
        plane_map_pub.publish(voxel_plane);
        loop.sleep();
    }

    void pubPlaneMap(const std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map,
                     const ros::Publisher &plane_map_pub)
    {
        // OctoTree *current_octo = nullptr;

        double max_trace = 0.25;
        double pow_num   = 0.2;
        ros::Rate loop(500);
        float use_alpha = 1.0;
        visualization_msgs::MarkerArray voxel_plane;
        voxel_plane.markers.reserve(1000000);

        for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++)
        {
            if (iter->second->plane_ptr_->is_update)
            {
                Eigen::Vector3d normal_rgb(0.0, 1.0, 0.0);

                V3D plane_cov =
                    iter->second->plane_ptr_->plane_cov.block<3, 3>(0, 0).diagonal();
                double trace = plane_cov.sum();
                if (trace >= max_trace)
                {
                    trace = max_trace;
                }
                trace = trace * (1.0 / max_trace);
                trace = pow(trace, pow_num);
                uint8_t r, g, b;
                mapJet(trace, 0, 1, r, g, b);
                Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
                // Eigen::Vector3d plane_rgb(1, 0, 0);
                float alpha = 0.0;
                if (iter->second->plane_ptr_->is_plane)
                {
                    alpha = use_alpha;
                }
                else
                {
                    // std::cout << "delete plane" << std::endl;
                }
                // if (iter->second->update_enable_) {
                //   plane_rgb << 1, 0, 0;
                // } else {
                //   plane_rgb << 0, 0, 1;
                // }
                pubSinglePlane(voxel_plane, "plane", *(iter->second->plane_ptr_), alpha,
                               plane_rgb);

                iter->second->plane_ptr_->is_update = false;
            }
            else
            {
                for (uint i = 0; i < 8; i++)
                {
                    if (iter->second->leaves_[i] != nullptr)
                    {
                        if (iter->second->leaves_[i]->plane_ptr_->is_update)
                        {
                            Eigen::Vector3d normal_rgb(0.0, 1.0, 0.0);

                            V3D plane_cov = iter->second->leaves_[i]
                                                ->plane_ptr_->plane_cov.block<3, 3>(0, 0)
                                                .diagonal();
                            double trace = plane_cov.sum();
                            if (trace >= max_trace)
                            {
                                trace = max_trace;
                            }
                            trace = trace * (1.0 / max_trace);
                            // trace = (max_trace - trace) / max_trace;
                            trace = pow(trace, pow_num);
                            uint8_t r, g, b;
                            mapJet(trace, 0, 1, r, g, b);
                            Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
                            plane_rgb << 0, 1, 0;
                            // fabs(iter->second->leaves_[i]->plane_ptr_->normal[0]),
                            //     fabs(iter->second->leaves_[i]->plane_ptr_->normal[1]),
                            //     fabs(iter->second->leaves_[i]->plane_ptr_->normal[2]);
                            float alpha = 0.0;
                            if (iter->second->leaves_[i]->plane_ptr_->is_plane)
                            {
                                alpha = use_alpha;
                            }
                            else
                            {
                                // std::cout << "delete plane" << std::endl;
                            }
                            pubSinglePlane(voxel_plane, "plane",
                                           *(iter->second->leaves_[i]->plane_ptr_), alpha, plane_rgb);
                            // loop.sleep();
                            iter->second->leaves_[i]->plane_ptr_->is_update = false;
                            // loop.sleep();
                        }
                        else
                        {
                            OctoTree *temp_octo_tree = iter->second->leaves_[i];
                            for (uint j = 0; j < 8; j++)
                            {
                                if (temp_octo_tree->leaves_[j] != nullptr)
                                {
                                    if (temp_octo_tree->leaves_[j]->octo_state_ == 0 && temp_octo_tree->leaves_[j]->plane_ptr_->is_update)
                                    {
                                        if (temp_octo_tree->leaves_[j]->plane_ptr_->is_plane)
                                        {
                                            // std::cout << "subsubplane" << std::endl;
                                            Eigen::Vector3d normal_rgb(0.0, 1.0, 0.0);
                                            V3D plane_cov =
                                                temp_octo_tree->leaves_[j]
                                                    ->plane_ptr_->plane_cov.block<3, 3>(0, 0)
                                                    .diagonal();
                                            double trace = plane_cov.sum();
                                            if (trace >= max_trace)
                                            {
                                                trace = max_trace;
                                            }
                                            trace = trace * (1.0 / max_trace);
                                            // trace = (max_trace - trace) / max_trace;
                                            trace = pow(trace, pow_num);
                                            uint8_t r, g, b;
                                            mapJet(trace, 0, 1, r, g, b);
                                            Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
                                            plane_rgb << 0, 0, 1;
                                            float alpha = 0.0;
                                            if (temp_octo_tree->leaves_[j]->plane_ptr_->is_plane)
                                            {
                                                alpha = use_alpha;
                                            }

                                            pubSinglePlane(voxel_plane, "plane",
                                                           *(temp_octo_tree->leaves_[j]->plane_ptr_),
                                                           alpha, plane_rgb);
                                            // loop.sleep();
                                            temp_octo_tree->leaves_[j]->plane_ptr_->is_update = false;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        plane_map_pub.publish(voxel_plane);
        // plane_map_pub.publish(voxel_norm);
        loop.sleep();
        // cout << "[Map Info] Plane counts:" << plane_count
        //      << " Sub Plane counts:" << sub_plane_count
        //      << " Sub Sub Plane counts:" << sub_sub_plane_count << endl;
        // cout << "[Map Info] Update plane counts:" << update_count
        //      << "total size: " << feat_map.size() << endl;
    }

    /**
     * @brief 计算lidar坐标系下给定点的协方差
     *        ? 函数名有问题，应该是 calcLidarCov
     * @param pb
     * @param range_inc
     * @param degree_inc
     * @return 包含角度不确定性和测距不确定性的协方差阵
     */
    M3D calcLidarCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc)
    {
        float range     = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);  //< 测距
        float range_var = range_inc * range_inc;                                //< 测距方差

        Eigen::Matrix2d direction_var;  //< 角度协方差阵（球面坐标的两个角度）
        direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);

        Eigen::Vector3d direction(pb);  //< 单位方向向量
        direction.normalize();

        Eigen::Matrix3d direction_hat;  //< 方向向量的反对称矩阵
        direction_hat << 0, -direction(2), direction(1), direction(2), 0,
            -direction(0), -direction(1), direction(0), 0;

        //< 两个与 direction 正交的基向量
        Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
        base_vector1.normalize();
        Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
        base_vector2.normalize();

        Eigen::Matrix<double, 3, 2> N;  //< 包含两个正交基的基向量矩阵
        N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);

        Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;  //< 用于计算协方差

        //* 返回协方差阵
        return direction * range_var * direction.transpose() + A * direction_var * A.transpose();
    }
}

#endif