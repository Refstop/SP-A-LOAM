//
// Created by bhbhchoi on 23. 2. 1.
//

#ifndef SP_A_LOAM_LIDARFACTOR_H
#define SP_A_LOAM_LIDARFACTOR_H
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

struct LidarEdgeFactor {
    LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
                    Eigen::Vector3d last_point_b_)
        : curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_) {}

    template <typename T>
    bool operator()(const T *q, const T *t, T *residual) const {
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
        Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

        Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], t[2]};

        // 현재 point에
        Eigen::Matrix<T, 3, 1> lp;
        lp = q_last_curr * cp + t_last_curr;

        Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb); // 외적
        Eigen::Matrix<T, 3, 1> de = lpa - lpb; // norm을 구하기 위한 연산

        residual[0] = nu.x() / de.norm();
        residual[1] = nu.y() / de.norm();
        residual[2] = nu.z() / de.norm();

        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
                                       const Eigen::Vector3d last_point_b_) {
        // ceres::AutoDiffCostFunction<(class 명), (residual 사이즈),
        // (첫번째 최적화 변수(배열, para_q_)의 사이즈), (두번째 최적화 변수(배열, para_t_)의 사이즈)>()
        return (new ceres::AutoDiffCostFunction<LidarEdgeFactor, 3, 4, 3>(
                        new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_)));
    }

    Eigen::Vector3d curr_point, last_point_a, last_point_b;
};

struct LidarPlaneFactor {
    LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_i_,
                     Eigen::Vector3d last_point_j_, Eigen::Vector3d last_point_k_)
        : curr_point(curr_point_), last_point_i(last_point_i_), last_point_j(last_point_j_), last_point_k(last_point_k_) {
        jik_norm = (last_point_i - last_point_j).cross(last_point_i - last_point_k);
        jik_norm.normalize();
    }
    template <typename T>
    bool operator()(const T *q, const T *t, T *residual) const {
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> lpi{T(last_point_i.x()), T(last_point_i.y()), T(last_point_i.z())};
        Eigen::Matrix<T, 3, 1> jik{T(jik_norm.x()), T(jik_norm.y()), T(jik_norm.z())};

        Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], t[2]};

        Eigen::Matrix<T, 3, 1> lp;
        lp = q_last_curr * cp + t_last_curr;

        residual[0] = (lp - lpi).dot(jik);

        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_i_,
                                       const Eigen::Vector3d last_point_j_, const Eigen::Vector3d last_point_k_) {
        return (new ceres::AutoDiffCostFunction<LidarPlaneFactor, 1, 4, 3>(
                new LidarPlaneFactor(curr_point_, last_point_i_, last_point_j_, last_point_k_)));
    }
    Eigen::Vector3d curr_point, last_point_i, last_point_j, last_point_k;
    Eigen::Vector3d jik_norm;
};

#endif //SP_A_LOAM_LIDARFACTOR_H
