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

/* struct LidarPlaneFactor {
    LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
                     Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_)
        : curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_), last_point_m(last_point_m_) {
        ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
        ljm_norm.normalize();
    }
    template <typename T>
    bool operator()(const T *q, const T *t, T *residual) const {
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
        Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

        Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], t[2]};

        Eigen::Matrix<T, 3, 1> lp;
        lp = q_last_curr * cp + t_last_curr;

        residual[0] = (lp - lpj).dot(ljm);

        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
                                       const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_) {
        return (new ceres::AutoDiffCostFunction<LidarPlaneFactor, 1, 4, 3>(
                new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_)));
    }
    Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
    Eigen::Vector3d ljm_norm;
};*/

struct LidarPlaneFactor
{
	LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
					 Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_)
		: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
		  last_point_m(last_point_m_)
	{
		ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
		ljm_norm.normalize();
	}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
		//Eigen::Matrix<T, 3, 1> lpl{T(last_point_l.x()), T(last_point_l.y()), T(last_point_l.z())};
		//Eigen::Matrix<T, 3, 1> lpm{T(last_point_m.x()), T(last_point_m.y()), T(last_point_m.z())};
		Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

		residual[0] = (lp - lpj).dot(ljm);

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
									   const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneFactor, 1, 4, 3>(
			new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_)));
	}

	Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
	Eigen::Vector3d ljm_norm;
};

// 라이다가 추출한 plane의 방향 norm값을 factor로 사용
struct LidarPlaneNormFactor {
    LidarPlaneNormFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_, double negative_OA_dot_norm_)
        :curr_point(curr_point_), plane_unit_norm(plane_unit_norm_), negative_OA_dot_norm(negative_OA_dot_norm_) {}
    
    // operator() (파라미터): 클래스 객체를 마치 함수처럼 사용할 수 있게 오버로딩
    template <typename T>
    bool operator() (const T *q, const T *t, T *residual) const {
        // 어째선지 operator의 는 주소값이지만, 대부분 "배열"의 이름으로만 받는다.
        // Eigen::Matrix의 주소값 같은거는 받을 수 없는 것인가?
        Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> point_w;
        // Eigen 라이브러리에서 쿼터니안*point 연산을 잘 지원해 준다
        // cp는 라이다 기준 point이므로, 아래 식을 통해 world 좌표계 기준 point로 변환
        point_w = q_w_curr * cp + t_w_curr;

        Eigen::Matrix<T, 3, 1> norm{T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z())};
        residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_,
                                       const double negative_OA_dot_norm_) {
        return new ceres::AutoDiffCostFunction<LidarPlaneNormFactor, 1, 4, 3>(
            new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)
        );
    }

    Eigen::Vector3d curr_point, plane_unit_norm;
    double negative_OA_dot_norm;
};

#endif //SP_A_LOAM_LIDARFACTOR_H
