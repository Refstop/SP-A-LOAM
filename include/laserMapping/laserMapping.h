#ifndef SP_A_LOAM_LASERMAPPING_H
#define SP_A_LOAM_LASERMAPPING_H

#include "common.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <mutex>
#include <queue>
#include <thread>

#include "lidarFactor.h"
#include "laserMapping/laserMappingOption.h"

class laserMapping {
public:
    laserMapping(lmOption option);
    ~laserMapping();
    void setCornerLast(const double timeStamp, pcl::PointCloud<PointType> laserCloud);
    void setSurfLast(const double timeStamp, pcl::PointCloud<PointType> laserCloud);
    void setFullRes(const double timeStamp, pcl::PointCloud<PointType> laserCloud);
    void setOdometry(const double timeStamp, Eigen::Quaterniond q_odom, Eigen::Vector3d t_odom,
                                             Eigen::Quaterniond &q_w_curr, Eigen::Vector3d &t_w_curr);
    OdometryStamped::Ptr getOdometry() { return odometryLast_; }
    PointCloudStamped::Ptr getLaserCloudCornerLast() { return laserCloudCornerLast_; }
    PointCloudStamped::Ptr getLaserCloudSurfLast() { return laserCloudSurfLast_; }
    PointCloudStamped::Ptr getLaserCloudFullRes() { return laserCloudFullRes_; }
    PointCloudStamped::Ptr getLaserCloudFullResGlobal();
    PointCloudStamped::Ptr getLaserCloudSurround();
    PointCloudStamped::Ptr getLaserCloudMap();
    
    bool run();
private:
    void transformAssociateToMap();
    void pointAssociateToMap(PointType const *const pi, PointType *const po);
    void transformUpdate();
    int cube(int i, int j, int k);
    lmOption opt_;
    PointCloudStamped::Ptr laserCloudCornerLast_, laserCloudSurfLast_, laserCloudFullRes_;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap_, laserCloudSurfFromMap_;

    OdometryStamped::Ptr odometryLast_;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap_, kdtreeSurfFromMap_;
    double parameters_[7] = {0, 0, 0, 1, 0, 0, 0}; // qx, qy, qz, qw, tx, ty, tz
    /*
    	Eigen::Map은 생성자 파라미터로 주소(parameters)를 받아
    	해당 주소에 있는 변수들을 템플릿 자료형(Eigen::Quaterniond)으로 변환한다.
    	140번 줄의 단순히 자료형으로 선언하는 것과 차이는 없으나, sparse matrix 연산에 유리하다.
    	하지만 왜 quaternion 크기의 배열을 Eigen::Map하는지?는 모르겠다
    */
    Eigen::Map<Eigen::Quaterniond> q_w_curr_;
    Eigen::Map<Eigen::Vector3d> t_w_curr_;
    // map2odom
    Eigen::Quaterniond q_wmap_wodom_;
    Eigen::Vector3d t_wmap_wodom_;
    // odom2laser(base_link)
    Eigen::Quaterniond q_wodom_curr_;
    Eigen::Vector3d t_wodom_curr_;

    std::queue<PointCloudStamped::Ptr> cornerLastBuf_, surfLastBuf_, fullResBuf_;
    std::queue<OdometryStamped::Ptr> odometryBuf_;
    std::mutex mBuf_;

    pcl::VoxelGrid<PointType> downSizeFilterCorner_, downSizeFilterSurf_;

    // (추정)Map feature로 사용 가능한 포인트와 그 주변놈들(생략대상)
    int laserCloudSurroundNum_;
    int laserCloudValidInd_[125];
    int laserCloudSurroundInd_[125];

    pcl::PointCloud<PointType>::Ptr *laserCloudCornerArray_, *laserCloudSurfArray_;
};

#endif