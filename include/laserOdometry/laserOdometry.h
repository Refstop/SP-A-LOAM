//
// Created by bhbhchoi on 23. 1. 31.
//

#ifndef SP_A_LOAM_LASERODOMETRY_H
#define SP_A_LOAM_LASERODOMETRY_H
#include "common.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <queue>

#include "laserOdometry/laserOdometryOption.h"
#include "lidarFactor.h"

class laserOdometry {
public:
    laserOdometry(loOption option);
    void setFullRes(double timeStamp, pcl::PointCloud<PointType> &fullRes);
    void setCornerSharpBuf(double timeStamp, pcl::PointCloud<PointType> &cornerPointsSharp);
    void setCornerPointsLessSharp(double timeStamp, pcl::PointCloud<PointType> &cornerPointsLessSharp);
    void setSurfPointsFlat(double timeStamp, pcl::PointCloud<PointType> &surfPointsFlat);
    void setSurfPointsLessFlat(double timeStamp, pcl::PointCloud<PointType> &surfPointsLessFlat);
    PointCloudStamped::Ptr getLaserCloudCornerLast() { return laserCloudCornerLast_; }
    PointCloudStamped::Ptr getLaserCloudSurfLast() { return laserCloudSurfLast_; }
    PointCloudStamped::Ptr getLaserCloudFullRes() { return laserCloudFullRes_; }


    Eigen::Quaterniond getOdomRotation() { return q_w_curr_; }
    Eigen::Vector3d getOdomTranslation() { return t_w_curr_; }

    bool run();

private:
    void transformToStart(PointType const *const pi, PointType *const po);
    double pointDistance(const PointType point1, const PointType point2);
    void findCornerIndices(PointType const *const pi, int &closestPointInd, int &minPointInd2);
    void findSurfIndices(PointType const *const pi, int &closestPointInd, int &minPointInd2, int &minPointInd3);
    loOption opt_;
    int corner_correspondence_, plane_correspondence_;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast_, kdtreeSurfLast_;
    PointCloudStamped::Ptr laserCloudCornerLast_, laserCloudSurfLast_, laserCloudFullRes_;

    Eigen::Quaterniond q_w_curr_;
    Eigen::Vector3d t_w_curr_;

    double para_q_[4] = {0, 0, 0, 1};
    double para_t_[3] = {0, 0, 0};

    Eigen::Map<Eigen::Quaterniond> q_last_curr_;
    Eigen::Map<Eigen::Vector3d> t_last_curr_;

    std::queue<PointCloudStamped::Ptr> cornerSharpBuf_, cornerLessSharpBuf_, surfFlatBuf_, surfFlatLessBuf_, fullPointsBuf_;
    std::mutex mBuf_;
};

#endif //SP_A_LOAM_LASERODOMETRY_H
