// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Created by bhbhchoi on 23. 1. 25.

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.

// Original source code: SC-A-LOAM의 src/scanRegistration.cpp

#ifndef SP_A_LOAM_SCANREGISTRATION_H
#define SP_A_LOAM_SCANREGISTRATION_H
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "common.h"
#include "scanRegistration/scanRegistrationOption.h"

using std::atan2;
using std::sin;
using std::cos;

//static float cloudCurvature_[400000];

class scanRegistration {
public:
    scanRegistration(srOption option);
    pcl::PointCloud<PointType> getLaserCloud() { return *laserCloud_; }
    pcl::PointCloud<PointType> getCornerPointsSharp() { return *cornerPointsSharp_; }
    pcl::PointCloud<PointType> getCornerPointsLessSharp() { return *cornerPointsLessSharp_; }
    pcl::PointCloud<PointType> getSurfPointsFlat() { return *surfPointsFlat_; }
    pcl::PointCloud<PointType> getSurfPointsLessFlat() { return *surfPointsLessFlat_; }

    // src/scanRegistration.cpp의 laserHandler를 작은 함수들로 쪼갠다(나중)
    void laserCloudHandler(pcl::PointCloud<pcl::PointXYZ> laserCloudIn);

private:
    static bool comp(int i, int j) {
        return (cloudCurvature_[i] < cloudCurvature_[j]);
    }
    void clearLaserClouds();
    template <typename PointT>
    void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, float thres) {
        if(&cloud_in != &cloud_out) {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }

        // unsigned int 대신에 size_t를 사용하는 이유?
        // unsigned int와 int는 64bit OS라고 해도 꼭 64bit 정수가 아닐 수 있다.
        // 하지만 size_t는 항상 OS의 bit를 따르는 부호 없는 자료형이기 때문에 size_t를 사용한다.
        size_t j = 0;
        for(size_t i = 0; i < cloud_in.size(); i++) {
            if(cloud_in.points[i].x * cloud_in.points[i].x
             + cloud_in.points[i].y * cloud_in.points[i].y
             + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres) {
                continue;
            }
            cloud_out.points[j] = cloud_in.points[i];
            j++;
        }
        // 이런 경우가 존재하나?
        if(j != cloud_in.points.size()) {
            cloud_out.points.resize(j);
        }

        cloud_out.height = 1;
        // static_cast의 목적은 형변환이다. 명시적 형변환이 허용되지만 제대로 동작하지 않는 경우 static_cast를 사용하여 형변환한다.
        // static은 형변환 시점이 컴파일 시임을 의미한다. 하지만 여기선 왜 썼는지 모르겠다. robust 형변환을 위해?
        // https://uncertainty-momo.tistory.com/68
        cloud_out.width = static_cast<uint32_t>(j);
        // NaN이나 Inf같은 invalid한 값이 없을 때 true, 근데 왜 수동설정??
        cloud_out.is_dense = true;
    }
    srOption opt_;
    pcl::PointCloud<PointType>::Ptr laserCloud_, cornerPointsSharp_, cornerPointsLessSharp_, surfPointsFlat_, surfPointsLessFlat_;
    static float cloudCurvature_[400000];
    int cloudSortInd_[400000];
    int cloudNeighborPicked_[400000];
    int cloudLabel_[400000];
};

#endif //SP_A_LOAM_SCANREGISTRATION_H