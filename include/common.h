//
// Created by bhbhchoi on 23. 1. 25.
//

#ifndef SP_A_LOAM_COMMON_H
#define SP_A_LOAM_COMMON_H
#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <cstdio>
#include <boost/smart_ptr/shared_ptr.hpp>

#include <eigen3/Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using std::cout;
using std::endl;
typedef pcl::PointXYZI PointType;

class timeStamped {
public:
    timeStamped(const double timeStamp):
        timeStamp(timeStamp) {}
    virtual void clear() {
        timeStamp = 0.0;
    }
    double timeStamp;
};

class PointCloudStamped: public timeStamped {
public:
    PointCloudStamped():
        timeStamped(0.0), laserCloud(pcl::PointCloud<PointType>()) {}
    PointCloudStamped(double timeStamp, pcl::PointCloud<PointType> laserCloud):
        timeStamped(timeStamp), laserCloud(std::move(laserCloud)) {}
    typedef boost::shared_ptr<PointCloudStamped> Ptr;
    void clear() override {
        timeStamp = 0.0;
        laserCloud.clear();
    }
    pcl::PointCloud<PointType> laserCloud;
};

class OdometryStamped: public timeStamped {
public:
    OdometryStamped():
        timeStamped(0.0), q_odom(Eigen::Quaterniond(1, 0, 0, 0)), t_odom(Eigen::Vector3d(0, 0, 0)) {}
    OdometryStamped(double timeStamp, Eigen::Quaterniond q_odom, Eigen::Vector3d t_odom):
        timeStamped(timeStamp), q_odom(q_odom), t_odom(t_odom) {}
    typedef boost::shared_ptr<OdometryStamped> Ptr;
    Eigen::Quaterniond q_odom;
    Eigen::Vector3d t_odom;
};


#endif //SP_A_LOAM_COMMON_H
