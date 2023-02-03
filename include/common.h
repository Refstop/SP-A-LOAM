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
#include <boost/smart_ptr/shared_ptr.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using std::cout;
using std::endl;
typedef pcl::PointXYZI PointType;

class PointTypeStamped {
public:
    PointTypeStamped(double timeStamp, pcl::PointCloud<PointType> laserCloud):
        timeStamp(timeStamp), laserCloud(std::move(laserCloud)) {}
    typedef boost::shared_ptr<PointTypeStamped> Ptr;
    void clear() {
        timeStamp = 0.0;
        laserCloud.clear();
    }
    double timeStamp;
    pcl::PointCloud<PointType> laserCloud;
};


#endif //SP_A_LOAM_COMMON_H
