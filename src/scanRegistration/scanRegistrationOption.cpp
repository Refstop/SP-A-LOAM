//
// Created by bhbhchoi on 23. 1. 25.
//

#include "scanRegistration/scanRegistrationOption.h"

scanRegistrationOption::scanRegistrationOption(std::string LIDAR_TYPE, bool verbose):
    scanPeriod(0.1),
    systemDelay(0),
    systemInitCount(0),
    systemInited(false),
    MINIMUM_RANGE(0.1),
    verbose(verbose) {
    this->LIDAR_TYPE = LIDAR_TYPE;
    if(LIDAR_TYPE == "KITTI" || LIDAR_TYPE == "VLP16") N_SCANS = 16;
    else if(LIDAR_TYPE == "VLP32") N_SCANS = 32; // 있는지모름 일단예시
}

scanRegistrationOption::scanRegistrationOption(std::string LIDAR_TYPE, double MINIMUM_RANGE, bool verbose):
    scanPeriod(0.1),
    systemDelay(0),
    systemInitCount(0),
    systemInited(false),
    verbose(verbose) {
    this->LIDAR_TYPE = LIDAR_TYPE;
    this->MINIMUM_RANGE = MINIMUM_RANGE;
    if(LIDAR_TYPE == "KITTI" || LIDAR_TYPE == "VLP16") N_SCANS = 16;
    else if(LIDAR_TYPE == "HDL32") N_SCANS = 32; // 있는지모름 일단예시
}
