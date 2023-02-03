//
// Created by bhbhchoi on 23. 1. 25.
//

#ifndef SP_A_LOAM_SCANREGISTRATIONOPTION_H
#define SP_A_LOAM_SCANREGISTRATIONOPTION_H
#include "common.h"

class scanRegistrationOption {
public:
    scanRegistrationOption(std::string LIDAR_TYPE = "VLP16", bool verbose = true);
    scanRegistrationOption(std::string LIDAR_TYPE = "VLP16", double MINIMUM_RANGE = 0.1, bool verbose = true);
    const double scanPeriod;
    const int systemDelay;
    int systemInitCount;
    bool systemInited;
    int N_SCANS;
    double MINIMUM_RANGE;
    std::string LIDAR_TYPE;
    const bool verbose;
};
typedef scanRegistrationOption srOption;

#endif //SP_A_LOAM_SCANREGISTRATIONOPTION_H
