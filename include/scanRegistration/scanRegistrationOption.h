//
// Created by bhbhchoi on 23. 1. 25.
//

#ifndef SP_A_LOAM_SCANREGISTRATIONOPTION_H
#define SP_A_LOAM_SCANREGISTRATIONOPTION_H
#include "common.h"

class scanRegistrationOption {
public:
    scanRegistrationOption(std::string LIDAR_TYPE, bool verbose);
    scanRegistrationOption(std::string LIDAR_TYPE, double MINIMUM_RANGE, bool verbose);
    const double scanPeriod;
    const int systemDelay;
    int systemInitCount;
    bool systemInited;
    int N_SCANS;
    double MINIMUM_RANGE;
    bool verbose;
    std::string LIDAR_TYPE;
};
typedef scanRegistrationOption srOption;

#endif //SP_A_LOAM_SCANREGISTRATIONOPTION_H
