//
// Created by bhbhchoi on 23. 1. 31.
//

#ifndef SP_A_LOAM_LASERODOMETRYOPTION_H
#define SP_A_LOAM_LASERODOMETRYOPTION_H
#include "common.h"

class laserOdometryOption {
public:
    laserOdometryOption();
    laserOdometryOption(const int skipFrameNum = 5, bool verbose = true);
    const double SCAN_PERIOD;
    const double DISTANCE_SQ_THRESHOLD;
    const double NEARBY_SCAN;
    const int skipFrameNum;
    bool systemInited;
    const bool verbose;
};

typedef laserOdometryOption loOption;

#endif //SP_A_LOAM_LASERODOMETRYOPTION_H
