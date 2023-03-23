//
// Created by bhbhchoi on 23. 1. 31.
//
#include "laserOdometry/laserOdometryOption.h"

laserOdometryOption::laserOdometryOption():
    SCAN_PERIOD(0.1),
    DISTANCE_SQ_THRESHOLD(25),
    NEARBY_SCAN(2.5),
    skipFrameNum(5),
    systemInited(false),
    verbose(true) {}

laserOdometryOption::laserOdometryOption(const int skipFrameNum, bool verbose):
    SCAN_PERIOD(0.1),
    DISTANCE_SQ_THRESHOLD(25),
    NEARBY_SCAN(2.5),
    skipFrameNum(skipFrameNum),
    systemInited(false),
    verbose(verbose) {}
