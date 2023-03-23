#ifndef SP_A_LOAM_LASERODOMETRYOPTION_H
#define SP_A_LOAM_LASERODOMETRYOPTION_H

#include "common.h"

class laserMappingOption {
public:
    laserMappingOption(float lineRes = 0.4, float planeRes = 0.8, bool verbose = true);
    const int laserCloudWidth;
    const int laserCloudHeight;
    const int laserCloudDepth;
    int laserCloudNum;
    int laserCloudCenWidth;
    int laserCloudCenHeight;
    int laserCloudCenDepth;
    float lineRes;
    float planeRes;
    const bool verbose;
};

typedef laserMappingOption lmOption;

#endif