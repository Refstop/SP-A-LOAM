#include "laserMapping/laserMappingOption.h"

laserMappingOption::laserMappingOption(float lineRes, float planeRes, bool verbose):
    laserCloudCenWidth(10),
    laserCloudCenHeight(10),
    laserCloudCenDepth(5),
    laserCloudWidth(21),
    laserCloudHeight(21),
    laserCloudDepth(11),
    lineRes(lineRes),
    planeRes(planeRes),
    verbose(verbose) {
    laserCloudNum = laserCloudWidth*laserCloudHeight*laserCloudDepth;
}