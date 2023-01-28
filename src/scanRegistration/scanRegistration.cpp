//
// Created by bhbhchoi on 23. 1. 25.
//
#include "scanRegistration/scanRegistration.h"

float scanRegistration::cloudCurvature_[400000];

// std::move를 사용하여 생성자 파라미터인 option의 소유권을 opt_로 넘겨줌으로서 불필요한 copy 연산을 하지 않을 수 있다.
scanRegistration::scanRegistration(srOption option):
opt_(std::move(option)),
laserCloud_(new pcl::PointCloud<PointType>),
cornerPointsSharp_(new pcl::PointCloud<PointType>),
cornerPointsLessSharp_(new pcl::PointCloud<PointType>),
surfPointsFlat_(new pcl::PointCloud<PointType>),
surfPointsLessFlat_(new pcl::PointCloud<PointType>) {
    if(opt_.N_SCANS != 16 && opt_.N_SCANS != 32 && opt_.N_SCANS != 64) {
        if(opt_.verbose) cout << "only support velodyne with 16, 32 or 64 scan line!\n";
        exit(0);
    }
}

void scanRegistration::clearLaserClouds() {
    laserCloud_->clear();
    cornerPointsSharp_->clear();
    cornerPointsLessSharp_->clear();
    surfPointsFlat_->clear();
    surfPointsLessFlat_->clear();
}

void scanRegistration::laserCloudHandler(pcl::PointCloud<pcl::PointXYZ> laserCloudIn) {
    if(!opt_.systemInited) {
        opt_.systemInitCount++;
        if(opt_.systemInitCount >= opt_.systemDelay) {
            opt_.systemInited = true;
        } else {
            return;
        }
    }
    clearLaserClouds();
    // scan pointcloud channel의 시작&종료 지점을 저장, VLP16의 N_SCANS는 16
    std::vector<int> scanStartInd(opt_.N_SCANS, 0);
    std::vector<int> scanEndInd(opt_.N_SCANS, 0);
    std::vector<int> indices;

    // raw pointcloud에서 NaN값, 최소 거리 threshold 이내 값을 필터링
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    removeClosedPointCloud(laserCloudIn, laserCloudIn, opt_.MINIMUM_RANGE);

    // pointcloud size 및 시작 & 종료지점 각도 계산
    size_t cloudSize = laserCloudIn.points.size();
    PointType point;
    size_t count = cloudSize;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(opt_.N_SCANS);
    for(int i = 0; i < cloudSize; i++) {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        /**********************************
         * 각 point들의 위치를 intensity에 저장하는 과정
         * LiDAR 센서의 spec에 따라서 intensity에 XX.xxx의 형태로 저장된다.
         * 정수부 XX는 해당 point가 몇 번째 channel에 위치하는지를 의미한다. 범위는 0~15이다.
         * 소수부 .xxx는 해당 point의 상대적 위치 비율을 나타낸다.
         * 시작지점~종료지점을 100%로 보고, 몇 % 지점에 이 점이 위치해 있는지를 소수로 표시한다.
         * ex) 13.125 => channel 13, 시작지점~종료지점 기준 12.5% 지점의 점임을 의미
        **********************************/
        double angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y + point.z * point.z)) * 180 / M_PI;
        // float angle = atan2(point.z, sqrt(point.x * point.x + point.y * point.y + point.z * point.z)) * 180 / M_PI;
        int scanID = 0;
        if(opt_.LIDAR_TYPE == "VLP16" && opt_.N_SCANS == 16) {
            scanID = static_cast<int>(lround((angle + 15) / 2));
            if (scanID > (opt_.N_SCANS - 1) || scanID < 0) {
                count--;
                continue;
            }
        } else if(opt_.LIDAR_TYPE == "HDL32" && opt_.N_SCANS == 32) {
            scanID = static_cast<int>((angle + 92.0 / 3.0) * 3.0 / 4.0);
            if (scanID > (opt_.N_SCANS - 1) || scanID < 0) {
                count--;
                continue;
            }
        } else {
            if(opt_.verbose) cout << "wrong scan number\n";
            return;
        }
        point.intensity = static_cast<float>(scanID);
        laserCloudScans[scanID].push_back(point);
    }

    cloudSize = count;
    // 각 channel별로 feature를 검색할 시작 & 종료지점을 지정한다.
    // Ptr은 shared_ptr으로, 스마트포인터이다. ConstPtr도 마찬가지
    for(int i = 0; i < opt_.N_SCANS; i++) {
        // laserCloudScans[i].size() = 100으로 균일이라 가정하면,
        scanStartInd[i] = static_cast<int>(laserCloud_->size() + 5); // 5 105 205 ...
        *laserCloud_ += laserCloudScans[i];
        scanEndInd[i] = static_cast<int>(laserCloud_->size() - 6); // 94 194 294 ...
    }
    // 결론적으로 앞 5개, 뒤 5개의 point를 feature poincloud를 찾는 데 사용한다.
    // => size 10의 window로 훑는것

    // 논문에서의 c(curvature)를 계산하는 부분
    // index 5~cloudSize - 6의 point를 size 10의 window로 훑는다.
    for(int i = 5; i < cloudSize - 5; i++) {
        float diffX = laserCloud_->points[i - 5].x + laserCloud_->points[i - 4].x + laserCloud_->points[i - 3].x + laserCloud_->points[i - 2].x + laserCloud_->points[i - 1].x + laserCloud_->points[i].x + laserCloud_->points[i + 1].x + laserCloud_->points[i + 2].x + laserCloud_->points[i + 3].x + laserCloud_->points[i + 4].x + laserCloud_->points[i + 5].x;
        float diffY = laserCloud_->points[i - 5].y + laserCloud_->points[i - 4].y + laserCloud_->points[i - 3].y + laserCloud_->points[i - 2].y + laserCloud_->points[i - 1].y + laserCloud_->points[i].y + laserCloud_->points[i + 1].y + laserCloud_->points[i + 2].y + laserCloud_->points[i + 3].y + laserCloud_->points[i + 4].y + laserCloud_->points[i + 5].y;
        float diffZ = laserCloud_->points[i - 5].z + laserCloud_->points[i - 4].z + laserCloud_->points[i - 3].z + laserCloud_->points[i - 2].z + laserCloud_->points[i - 1].z + laserCloud_->points[i].z + laserCloud_->points[i + 1].z + laserCloud_->points[i + 2].z + laserCloud_->points[i + 3].z + laserCloud_->points[i + 4].z + laserCloud_->points[i + 5].z;

        cloudCurvature_[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd_[i] = i;
        cloudNeighborPicked_[i] = 0;
        cloudLabel_[i] = 0;
    }

    // edge feature와 planar feature를 저장할 변수
    for(int i = 0; i < opt_.N_SCANS; i++) {
        if(scanEndInd[i] - scanStartInd[i] < 6) continue;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        // 논문에서 subregion을 4개로 분할한 것처럼, 본 코드는 6개의 subregion으로 분할한다.
        for(int j = 0; j < 6; j++) {
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            // input은 cloudSortInd_을 기준으로 한 주소이나, comp 함수에서 cloudCurvature_의 정보를 비교하기 때문에
            // 사실상 cloudCurvature_ index에 대한 정렬이다. 또한 현재 subregion에 대해서만 정렬
            std::sort(cloudSortInd_ + sp, cloudSortInd_ + ep + 1, comp);

            int largestPickedNum = 0;
            // 역순 = largest curvature부터 시작
            for(int k = ep; k >= sp; k--) {
                // 현재 검사중인 "point"의 index(범위: 5~cloudSize-6)
                // cloudCurvature_ 기준으로 정렬된 상태의 index이다.
                int ind = cloudSortInd_[k];

                // cloudCurvature_[ind]는 0.1을 기준으로 edge/planar point로 분류된다.
                if(cloudNeighborPicked_[ind] == 0 && cloudCurvature_[ind] > 0.1) {
                    // largestPickedNum의 의미는?
                    // cloudCurvature_[ind]가 큰 것을 pick할 때마다 올려주는 수
                    largestPickedNum++;
                    if(largestPickedNum >= 2) {
                        // label의 의미:
                        // 2: best quality edge point의 index
                        // 1: 그다음으로 quality 괜찮은 edge point의 index
                        // cornerPointsSharp: best quality edge point(cloudNeighborPicked_[ind] == 0을 합격한 상위 2개)
                        // cornerPointsLessSharp: best quality edge point+상위 18개(사실상 cloudNeighborPicked_[ind] == 0을 합격한 상위20개)
                        cloudLabel_[ind] = 2;
                        cornerPointsSharp_->push_back(laserCloud_->points[ind]);
                        cornerPointsLessSharp_->push_back(laserCloud_->points[ind]);
                    } else if(largestPickedNum <= 20) {
                        cloudLabel_[ind] = 1;
                        cornerPointsLessSharp_->push_back(laserCloud_->points[ind]);
                    } else {
                        break; // largestPickedNum은 20까지만 본다 => 상위 20개만 가지고 간다는 뜻
                    }

                    cloudNeighborPicked_[ind] = 1;
                    for(int l = 1; l <= 5; l++) {
                        float diffX = laserCloud_->points[ind + l].x - laserCloud_->points[ind + l - 1].x;
                        float diffY = laserCloud_->points[ind + l].y - laserCloud_->points[ind + l - 1].y;
                        float diffZ = laserCloud_->points[ind + l].z - laserCloud_->points[ind + l - 1].z;
                        if(diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                            break;
                        }
                        // diff의 합이 차이가 안나면 neighbor이 존재한다고 취급
                        // neighbor이 존재한다는 말은 feature pointcloud 그룹 안에 소속되었음을 의미?
                        // 어떠한 점과 neighbor 관계인지 알 수 있는 방법을 저장하지 않는데, 혹시 상관이 없는것인가?
                        // 추후 for문을 돌면서 cloudNeighborPicked_[ind] == 0 조건에 걸리게 된다.
                        cloudNeighborPicked_[ind + l] = 1;
                    }
                    for(int l = -1; l >= -5; l--) {
                        float diffX = laserCloud_->points[ind + l].x - laserCloud_->points[ind + l + 1].x;
                        float diffY = laserCloud_->points[ind + l].y - laserCloud_->points[ind + l + 1].y;
                        float diffZ = laserCloud_->points[ind + l].z - laserCloud_->points[ind + l + 1].z;
                        if(diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                            break;
                        }
                        cloudNeighborPicked_[ind + l] = 1;
                    }
                }
            }

            int smallestPickedNum = 0;
            for(int k = sp; k < ep; k++) {
                int ind = cloudSortInd_[k];

                if(cloudNeighborPicked_[ind] == 0 && cloudCurvature_[ind] < 0.1) {
                    // planar feature의 label은 -1
                    cloudLabel_[ind] = -1;
                    surfPointsFlat_->push_back(laserCloud_->points[ind]);

                    smallestPickedNum++;
                    if(smallestPickedNum <= 4) {
                        break;
                    }
                    cloudNeighborPicked_[ind] = 1;
                    for(int l = 1; l <= 5; l++) {
                        float diffX = laserCloud_->points[ind + l].x - laserCloud_->points[ind + l - 1].x;
                        float diffY = laserCloud_->points[ind + l].y - laserCloud_->points[ind + l - 1].y;
                        float diffZ = laserCloud_->points[ind + l].z - laserCloud_->points[ind + l - 1].z;
                        if(diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                            break;
                        }
                        cloudNeighborPicked_[ind + l] = 1;
                    }
                    for(int l = -1; l >= -5; l--) {
                        float diffX = laserCloud_->points[ind + l].x - laserCloud_->points[ind + l + 1].x;
                        float diffY = laserCloud_->points[ind + l].y - laserCloud_->points[ind + l + 1].y;
                        float diffZ = laserCloud_->points[ind + l].z - laserCloud_->points[ind + l + 1].z;
                        if(diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                            break;
                        }
                        cloudNeighborPicked_[ind + l] = 1;
                    }
                }
            }
            for(int k = sp; k <= ep; k++) {
                if(cloudLabel_[k] <= 0) {
                    // surfPointsFlat로 취급된 것 + corner,planar 검사를 거치지 않은 것 모두
                    // surfPointsLessFlatScan에 넣어준다?? 검사안거친건 왜?
                    surfPointsLessFlatScan->push_back(laserCloud_->points[k]);
                }
            }
        }
        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        *surfPointsLessFlat_ += surfPointsLessFlatScanDS;
    }
}