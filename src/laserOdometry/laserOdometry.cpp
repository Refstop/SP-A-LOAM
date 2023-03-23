//
// Created by bhbhchoi on 23. 1. 31.
//
#include "laserOdometry/laserOdometry.h"
#include "tic_toc.h"

laserOdometry::laserOdometry(loOption option):
opt_(option),
q_w_curr_(1,0,0,0),
t_w_curr_(0,0,0),
q_last_curr_(para_q_),
t_last_curr_(para_t_),
kdtreeCornerLast_(new pcl::KdTreeFLANN<PointType>()),
kdtreeSurfLast_(new pcl::KdTreeFLANN<PointType>()) {}

void laserOdometry::setFullRes(double timeStamp, pcl::PointCloud<PointType> &fullRes) {
    mBuf_.lock();
    // std::queue push와 emplace
    // emplace는 push와 기능적으로 거의 동일하지만, 메모리 낭비를 막아준다는 장점을 갖고 있다.
    // push: 생성자 호출->생성자를 임시로 메모리에 저장->queue에 복사->임시로 생성자를 저장한 메모리 해제
    // emplace: 생성자 호출->생성자를 queue 메모리에 저장->끝
    // 참고: https://blog.junie.land/7
    fullPointsBuf_.emplace(PointCloudStamped::Ptr(
            new PointCloudStamped(timeStamp, fullRes)));
    mBuf_.unlock();
}

void laserOdometry::setCornerSharpBuf(double timeStamp, pcl::PointCloud<PointType> &cornerPointsSharp) {
    mBuf_.lock();
    cornerSharpBuf_.emplace(PointCloudStamped::Ptr(
            new PointCloudStamped(timeStamp, cornerPointsSharp)));
    mBuf_.unlock();
}

void laserOdometry::setCornerPointsLessSharp(double timeStamp, pcl::PointCloud<PointType> &cornerPointsLessSharp) {
    mBuf_.lock();
    cornerLessSharpBuf_.emplace(PointCloudStamped::Ptr(
            new PointCloudStamped(timeStamp, cornerPointsLessSharp)));
    mBuf_.unlock();
}

void laserOdometry::setSurfPointsFlat(double timeStamp, pcl::PointCloud<PointType> &surfPointsFlat)  {
    mBuf_.lock();
    surfFlatBuf_.emplace(PointCloudStamped::Ptr(
            new PointCloudStamped(timeStamp, surfPointsFlat)));
    mBuf_.unlock();
}

void laserOdometry::setSurfPointsLessFlat(double timeStamp, pcl::PointCloud<PointType> &surfPointsLessFlat) {
    mBuf_.lock();
    surfFlatLessBuf_.emplace(PointCloudStamped::Ptr(
            new PointCloudStamped(timeStamp, surfPointsLessFlat)));
    mBuf_.unlock();
}

/**********************************
transformToStart
Purpose : LiDAR raw data의 distortion을 없애기 위한 함수, 본 코드에서는 Distortion이 없으므로 pi와 po가 같다.
Input   : PointType const *const pi
Output  : PointType *const po
Flow    : 
Note    : 
	- PointType const *const : 상수 데이터(const PointType과 동일), 상수 포인터
	- PointType *const : 비상수 데이터, 상수 포인터
	- 결론: pi는 안건드림, po는 데이터 주입만
	- 참고: https://programmerpsy.tistory.com/64
    - const int* ptr와 int *const ptr의 차이
        - const int* ptr : 상수 변수에 대한 포인터(*ptr=6 허용 안됨, ptr=&a 허용)
        - int *const ptr : 상수 포인터(*ptr=6 허용, ptr=&a 허용 안됨)
**********************************/
void laserOdometry::transformToStart(PointType const *const pi, PointType *const po) {
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_last_curr_ * point + t_last_curr_;

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}

double laserOdometry::pointDistance(const PointType point1, const PointType point2) {
    return (point1.x - point2.x) * (point1.x - point2.x) +
           (point1.y - point2.y) * (point1.y - point2.y) +
           (point1.z - point2.z) * (point1.z - point2.z);
}

void laserOdometry::findCornerIndices(PointType const *const pi, int &closestPointInd, int &minPointInd2) {
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtreeCornerLast_->nearestKSearch(*pi, 1, pointSearchInd, pointSearchSqDis);

    if(pointSearchSqDis[0] < opt_.DISTANCE_SQ_THRESHOLD) {
        closestPointInd = pointSearchInd[0];
        // 가장 가까운 point의 위치를 특정, intensity로부터 channel만을 가져옴
        int closestPointScanID = static_cast<int>(laserCloudCornerLast_->laserCloud.points[closestPointInd].intensity);

        double minPointSqDis2 = opt_.DISTANCE_SQ_THRESHOLD;
        // search in the direction of increasing scan line
        // pointSel과 가까운 점이 속한 채널을 기준으로 minPointSqDis2, minPointInd2을 계산한다.
        for(int j = closestPointInd + 1; j < (int)laserCloudCornerLast_->laserCloud.points.size(); ++j) {
            // if in the same scan line, continue
            // pointSel과 가까운 점이 속한 채널보다 밑에 있으면 컨티뉴
            // 일단 아래쪽 point는 신경안씀
            if(int(laserCloudCornerLast_->laserCloud.points[j].intensity) <= closestPointScanID) continue;

            // if not in nearby scans, end the loop
            // pointSel과 가까운 점이 속한 채널의 주변 채널에 있는게 아니면 브레이크
            // 일단 위쪽 2개 안에 있는지만 검사
            if(int(laserCloudCornerLast_->laserCloud.points[j].intensity) > (closestPointScanID + opt_.NEARBY_SCAN)) break;

            // 점 간의 거리 계산
            double pointSqDis = pointDistance(laserCloudCornerLast_->laserCloud.points[j], *pi);

            // minPointSqDis2보다 작으면 최소거리값으로 결정(최소값 구하는 알고리즘과 동일)
            if(pointSqDis < minPointSqDis2) {
                // find nearer point
                minPointSqDis2 = pointSqDis;
                minPointInd2 = j;
            }
        }

        // search in the direction of decreasing scan line
        for(int j = closestPointInd - 1; j >= 0; --j) {
            // if in the same scan line, continue
            // 위쪽은 아까 했으니 신경안씀
            if(int(laserCloudCornerLast_->laserCloud.points[j].intensity) >= closestPointScanID) continue;

            // if not in nearby scans, end the loop
            // 아래쪽 2개 안에 있는지 검사
            if(int(laserCloudCornerLast_->laserCloud.points[j].intensity) < (closestPointScanID - opt_.NEARBY_SCAN)) break;

            double pointSqDis = pointDistance(laserCloudCornerLast_->laserCloud.points[j], *pi);

            if(pointSqDis < minPointSqDis2) {
                // find nearer point
                minPointSqDis2 = pointSqDis;
                minPointInd2 = j;
            }
        }
    }
}

void laserOdometry::findSurfIndices(PointType const *const pi, int &closestPointInd, int &minPointInd2, int &minPointInd3) {
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtreeSurfLast_->nearestKSearch(*pi, 1, pointSearchInd, pointSearchSqDis);
    if(pointSearchSqDis[0] < opt_.DISTANCE_SQ_THRESHOLD) {
        closestPointInd = pointSearchInd[0];

        int closestPointScanID = int(laserCloudSurfLast_->laserCloud.points[closestPointInd].intensity);
        double minPointSqDis1 = opt_.DISTANCE_SQ_THRESHOLD, minPointSqDis2 = opt_.DISTANCE_SQ_THRESHOLD;

        for(int j = closestPointInd + 1; j < laserCloudSurfLast_->laserCloud.points.size(); ++j) {
            if(laserCloudSurfLast_->laserCloud.points[j].intensity > (closestPointScanID + opt_.NEARBY_SCAN)) break;
            double pointSqDis = pointDistance(laserCloudSurfLast_->laserCloud.points[j], *pi);

            if(laserCloudSurfLast_->laserCloud.points[j].intensity <= closestPointScanID && pointSqDis < minPointSqDis1) {
                // if in the same or lower scan line
                minPointSqDis1 = pointSqDis;
                minPointInd2 = j;
            } else if(laserCloudSurfLast_->laserCloud.points[j].intensity > closestPointScanID && pointSqDis < minPointSqDis2) {
                // if in the higher scan line
                minPointSqDis2 = pointSqDis;
                minPointInd3 = j;
            }
        }
        
        for(int j = closestPointInd - 1; j >= 0; --j) {
            if(laserCloudSurfLast_->laserCloud.points[j].intensity < closestPointScanID - opt_.NEARBY_SCAN) break;
            double pointSqDis = pointDistance(laserCloudSurfLast_->laserCloud.points[j], *pi);

            if(laserCloudSurfLast_->laserCloud.points[j].intensity >= closestPointScanID && pointSqDis < minPointSqDis1) {
                // if in the same or lower scan line
                minPointSqDis1 = pointSqDis;
                minPointInd2 = j;
            } else if(laserCloudSurfLast_->laserCloud.points[j].intensity < closestPointScanID && pointSqDis < minPointSqDis2) {
                // if in the higher scan line
                minPointSqDis2 = pointSqDis;
                minPointInd3 = j;
            }
        }
    }
}

bool laserOdometry::run() {
    if(!cornerSharpBuf_.empty() && !cornerLessSharpBuf_.empty() &&
       !surfFlatBuf_.empty() && !surfFlatLessBuf_.empty() && !fullPointsBuf_.empty()) {
        
        if(cornerSharpBuf_.front()->timeStamp != fullPointsBuf_.front()->timeStamp ||
           cornerLessSharpBuf_.front()->timeStamp != fullPointsBuf_.front()->timeStamp ||
           surfFlatBuf_.front()->timeStamp != fullPointsBuf_.front()->timeStamp ||
           surfFlatLessBuf_.front()->timeStamp != fullPointsBuf_.front()->timeStamp) {
            if(opt_.verbose) cout << "unsync message!";
            return false;
        }

        // 소멸자는 객체의 수명이 끝났을 때 호출되고, 메모리 할당 해제를 의미하지는 않는다.
        // 소멸자가 호출되더라도 객체 내에서 메모리를 할당받았지만 해제하지 않은 경우 메모리 누수가 발생한다.
        // 메모리 할당 해제는 많은 방법이 있겠지만, 일반적으로 많이 사용하는 방식은 delete나 스마트 포인터이다.
        // mBuf_...의 필요 위..치?
        mBuf_.lock();
        PointCloudStamped::Ptr cornerPointsSharp = cornerSharpBuf_.front(),
                              cornerPointsLessSharp = cornerLessSharpBuf_.front(),
                              surfPointsFlat = surfFlatBuf_.front(),
                              surfPointsLessFlat = surfFlatLessBuf_.front();
        laserCloudFullRes_ = fullPointsBuf_.front();
        // cout << std::fixed;
        // cout.precision(20);
        // cout << "SPtimeLaserCloudFullRes: " << laserCloudFullRes_->timeStamp << endl;
        // cout << "SStimeLaserCloudFullRes: " << surfPointsLessFlat->timeStamp << endl;
        cornerSharpBuf_.pop();
        cornerLessSharpBuf_.pop();
        surfFlatBuf_.pop();
        surfFlatLessBuf_.pop();
        fullPointsBuf_.pop();
        mBuf_.unlock();

        // initializing
        if(!opt_.systemInited) {
            opt_.systemInited = true; // 왜 있는거지? 바로 시작하면 안되나?
            if(opt_.verbose) cout << "Initialization finished \n";
        } else {
            TicTocV2 t_opt(true);
            int cornerPointsSharpNum = cornerPointsSharp->laserCloud.points.size();
            int surfPointsFlatNum = surfPointsFlat->laserCloud.points.size();

            for(size_t opti_counter = 0; opti_counter < 2; ++opti_counter) {
                corner_correspondence_ = 0;
                plane_correspondence_ = 0;

                // 메모리 누수 문제 없나??
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::LocalParameterization * q_parameterization =
                        new ceres::EigenQuaternionParameterization();
                ceres::Problem::Options problem_options;


                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(para_q_, 4, q_parameterization);
                problem.AddParameterBlock(para_t_, 3);

                pcl::PointXYZI pointSel;
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;
                
                // corner matching & optimize for Rt
                for(int i = 0; i < cornerPointsSharpNum; ++i) {
                    int closestPointInd = -1, minPointInd2 = -1;
                    transformToStart(&(cornerPointsSharp->laserCloud.points[i]), &pointSel);
                    findCornerIndices(&pointSel, closestPointInd, minPointInd2);
                    if(minPointInd2 >= 0) {
                        Eigen::Vector3d curr_point(cornerPointsSharp->laserCloud.points[i].x,
                                                   cornerPointsSharp->laserCloud.points[i].y,
                                                   cornerPointsSharp->laserCloud.points[i].z);
                        // KDTree의 결과로 제일 가깝다고 판단된 point의 laserCloudCornerLast에서의 index
                        Eigen::Vector3d last_point_a(laserCloudCornerLast_->laserCloud.points[closestPointInd].x,
                                                     laserCloudCornerLast_->laserCloud.points[closestPointInd].y,
                                                     laserCloudCornerLast_->laserCloud.points[closestPointInd].z);
                        // closestPointInd point와 다른 channel에 있는 가장 가까운 point의 index
                        Eigen::Vector3d last_point_b(laserCloudCornerLast_->laserCloud.points[minPointInd2].x,
                                                     laserCloudCornerLast_->laserCloud.points[minPointInd2].y,
                                                     laserCloudCornerLast_->laserCloud.points[minPointInd2].z);

                        // cost function과 loss function의 차이점?
                        ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b);
                        problem.AddResidualBlock(cost_function, loss_function, para_q_, para_t_);
                        corner_correspondence_++;
                    }
                }
                
                // planar feature 추출
                for(int i = 0; i < surfPointsFlatNum; ++i) {
                    transformToStart(&(surfPointsFlat->laserCloud.points[i]), &pointSel);
                    // if(i < 15) {
                    //     cout << "SPpointSel: " << pointSel.x << " " << pointSel.y << " " << pointSel.z << "\n";
                    // }
                    kdtreeSurfLast_->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                    int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                    if(pointSearchSqDis[0] < opt_.DISTANCE_SQ_THRESHOLD) {
                        closestPointInd = pointSearchInd[0];

                        // get closest point's scan ID
                        int closestPointScanID = int(laserCloudSurfLast_->laserCloud.points[closestPointInd].intensity);
                        double minPointSqDis2 = opt_.DISTANCE_SQ_THRESHOLD, minPointSqDis3 = opt_.DISTANCE_SQ_THRESHOLD;

                        // search in the direction of increasing scan line
                        for(int j = closestPointInd + 1; j < (int)laserCloudSurfLast_->laserCloud.points.size(); ++j) {
                            // if not in nearby scans, end the loop
                            if(int(laserCloudSurfLast_->laserCloud.points[j].intensity) > (closestPointScanID + opt_.NEARBY_SCAN)) break;

                            double pointSqDis = pointDistance(laserCloudSurfLast_->laserCloud.points[j], pointSel);
                            
                            if(int(laserCloudSurfLast_->laserCloud.points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2) {
                                // if in the same or lower scan line
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            } else if(int(laserCloudSurfLast_->laserCloud.points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3) {
                                // if in the higher scan line
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }

                        // search in the direction of decreasing scan line
                        for(int j = closestPointInd - 1; j >= 0; --j) {
                            // if not in nearby scans, end the loop
                            if(int(laserCloudSurfLast_->laserCloud.points[j].intensity) < (closestPointScanID - opt_.NEARBY_SCAN)) break;

                            double pointSqDis = pointDistance(laserCloudSurfLast_->laserCloud.points[j], pointSel);

                            // if in the same or higher scan line
                            if(int(laserCloudSurfLast_->laserCloud.points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2) {
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            } else if(int(laserCloudSurfLast_->laserCloud.points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3) {
                                // find nearer point
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }

                        if(minPointInd2 >= 0 && minPointInd3 >= 0) {
                            Eigen::Vector3d curr_point(surfPointsFlat->laserCloud.points[i].x,
                                                       surfPointsFlat->laserCloud.points[i].y,
                                                       surfPointsFlat->laserCloud.points[i].z);
                            Eigen::Vector3d last_point_a(laserCloudSurfLast_->laserCloud.points[closestPointInd].x,
                                                         laserCloudSurfLast_->laserCloud.points[closestPointInd].y,
                                                         laserCloudSurfLast_->laserCloud.points[closestPointInd].z);
                            Eigen::Vector3d last_point_b(laserCloudSurfLast_->laserCloud.points[minPointInd2].x,
                                                         laserCloudSurfLast_->laserCloud.points[minPointInd2].y,
                                                         laserCloudSurfLast_->laserCloud.points[minPointInd2].z);
                            Eigen::Vector3d last_point_c(laserCloudSurfLast_->laserCloud.points[minPointInd3].x,
                                                         laserCloudSurfLast_->laserCloud.points[minPointInd3].y,
                                                         laserCloudSurfLast_->laserCloud.points[minPointInd3].z);
                            
                            ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c);
                            problem.AddResidualBlock(cost_function, loss_function, para_q_, para_t_);
                            plane_correspondence_++;
                        }
                    }
                }
                
                if(opt_.verbose) cout << "corner_correspondence " << corner_correspondence_ << ", " << "plane_correspondence " << plane_correspondence_ << "\n";
                if((corner_correspondence_ + plane_correspondence_) < 10 && opt_.verbose) cout << "less correspondence! *************************************************\n";

                TicTocV2 t_solver(true);
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4;
                options.minimizer_progress_to_stdout = false;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
                // t_solver.toc("SPSolver time ");
            }
            cout << "SPcorner_correspondence " << corner_correspondence_ << ", " << "plane_correspondence " << plane_correspondence_ << "\n";
            // t_opt.toc("SPopt twice");

            // (추정) 나온 결과가 q_last_curr_과 t_last_curr_에 연결되어 있으므로 그대로 사용한다.
            t_w_curr_ = t_w_curr_ + q_w_curr_ * t_last_curr_;
            q_w_curr_ = q_w_curr_ * q_last_curr_;
        }
        PointCloudStamped::Ptr laserCloudTemp = cornerPointsLessSharp;
        cornerPointsSharp = laserCloudCornerLast_;
        laserCloudCornerLast_ = laserCloudTemp;

        laserCloudTemp = surfPointsFlat;
        surfPointsLessFlat = laserCloudSurfLast_;
        laserCloudSurfLast_ = laserCloudTemp;
        
        // laserCloudCornerLast_->laserCloud의 주소를 바로 kdCornerInput에 할당해버리면
        // 이후 스마트 포인터인 kdCornerInput이 할당 해제될 때 laserCloudCornerLast_->laserCloud까지 할당 해제되어 버린다.
        // 따라서 이 부분에서는 주소를 이어주지 않고 새로운 pcl::PointCloud<PointType> 객체를 만들어 새 주소를 생성해주고
        // laserCloudCornerLast_->laserCloud를 복사해서 사용한다.
        pcl::PointCloud<PointType>::Ptr kdCornerInput(new pcl::PointCloud<PointType>(laserCloudCornerLast_->laserCloud));
        pcl::PointCloud<PointType>::Ptr kdSurfInput(new pcl::PointCloud<PointType>(laserCloudSurfLast_->laserCloud));
        
        kdtreeCornerLast_->setInputCloud(kdCornerInput);
        kdtreeSurfLast_->setInputCloud(kdSurfInput);

        return true;
    } else {
        return false;
    }
}
