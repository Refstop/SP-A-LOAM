//
// Created by bhbhchoi on 23. 1. 31.
//
#include "laserOdometry/laserOdometry.h"

laserOdometry::laserOdometry(loOption option):
opt_(option),
q_w_curr_(1,0,0,0),
t_w_curr_(0, 0, 0),
q_last_curr_(Eigen::Map<Eigen::Quaterniond>(para_q_)),
t_last_curr_(Eigen::Map<Eigen::Vector3d>(para_t_)),
frameCount_(0) {}

void laserOdometry::setFullRes(double timeStamp, pcl::PointCloud<PointType> &fullRes) {
    mBuf_.lock();
    // std::queue push와 emplace
    // emplace는 push와 기능적으로 거의 동일하지만, 메모리 낭비를 막아준다는 장점을 갖고 있다.
    // push: 생성자 호출->생성자를 임시로 메모리에 저장->queue에 복사->임시로 생성자를 저장한 메모리 해제
    // emplace: 생성자 호출->생성자를 queue 메모리에 저장->끝
    // 참고: https://blog.junie.land/7
    fullPointsBuf_.emplace(PointTypeStamped::Ptr(
            new PointTypeStamped(timeStamp, fullRes)));
    mBuf_.unlock();
}

void laserOdometry::setCornerSharpBuf(double timeStamp, pcl::PointCloud<PointType> &cornerPointsSharp) {
    mBuf_.lock();
    cornerSharpBuf_.emplace(PointTypeStamped::Ptr(
            new PointTypeStamped(timeStamp, cornerPointsSharp)));
    mBuf_.unlock();
}

void laserOdometry::setCornerPointsLessSharp(double timeStamp, pcl::PointCloud<PointType> &cornerPointsLessSharp) {
    mBuf_.lock();
    cornerLessSharpBuf_.emplace(PointTypeStamped::Ptr(
            new PointTypeStamped(timeStamp, cornerPointsLessSharp)));
    mBuf_.unlock();
}

void laserOdometry::setSurfPointsFlat(double timeStamp, pcl::PointCloud<PointType> &surfPointsFlat)  {
    mBuf_.lock();
    surfFlatBuf_.emplace(PointTypeStamped::Ptr(
            new PointTypeStamped(timeStamp, surfPointsFlat)));
    mBuf_.unlock();
}

void laserOdometry::setSurfPointsLessFlat(double timeStamp, pcl::PointCloud<PointType> &surfPointsLessFlat) {
    mBuf_.lock();
    surfFlatLessBuf_.emplace(PointTypeStamped::Ptr(
            new PointTypeStamped(timeStamp, surfPointsLessFlat)));
    mBuf_.unlock();
}

double laserOdometry::pointDistance(const PointType point1, const PointType point2) {
    return (point1.x - point2.x) * (point1.x - point2.x) +
           (point1.y - point2.y) * (point1.y - point2.y) +
           (point1.z - point2.z) * (point1.z - point2.z);
}

void laserOdometry::findCornerIndices(const PointType pointSel, int &closestPointInd, int &minPointInd) {
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtreeCornerLast_->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

    if(pointSearchSqDis[0] < opt_.DISTANCE_SQ_THRESHOLD) {
        closestPointInd = static_cast<int>(pointSearchSqDis[0]);
        int closestPointScanID = int(laserCloudCornerLast_->laserCloud.points[closestPointInd].intensity);

        double minPointSqDis2 = opt_.DISTANCE_SQ_THRESHOLD;
        for(int j = closestPointInd + 1; j < (int)laserCloudCornerLast_->laserCloud.points.size(); ++j) {
            if(laserCloudCornerLast_->laserCloud.points[j].intensity <= closestPointScanID) continue;
            if(laserCloudCornerLast_->laserCloud.points[j].intensity > closestPointScanID + opt_.NEARBY_SCAN) break;
            double pointSqDis = pointDistance(laserCloudCornerLast_->laserCloud.points[j], pointSel);
            if(pointSqDis < minPointSqDis2) {
                minPointSqDis2 = pointSqDis;
                minPointInd = j;
            }
        }
        for(int j = closestPointInd - 1; j >= 0; --j) {
            if(laserCloudCornerLast_->laserCloud.points[j].intensity >= closestPointScanID) continue;
            if(laserCloudCornerLast_->laserCloud.points[j].intensity < closestPointScanID + opt_.NEARBY_SCAN) break;
            double pointSqDis = pointDistance(laserCloudCornerLast_->laserCloud.points[j], pointSel);
            if(pointSqDis < minPointSqDis2) {
                minPointSqDis2 = pointSqDis;
                minPointInd = j;
            }
        }
    }
}

void laserOdometry::findSurfIndices(const PointType pointSel, int &closestPointInd, int &minPointInd1, int &minPointInd2) {
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtreeSurfLast_->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
    if(pointSearchSqDis[0] < opt_.DISTANCE_SQ_THRESHOLD) {
        closestPointInd = pointSearchInd[0];

        int closestPointScanID = int(laserCloudSurfLast_->laserCloud.points[closestPointInd].intensity);
        double minPointSqDis1 = opt_.DISTANCE_SQ_THRESHOLD, minPointSqDis2 = opt_.DISTANCE_SQ_THRESHOLD;

        for(int j = closestPointInd + 1; j < laserCloudSurfLast_->laserCloud.points.size(); ++j) {
            if(laserCloudSurfLast_->laserCloud.points[j].intensity > closestPointScanID + opt_.NEARBY_SCAN) break;
            double pointSqDis = pointDistance(laserCloudSurfLast_->laserCloud.points[j], pointSel);

            if(laserCloudSurfLast_->laserCloud.points[j].intensity <= closestPointScanID && pointSqDis < minPointSqDis1) {
                // if in the same or lower scan line
                minPointSqDis1 = pointSqDis;
                minPointInd1 = j;
            } else if(laserCloudSurfLast_->laserCloud.points[j].intensity > closestPointScanID && pointSqDis < minPointSqDis2) {
                // if in the higher scan line
                minPointSqDis2 = pointSqDis;
                minPointInd2 = j;
            }
        }
        for(int j = closestPointInd - 1; j >= 0; --j) {
            if(laserCloudSurfLast_->laserCloud.points[j].intensity < closestPointScanID - opt_.NEARBY_SCAN) break;
            double pointSqDis = pointDistance(laserCloudSurfLast_->laserCloud.points[j], pointSel);

            if(laserCloudSurfLast_->laserCloud.points[j].intensity >= closestPointScanID && pointSqDis < minPointSqDis1) {
                // if in the same or lower scan line
                minPointSqDis1 = pointSqDis;
                minPointInd1 = j;
            } else if(laserCloudSurfLast_->laserCloud.points[j].intensity < closestPointScanID && pointSqDis < minPointSqDis2) {
                // if in the higher scan line
                minPointSqDis2 = pointSqDis;
                minPointInd2 = j;
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
        PointTypeStamped::Ptr cornerPointsSharp = cornerSharpBuf_.front(),
                              cornerPointsLessSharp = cornerLessSharpBuf_.front(),
                              surfPointsFlat = surfFlatBuf_.front(),
                              surfPointsLessFlat = surfFlatLessBuf_.front();
        laserCloudFullRes_ = fullPointsBuf_.front();
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
            int cornerPointsSharpNum = static_cast<int>(cornerPointsSharp->laserCloud.points.size());
            int surfPointsFlatNum = static_cast<int>(surfPointsFlat->laserCloud.points.size());

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

                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;

                // corner matching & optimize for Rt
                for(int i = 0; i < cornerPointsSharpNum; ++i) {
                    int closestPointInd = -1, minPointInd2 = -1;
                    // input: const PointType cornerPointsSharp.laserCloud.points[i]
                    // output: int &closestPointInd, int &minPointInd2
                    findCornerIndices(cornerPointsSharp->laserCloud.points[i], closestPointInd, minPointInd2);

                    if(minPointInd2 >= 0) {
                        Eigen::Vector3d curr_point(cornerPointsSharp->laserCloud.points[i].x,
                                                   cornerPointsSharp->laserCloud.points[i].y,
                                                   cornerPointsSharp->laserCloud.points[i].z);
                        Eigen::Vector3d last_point_a(laserCloudCornerLast_->laserCloud.points[closestPointInd].x,
                                                     laserCloudCornerLast_->laserCloud.points[closestPointInd].y,
                                                     laserCloudCornerLast_->laserCloud.points[closestPointInd].z);
                        Eigen::Vector3d last_point_b(laserCloudCornerLast_->laserCloud.points[minPointInd2].x,
                                                     laserCloudCornerLast_->laserCloud.points[minPointInd2].y,
                                                     laserCloudCornerLast_->laserCloud.points[minPointInd2].z);

                        // cost function과 loss function의 차이점
                        // loss function은 outlier의 영향을 제거하기 위한 함수이다.
                        // 너무 큰 residual은 cost function에서 배제하는 방식으로 작동한다.
                        ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b);
                        problem.AddResidualBlock(cost_function, loss_function, para_q_, para_t_);
                        corner_correspondence_++;
                    }
                }

                for(int i = 0; i < surfPointsFlatNum; ++i) {
                    int closestPointInd = -1, minPointInd1 = -1, minPointInd2 = -1;
                    findSurfIndices(surfPointsFlat->laserCloud.points[i], closestPointInd, minPointInd1, minPointInd2);

                    if(minPointInd1 >= 0 && minPointInd2 >= 0) {
                        Eigen::Vector3d curr_point(surfPointsFlat->laserCloud.points[i].x,
                                                   surfPointsFlat->laserCloud.points[i].y,
                                                   surfPointsFlat->laserCloud.points[i].z);
                        Eigen::Vector3d last_point_a(surfPointsFlat->laserCloud.points[closestPointInd].x,
                                                     surfPointsFlat->laserCloud.points[closestPointInd].y,
                                                     surfPointsFlat->laserCloud.points[closestPointInd].z);
                        Eigen::Vector3d last_point_b(surfPointsFlat->laserCloud.points[minPointInd1].x,
                                                     surfPointsFlat->laserCloud.points[minPointInd1].y,
                                                     surfPointsFlat->laserCloud.points[minPointInd1].z);
                        Eigen::Vector3d last_point_c(surfPointsFlat->laserCloud.points[minPointInd2].x,
                                                     surfPointsFlat->laserCloud.points[minPointInd2].y,
                                                     surfPointsFlat->laserCloud.points[minPointInd2].z);

                        ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c);
                        problem.AddResidualBlock(cost_function, loss_function, para_q_, para_t_);
                        plane_correspondence_++;
                    }
                }
                if(opt_.verbose) cout << "corner_correspondence" << corner_correspondence_ << ", " << "plane_correspondence" << plane_correspondence_ << "\n";
                if((corner_correspondence_ + plane_correspondence_) < 10 && opt_.verbose) cout << "less correspondence! *************************************************\n";

                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4;
                options.minimizer_progress_to_stdout = false;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);

                // (추정) 나온 결과가 q_last_curr_과 t_last_curr_에 연결되어 있으므로 그대로 사용한다.
                t_w_curr_ = t_w_curr_ + q_w_curr_ * t_last_curr_;
                q_w_curr_ = q_w_curr_ * q_last_curr_;
            }

            PointTypeStamped::Ptr laserCloudTemp = cornerPointsLessSharp;
            cornerPointsSharp = laserCloudCornerLast_;
            laserCloudCornerLast_ = laserCloudTemp;

            laserCloudTemp = surfPointsFlat;
            surfPointsLessFlat = laserCloudSurfLast_;
            laserCloudSurfLast_ = laserCloudTemp;

            laserCloudCornerLastNum_ = static_cast<int>(laserCloudCornerLast_->laserCloud.points.size());
            laserCloudSurfLastNum_ = static_cast<int>(laserCloudSurfLast_->laserCloud.points.size());

            pcl::PointCloud<PointType>::Ptr kdCornerInput(laserCloudCornerLast_, &laserCloudCornerLast_->laserCloud);
            pcl::PointCloud<PointType>::Ptr kdSurfInput(laserCloudSurfLast_, &laserCloudSurfLast_->laserCloud);

            kdtreeCornerLast_->setInputCloud(kdCornerInput);
            kdtreeSurfLast_->setInputCloud(kdSurfInput);
        }
        return true;
    } else {
        return false;
    }
}
