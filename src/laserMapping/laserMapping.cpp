#include "laserMapping/laserMapping.h"

laserMapping::laserMapping(lmOption option):
    opt_(option),
    q_w_curr_(parameters_),
    // 주소 parameters로부터 double 크기로 4만큼 떨어진 곳의 주소를 생성자 파라미터로 준다.
    t_w_curr_(parameters_ + 4), // parameters의 [4], [5], [6]을 의미
    q_wmap_wodom_(1, 0, 0, 0),
    t_wmap_wodom_(0, 0, 0),
    q_wodom_curr_(1, 0, 0, 0),
    t_wodom_curr_(0, 0, 0),
    kdtreeCornerFromMap_(new pcl::KdTreeFLANN<PointType>()),
    kdtreeSurfFromMap_(new pcl::KdTreeFLANN<PointType>()),
    laserCloudCornerArray_(new pcl::PointCloud<PointType>::Ptr[opt_.laserCloudNum]),
    laserCloudSurfArray_(new pcl::PointCloud<PointType>::Ptr[opt_.laserCloudNum]),
    downSizeFilterCorner_(),
    downSizeFilterSurf_(),
    laserCloudCornerFromMap_(new pcl::PointCloud<PointType>()),
    laserCloudSurfFromMap_(new pcl::PointCloud<PointType>()) {
    downSizeFilterCorner_.setLeafSize(opt_.lineRes, opt_.lineRes, opt_.lineRes);
    downSizeFilterSurf_.setLeafSize(opt_.planeRes, opt_.planeRes, opt_.planeRes);

    cout << opt_.laserCloudNum << endl;
    for(int i = 0; i < opt_.laserCloudNum; i++) {
        laserCloudCornerArray_[i].reset(new pcl::PointCloud<PointType>());
        laserCloudSurfArray_[i].reset(new pcl::PointCloud<PointType>());
    }
    cout << laserCloudCornerArray_[15]->points.size() << endl;
    cout << laserCloudSurfArray_[15]->points.size() << endl;
}

laserMapping::~laserMapping() {
    delete[] laserCloudCornerArray_;
    delete[] laserCloudSurfArray_;
}

void laserMapping::setCornerLast(const double timeStamp, pcl::PointCloud<PointType> laserCloud) {
    mBuf_.lock();
    cornerLastBuf_.emplace(PointCloudStamped::Ptr(new PointCloudStamped(timeStamp, laserCloud)));
    mBuf_.unlock();
}
void laserMapping::setSurfLast(const double timeStamp, pcl::PointCloud<PointType> laserCloud) {
    mBuf_.lock();
    surfLastBuf_.emplace(PointCloudStamped::Ptr(new PointCloudStamped(timeStamp, laserCloud)));
    mBuf_.unlock();
}
void laserMapping::setFullRes(const double timeStamp, pcl::PointCloud<PointType> laserCloud) {
    mBuf_.lock();
    fullResBuf_.emplace(PointCloudStamped::Ptr(new PointCloudStamped(timeStamp, laserCloud)));
    mBuf_.unlock();
}
void laserMapping::setOdometry(const double timeStamp, Eigen::Quaterniond q_wodom_curr, Eigen::Vector3d t_wodom_curr,
                                                       Eigen::Quaterniond &q_w_curr, Eigen::Vector3d &t_w_curr) {
    mBuf_.lock();
    odometryBuf_.emplace(OdometryStamped::Ptr(new OdometryStamped(timeStamp, q_wodom_curr, t_wodom_curr)));
    mBuf_.unlock();

    q_w_curr = q_wmap_wodom_ * q_wodom_curr;
    t_w_curr = q_wmap_wodom_ * t_wodom_curr + t_wmap_wodom_;
}

PointCloudStamped::Ptr laserMapping::getLaserCloudFullResGlobal() {
    int laserCloudFullResNum = laserCloudFullRes_->laserCloud.points.size();
    for(int i = 0; i < laserCloudFullResNum; i++) {
        pointAssociateToMap(&laserCloudFullRes_->laserCloud.points[i], &laserCloudFullRes_->laserCloud.points[i]);
    }

    return laserCloudFullRes_;
}

PointCloudStamped::Ptr laserMapping::getLaserCloudSurround() {
    PointCloudStamped::Ptr laserCloudSurround = PointCloudStamped::Ptr(new PointCloudStamped());
    laserCloudSurround->timeStamp = odometryLast_->timeStamp;
    for(int i = 0; i < laserCloudSurroundNum_; i++) {
        int ind = laserCloudSurroundInd_[i];
        laserCloudSurround->laserCloud += *laserCloudCornerArray_[ind];
        laserCloudSurround->laserCloud += *laserCloudSurfArray_[ind];
    }

    return laserCloudSurround;
}

PointCloudStamped::Ptr laserMapping::getLaserCloudMap() {
    PointCloudStamped::Ptr laserCloudMap = PointCloudStamped::Ptr(new PointCloudStamped());
    laserCloudMap->timeStamp = odometryLast_->timeStamp;
    for(int i = 0; i < opt_.laserCloudNum; i++) {
        laserCloudMap->laserCloud += *laserCloudCornerArray_[i];
        laserCloudMap->laserCloud += *laserCloudSurfArray_[i];
    }

    return laserCloudMap;
}

/**********************************
transformAssociateToMap
Purpose : world2laser(curr) 초기 추정값 설정
Input   :
	- Eigen::Quaterniond q_wmap_wodom
	- Eigen::Quaterniond q_wodom_curr
	- Eigen::Quaterniond q_wmap_wodom
	- Eigen::Vector3d t_wodom_curr
	- Eigen::Vector3d t_wmap_wodom
Output  : 
	- Eigen::Map<Eigen::Quaterniond> q_w_curr
	- Eigen::Map<Eigen::Vector3d> t_w_curr
Flow    : 
Note    :
**********************************/
void laserMapping::transformAssociateToMap() {
    q_w_curr_ = q_wmap_wodom_ * q_wodom_curr_;
    t_w_curr_ = q_wmap_wodom_ * t_wodom_curr_ + t_wmap_wodom_;
}

/**********************************
pointAssociateToMap
Purpose : Point를 Map에 합칠 수 있도록 world 좌표계 기준으로 변환하는 함수
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
void laserMapping::pointAssociateToMap(PointType const *const pi, PointType *const po) {
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr_ * point_curr + t_w_curr_;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
}

/**********************************
transformUpdate
Purpose : map2odom 추정값 계산
Input   :
	- Eigen::Map<Eigen::Quaterniond> q_w_curr_
	- Eigen::Quaterniond q_wodom_curr_
	- Eigen::Map<Eigen::Vector3d> t_w_curr_
	- Eigen::Quaterniond q_wmap_wodom_
	- Eigen::Vector3d t_wodom_curr_
Output  :
	- Eigen::Quaterniond q_wmap_wodom_
	- Eigen::Vector3d t_wmap_wodom_
Flow    : 
Note    : Localization을 위한 map2odom transform을 계산하는 것으로 추측
**********************************/
void laserMapping::transformUpdate() {
    q_wmap_wodom_ = q_w_curr_ * q_wodom_curr_.inverse();
    t_wmap_wodom_ = t_w_curr_ - q_wmap_wodom_ * t_wodom_curr_;
}

int laserMapping::cube(int i, int j, int k) {
    return i + opt_.laserCloudWidth * j + opt_.laserCloudWidth * opt_.laserCloudHeight * k;
}

bool laserMapping::run() {

    bool successed = false;
    while(!cornerLastBuf_.empty() && !surfLastBuf_.empty() &&
          !fullResBuf_.empty() && !odometryBuf_.empty()) {
        
        // cornlastbuf의 front의 시간을 기준으로 일찍 들어온 odometryBuf, surfLastBuf, fullResBuf의 요소들은 다 컷
        mBuf_.lock();
        while(!odometryBuf_.empty() && odometryBuf_.front()->timeStamp < cornerLastBuf_.front()->timeStamp) {
            odometryBuf_.pop();
        }
        if(odometryBuf_.empty()) {
            mBuf_.unlock();
            break;
        }
        
        while(!surfLastBuf_.empty() && surfLastBuf_.front()->timeStamp < cornerLastBuf_.front()->timeStamp) {
            surfLastBuf_.pop();
        }
        if(surfLastBuf_.empty()) {
            mBuf_.unlock();
            break;
        }
        
        while(!fullResBuf_.empty() && fullResBuf_.front()->timeStamp < cornerLastBuf_.front()->timeStamp) {
            fullResBuf_.pop();
        }
        if(fullResBuf_.empty()) {
            mBuf_.unlock();
            break;
        }
        
        // timeLaserOdometry의 시작시간과 같지 않으면 시간 동기화가 안되었음을 의미하며, 처리하지 않는다.
        if(cornerLastBuf_.front()->timeStamp != odometryBuf_.front()->timeStamp ||
           surfLastBuf_.front()->timeStamp != odometryBuf_.front()->timeStamp ||
           fullResBuf_.front()->timeStamp != odometryBuf_.front()->timeStamp) {
            mBuf_.unlock();
            break;
        }
        
        // pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>())
		// msg 형태의 cornerLastBuf의 각 요소를 laserCloudCornerLast에 저장
		// laserCloudCornerLast는 각 iter마다 하나의 feature pointcloud만을 가진다.
        laserCloudCornerLast_ = cornerLastBuf_.front();
        cornerLastBuf_.pop();
        laserCloudSurfLast_ = surfLastBuf_.front();
        surfLastBuf_.pop();
        laserCloudFullRes_ = fullResBuf_.front();
        fullResBuf_.pop();
        
        q_wodom_curr_ = odometryBuf_.front()->q_odom;
        t_wodom_curr_ = odometryBuf_.front()->t_odom;
        odometryLast_ = OdometryStamped::Ptr(new OdometryStamped());
        odometryLast_->timeStamp = odometryBuf_.front()->timeStamp;
        odometryBuf_.pop();
        
        while(!cornerLastBuf_.empty()) {
            cornerLastBuf_.pop();
        }
        mBuf_.unlock();
        
        /**********************************
		Purpose : world2laser(curr) 초기 추정값 설정
		Output  : 
			- Eigen::Map<Eigen::Quaterniond> q_w_curr_
			- Eigen::Map<Eigen::Vector3d> t_w_curr_
		**********************************/
        transformAssociateToMap();
        
        // 전체 map은 50x50x50m의 cube가 21x21x10개 모여서 만들어진다.
		// centerCube는 현재 laser가 인식한 local pointcloud 영역으로, 1 process는 이 centerCube 안에서 이루어진다.
		// centerCubeI,J,K는 현재 전체 21x21x10cube 크기의 map 내에 centerCube의 위치(index)를 의미한다.
        int centerCubeI = int((t_w_curr_.x() + 25.0) / 50.0) + opt_.laserCloudCenWidth;
        int centerCubeJ = int((t_w_curr_.y() + 25.0) / 50.0) + opt_.laserCloudCenHeight;
        int centerCubeK = int((t_w_curr_.z() + 25.0) / 50.0) + opt_.laserCloudCenDepth;

        if(t_w_curr_.x() + 25.0 < 0) centerCubeI++;
        if(t_w_curr_.y() + 25.0 < 0) centerCubeJ++;
        if(t_w_curr_.z() + 25.0 < 0) centerCubeK++;

        // centerCube 위치가 가생이에서 3칸 안에 있으면, 그 가생이 방향에 map 저장 방향이 부족하다고 판단하여
		// 모자란 방향으로 map을 확장시킨다. 
		// 이때, 21x21x10cube 크기는 고정이므로, 반대쪽 가생이에서 공간을 가져와 보충한다.
		// 이때 반대쪽 가생이에서 가져온 map 공간에 뭐 들어있을 수도 있지 않은가?
        while(centerCubeI < 3) {
            for(int j = 0; j < opt_.laserCloudHeight; j++) {
                for(int k = 0; k < opt_.laserCloudDepth; k++) {
                    int i = opt_.laserCloudWidth - 1;

                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer = 
                        laserCloudCornerArray_[cube(i, j, k)];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer = 
                        laserCloudSurfArray_[cube(i, j, k)];
                    for(; i >= 1; i++) {
                        laserCloudCornerArray_[cube(i, j, k)] = laserCloudCornerArray_[cube(i-1, j, k)];
                        laserCloudSurfArray_[cube(i, j, k)] = laserCloudSurfArray_[cube(i-1, j, k)];
                    }
                    // i = 0인 곳에 laserCloudCubeCornerPointer를 넣는다.
					// laserCloudCornerArray의 indexing 방법
					// i: width, j: height, k: depth
                    laserCloudCornerArray_[cube(i, j, k)] = laserCloudCubeCornerPointer;
                    laserCloudSurfArray_[cube(i, j, k)] = laserCloudCubeCornerPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }
            centerCubeI++;
            opt_.laserCloudCenWidth++;
        }

        while(centerCubeI >= opt_.laserCloudWidth - 3) {
            for(int j = 0; j < opt_.laserCloudHeight; j++) {
                for(int k = 0; k < opt_.laserCloudCenDepth; j++) {
                    int i = 0;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                        laserCloudCornerArray_[cube(i, j, k)];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                        laserCloudSurfArray_[cube(i, j, k)];

                    for(; i < opt_.laserCloudWidth - 1; i++) {
                        laserCloudCornerArray_[cube(i, j, k)] = laserCloudCornerArray_[cube(i+1, j, k)];
                        laserCloudSurfArray_[cube(i, j, k)] = laserCloudSurfArray_[cube(i+1, j, k)];
                    }
                    laserCloudCornerArray_[cube(i, j, k)] = laserCloudCubeCornerPointer;
                    laserCloudSurfArray_[cube(i, j, k)] = laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
                centerCubeI--;
                opt_.laserCloudCenWidth--;
            }
        }

        while(centerCubeJ < 3) {
            for(int i = 0; i < opt_.laserCloudWidth; i++) {
                for(int k = 0; k < opt_.laserCloudDepth; k++) {
                    int j = opt_.laserCloudHeight - 1;

                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer = 
                        laserCloudCornerArray_[cube(i, j, k)];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer = 
                        laserCloudSurfArray_[cube(i, j, k)];
                    for(; j >= 1; j--) {
                        laserCloudCornerArray_[cube(i, j, k)] = laserCloudCornerArray_[cube(i, j-1, k)];
                        laserCloudSurfArray_[cube(i, j, k)] = laserCloudSurfArray_[cube(i, j-1, k)];
                    }
                    // i = 0인 곳에 laserCloudCubeCornerPointer를 넣는다.
					// laserCloudCornerArray의 indexing 방법
					// i: width, j: height, k: depth
                    laserCloudCornerArray_[cube(i, j, k)] = laserCloudCubeCornerPointer;
                    laserCloudSurfArray_[cube(i, j, k)] = laserCloudCubeCornerPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }
            centerCubeJ++;
            opt_.laserCloudCenHeight++;
        }

        while(centerCubeJ >= opt_.laserCloudHeight - 3) {
            for(int i = 0; i < opt_.laserCloudWidth; i++) {
                for(int k = 0; k < opt_.laserCloudCenDepth; k++) {
                    int j = 0;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer = laserCloudCornerArray_[cube(i, j, k)];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer = laserCloudSurfArray_[cube(i, j, k)];

                    for(; i < opt_.laserCloudHeight - 1; i++) {
                        laserCloudCornerArray_[cube(i, j, k)] = laserCloudCornerArray_[cube(i, j+1, k)];
                        laserCloudSurfArray_[cube(i, j, k)] = laserCloudSurfArray_[cube(i, j+1, k)];
                    }
                    laserCloudCornerArray_[cube(i, j, k)] = laserCloudCubeCornerPointer;
                    laserCloudSurfArray_[cube(i, j, k)] = laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
                centerCubeJ--;
                opt_.laserCloudCenHeight--;
            }
        }

        while(centerCubeK < 3) {
            for(int i = 0; i < opt_.laserCloudWidth; i++) {
                for(int j = 0; j < opt_.laserCloudHeight; j++) {
                    int k = opt_.laserCloudDepth - 1;

                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer = 
                        laserCloudCornerArray_[cube(i, j, k)];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer = 
                        laserCloudSurfArray_[cube(i, j, k)];
                    for(; k >= 1; k--) {
                        laserCloudCornerArray_[cube(i, j, k)] = laserCloudCornerArray_[cube(i, j, k-1)];
                        laserCloudSurfArray_[cube(i, j, k)] = laserCloudSurfArray_[cube(i, j, k-1)];
                    }
                    // i = 0인 곳에 laserCloudCubeCornerPointer를 넣는다.
					// laserCloudCornerArray의 indexing 방법
					// i: width, j: height, k: depth
                    laserCloudCornerArray_[cube(i, j, k)] = laserCloudCubeCornerPointer;
                    laserCloudSurfArray_[cube(i, j, k)] = laserCloudCubeCornerPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }
            centerCubeK++;
            opt_.laserCloudCenDepth++;
        }

        while(centerCubeK >= opt_.laserCloudDepth - 3) {
            for(int i = 0; i < opt_.laserCloudWidth; i++) {
                for(int j = 0; j < opt_.laserCloudCenHeight; j++) {
                    int k = 0;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer = laserCloudCornerArray_[cube(i, j, k)];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer = laserCloudSurfArray_[cube(i, j, k)];

                    for(; k < opt_.laserCloudDepth - 1; k++) {
                        laserCloudCornerArray_[cube(i, j, k)] = laserCloudCornerArray_[cube(i, j, k+1)];
                        laserCloudSurfArray_[cube(i, j, k)] = laserCloudSurfArray_[cube(i, j, k+1)];
                    }
                    laserCloudCornerArray_[cube(i, j, k)] = laserCloudCubeCornerPointer;
                    laserCloudSurfArray_[cube(i, j, k)] = laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
                centerCubeK--;
                opt_.laserCloudCenDepth--;
            }
        }

        
        // valid&surround feature 추출
        
        int laserCloudValidNum = 0;
        laserCloudSurroundNum_ = 0;

        // centerCubeI의 범위: centerCubeI-2 <= centerCubeI <= centerCubeI+2
		// centerCubeJ의 범위: centerCubeJ-2 <= centerCubeJ <= centerCubeJ+2
		// centerCubeK의 범위: centerCubeK-1 <= centerCubeK <= centerCubeK+1
		// 즉 주변 5x5x3 범위의 큐브들의 index를 저장
        for(int i = centerCubeI - 2; i <= centerCubeI + 2; i++) {
            for(int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) {
                for(int k = centerCubeK - 1; k <= centerCubeK + 2; k++) {
                    if(i >= 0 && i < opt_.laserCloudWidth &&
                       j >= 0 && j < opt_.laserCloudHeight &&
                       k >= 0 && k < opt_.laserCloudDepth) {
                        laserCloudValidInd_[laserCloudValidNum] = cube(i, j, k);
                        laserCloudValidNum++;
                        laserCloudSurroundInd_[laserCloudSurroundNum_] = cube(i, j, k);
                        laserCloudSurroundNum_++;
                    }
                }
            }
        }
        
        // surround points in map to build tree
		// 해당 위치의 cube 주변의 5x5x3 cube의 point들을 laserCloudCornerFromMap에 추가
        laserCloudCornerFromMap_->clear();
        laserCloudSurfFromMap_->clear();
        for(int i = 0; i < laserCloudValidNum; i++) {
            *laserCloudCornerFromMap_ += *laserCloudCornerArray_[laserCloudValidInd_[i]];
            *laserCloudSurfFromMap_ += *laserCloudSurfArray_[laserCloudValidInd_[i]];
        }
        int laserCloudCornerFromMapNum = laserCloudCornerFromMap_->points.size();
        int laserCloudSurfFromMapNum = laserCloudSurfFromMap_->points.size();

        
        // feature pointcloud를 다운샘플링: pointcloud를 복셀로 만들어, 1복셀당 1point로 취급
		// 변환: laserCloudCornerLast >>(downfilter)>> laserCloudCornerStack
		// 참고: https://pcl.gitbook.io/tutorial/part-1/part01-chapter02
        pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr dsCornerInput(new pcl::PointCloud<PointType>(laserCloudCornerLast_->laserCloud));
        downSizeFilterCorner_.setInputCloud(dsCornerInput);
        downSizeFilterCorner_.filter(*laserCloudCornerStack);
        // downfilter된 pointcloud 개수 저장
        int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

        
        pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr dsSurfInput(new pcl::PointCloud<PointType>(laserCloudSurfLast_->laserCloud));
        downSizeFilterSurf_.setInputCloud(dsSurfInput);
        downSizeFilterSurf_.filter(*laserCloudSurfStack);
        int laserCloudSurfStackNum = laserCloudSurfStack->points.size();
        
        
        PointType pointSel;
        // 다운사이즈 된 이후의 corner feature와 surf feature 개수가 일정 이상이면
        if(laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50) {
            kdtreeCornerFromMap_->setInputCloud(laserCloudCornerFromMap_);
            kdtreeSurfFromMap_->setInputCloud(laserCloudSurfFromMap_);

            // ceres 최적화 과정 진행
            for(int iterCount = 0; iterCount < 2; iterCount++) {
                // outlier를 위한 loss function
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::LocalParameterization *q_parameterization =
                    new ceres::EigenQuaternionParameterization();
                ceres::Problem::Options problem_options;

                // parameter들을 블럭에 추가
                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(parameters_, 4, q_parameterization);
                problem.AddParameterBlock(parameters_ + 4, 3);

                // 최적화 함수에 넣기 위한 가공(전처리) 과정 시작
                PointType pointOri;
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;

                // corner feature pointcloud 최적화 재료 만들기
                int corner_num = 0;
                for(int i = 0; i < laserCloudCornerStackNum; i++) {
                    // 다운사이즈된 corner feature pointcloud를 저장 & world좌표계 기준으로 변환
                    pointOri = laserCloudCornerStack->points[i];
                    pointAssociateToMap(&pointOri, &pointSel);
                    // world좌표계 기준으로 변환된 point와 가장 비슷한 point를 map에서 찾는다.
                    kdtreeCornerFromMap_->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                    // 제일 안비슷한 녀석이 1m보다 작으면
                    if(pointSearchSqDis[4] < 1.0) {
                        // nearCorners: 제일 비슷한 point를 순서대로 5개 집어넣음
                        std::vector<Eigen::Vector3d> nearCorners;
                        Eigen::Vector3d center(0, 0, 0);
                        for(int j = 0; j < 5; j++) {
                            Eigen::Vector3d tmp(laserCloudCornerFromMap_->points[pointSearchInd[j]].x,
                                                laserCloudCornerFromMap_->points[pointSearchInd[j]].y,
                                                laserCloudCornerFromMap_->points[pointSearchInd[j]].z);
                            center += tmp;
                            nearCorners.push_back(tmp);
                        }
                        // 제일 가까운 5개의 point의 클러스터 중심점을 구한다
                        center /= 5.0;

                        // 그 중심점을 기준으로 하는 공분산을 구한다.
                        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                        for(int j = 0; j < 5; j++) {
                            Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                            covMat += tmpZeroMean * tmpZeroMean.transpose();
                        }
                        // 공분산의 고유값분해, 즉 PCA 분석으로 데이터의 방향성을 고유값, 고유벡터로 나타낸다.
                        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
                            
                        // 고유값은 오름차순으로 정렬되고, 고유벡터도 고유값과 같은 순서로 정렬된다.
                        Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                        Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                        // 오름차순으로 정렬된 고유값에 따라서 max값이 두번째 max값보다 3배 이상 클 때 실행
                        if(saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
                            // 클러스터 중심으로부터 데이터 방향으로 조금 떨어진 점 point_a, point_b
                            Eigen::Vector3d point_on_line = center;
                            Eigen::Vector3d point_a, point_b;
                            point_a = 0.1 * unit_direction + point_on_line;
                            point_b = -0.1 * unit_direction + point_on_line;

                            // 그 둘과 현재 pointOri 점과 비교하여 최적화 진행
                            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b);
                            problem.AddResidualBlock(cost_function, loss_function, parameters_, parameters_ + 4);
                            corner_num++;
                        }
                    }
                }

                // surf feature pointcloud 최적화 재료 만들기
                int surf_num = 0;
                for(int i = 0; i < laserCloudSurfStackNum; i++) {
                    pointOri = laserCloudSurfStack->points[i];
                    pointAssociateToMap(&pointOri, & pointSel);
                    kdtreeSurfFromMap_->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                    Eigen::Matrix<double, 5, 3> matA0;
                    Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                    if(pointSearchSqDis[4] < 1.0) {
                        for(int j = 0; j < 5; j++) {
                            matA0(j, 0) = laserCloudSurfFromMap_->points[pointSearchInd[j]].x;
                            matA0(j, 1) = laserCloudSurfFromMap_->points[pointSearchInd[j]].y;
                            matA0(j, 2) = laserCloudSurfFromMap_->points[pointSearchInd[j]].z;
                        }
                        // QR 분해, matA0 * norm = matB0을 풀어놓은 값
                        Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                        double negative_OA_dot_norm = 1 / norm.norm();
                        norm.normalize();

                        bool planeValid = true;
                        // plane인지 아닌지 구별
                        for(int j = 0; j < 5; j++) {
                            if(fabs(norm(0) * laserCloudSurfFromMap_->points[pointSearchInd[j]].x +
                                    norm(1) * laserCloudSurfFromMap_->points[pointSearchInd[j]].y +
                                    norm(2) * laserCloudSurfFromMap_->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2) {
                                planeValid = false;
                                break;
                            }
                        }
                        Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                        if(planeValid) {
                            ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                            problem.AddResidualBlock(cost_function, loss_function, parameters_, parameters_+4);
                            surf_num++;
                        }
                    }
                }

                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4;
                options.minimizer_progress_to_stdout = false;
                options.check_gradients = false;
                options.check_gradients = false;
                options.gradient_check_relative_precision = 1e-4;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
            }
        } else {
            cout << "time Map corner and surf num are not enough" << endl;
        }
        // 최적화 결과인 map2laser에서 map2odom값 추출
        transformUpdate();

        for(int i = 0; i < laserCloudCornerStackNum; i++) {
            // laser 기준 point를 world 좌표계 기준 point로 변환 >> 결과: pointSel
            pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

            // 지금 검사하고 있는 point가 cube에서 어느 위치에 들어갈지를 정하는 과정
            int cubeI = int((pointSel.x + 25.0) / 50.0) + opt_.laserCloudCenWidth;
            int cubeJ = int((pointSel.y + 25.0) / 50.0) + opt_.laserCloudCenHeight;
            int cubeK = int((pointSel.z + 25.0) / 50.0) + opt_.laserCloudCenDepth;
            
            if(pointSel.x + 25.0 < 0) cubeI--;
            if(pointSel.y + 25.0 < 0) cubeJ--;
            if(pointSel.z + 25.0 < 0) cubeK--;

            // 들어가도 될거같으면 넣는다
            if(cubeI >= 0 && cubeI < opt_.laserCloudWidth &&
               cubeJ >= 0 && cubeJ < opt_.laserCloudHeight &&
               cubeK >= 0 && cubeK < opt_.laserCloudDepth) {
                laserCloudCornerArray_[cube(cubeI, cubeJ, cubeK)]->push_back(pointSel);
            }
        }

        for(int i = 0; i< laserCloudSurfStackNum; i++) {
            // laser 기준 point를 world 좌표계 기준 point로 변환 >> 결과: pointSel
            pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

            // 지금 검사하고 있는 point가 cube에서 어느 위치에 들어갈지를 정하는 과정
            int cubeI = int((pointSel.x + 25.0) / 50.0) + opt_.laserCloudCenWidth;
            int cubeJ = int((pointSel.y + 25.0) / 50.0) + opt_.laserCloudCenHeight;
            int cubeK = int((pointSel.z + 25.0) / 50.0) + opt_.laserCloudCenDepth;
            
            // ...
            if(pointSel.x + 25.0 < 0) cubeI--;
            if(pointSel.y + 25.0 < 0) cubeJ--;
            if(pointSel.z + 25.0 < 0) cubeK--;

            // 들어가도 될거같으면 넣는다
            if(cubeI >= 0 && cubeI < opt_.laserCloudWidth &&
               cubeJ >= 0 && cubeJ < opt_.laserCloudHeight &&
               cubeK >= 0 && cubeK < opt_.laserCloudDepth) {
                laserCloudSurfArray_[cube(cubeI, cubeJ, cubeK)]->push_back(pointSel);
            }
        }

        for(int i = 0; i < laserCloudValidNum; i++) {
            int ind = laserCloudValidInd_[i];

            pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
            downSizeFilterCorner_.setInputCloud(laserCloudCornerArray_[ind]);
            downSizeFilterCorner_.filter(*tmpCorner);
            laserCloudCornerArray_[ind] = tmpCorner;

            pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
            downSizeFilterSurf_.setInputCloud(laserCloudSurfArray_[ind]);
            downSizeFilterSurf_.filter(*tmpSurf);
            laserCloudSurfArray_[ind] = tmpSurf;
        }
        odometryLast_->q_odom = q_w_curr_;
        odometryLast_->t_odom = t_w_curr_;
        successed = true;
    }
    return successed;
}