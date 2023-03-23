set(scanRegistration_SOURCES
        scanRegistration/scanRegistration.cpp
        scanRegistration/scanRegistrationOption.cpp)

set(laserOdometry_SOURCES
        laserOdometry/laserOdometry.cpp
        laserOdometry/laserOdometryOption.cpp)

set(laserMapping_SOURCES
        laserMapping/laserMapping.cpp
        laserMapping/laserMappingOption.cpp)

set(kittiHelper_SOURCES
        kittiHelper/kittiHelper.cpp)


add_library(${PROJECT_NAME} SHARED
        ${scanRegistration_SOURCES}
        ${laserOdometry_SOURCES}
        ${laserMapping_SOURCES})
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        ${PCL_CONVERSIONS_LIBRARIES}
        ${CERES_LIBRARIES})