# CMakeLists.txt 간단 사용법
# https://alida.tistory.com/19
cmake_minimum_required(VERSION 3.8)
project(SP_A_LOAM)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(SP_A_LOAM_VERSION 1.0.0)
set(ConfigPackageLocation lib/cmake/${CMAKE_PROJECT_NAME})

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${PCL_CONVERSIONS_INCLUDE_DIRS})

add_subdirectory(src)

## 아래는 add_library로부터 나온 target인 SP_A_LOAM으로부터 Config.cmake, ConfigVersion.cmake, Targets.cmake를 생성하는 과정이다.
## 세 개의 cmake 파일이 무엇을 목적으로 하는지는 아직 모르지만 우선 생성 방법은 아래와 같다.
set(${PROJECT_NAME}_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${ConfigPackageLocation})
# (뇌피셜)타겟 SP_A_LOAM을 install 할때 쓸 수 있는 SP_A_LOAMTargets형태로 변환한다
# 그리고 타겟 SP_A_LOAM 출신 요소들인 LIBRARY, ARCHIVE, RUNTIME, INCLUDES를 각 DESTINATION에 저장한다.
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

# install = 복사만 하는거
install(DIRECTORY include/
        DESTINATION include)

# for build tree / in workspace
# 이 시점에서 SP_A_LOAMTargets.cmake가 build/lib/cmake에 export(=저장??)된다.
export(EXPORT ${PROJECT_NAME}Targets
        FILE ${${PROJECT_NAME}_INSTALL_DIR}/${PROJECT_NAME}Targets.cmake)

# exported targets
# 이 시점에서 install 폴더에 저장
# 사실 cmake 결과인 cmake_install.cmake가 실행되어서 install_manifest.txt를
# 불러오고 그거갖고 머시기 다 한 뒤에야 SP_A_LOAMTargets.cmake가 install/SP_A_LOAM/lib/cmake 폴더에 저장
install(EXPORT ${PROJECT_NAME}Targets
        FILE ${PROJECT_NAME}Targets.cmake
        DESTINATION ${ConfigPackageLocation})

## generate package version file
include(CMakePackageConfigHelpers)
# 이 시점에서 SP_A_LOAMConfigVersion.cmake가 build/lib/cmake에 export된다.
write_basic_package_version_file(
        ${${PROJECT_NAME}_INSTALL_DIR}/${PROJECT_NAME}ConfigVersion.cmake
        VERSION ${SP_A_LOAM_VERSION}
        COMPATIBILITY SameMajorVersion)
## generate package configuration file
# 이 시점에서 SP_A_LOAMConfig.cmake가 build/lib/cmake에 export된다.
configure_package_config_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
        "${${PROJECT_NAME}_INSTALL_DIR}/${PROJECT_NAME}Config.cmake"
        INSTALL_DESTINATION ${${PROJECT_NAME}_INSTALL_DIR}
        PATH_VARS ${PROJECT_NAME}_INSTALL_DIR)

## install the package configuration files
# FILES에 적힌 파일들을 DESTINATION으로 복사(install)
 install(FILES
         ${${PROJECT_NAME}_INSTALL_DIR}/${PROJECT_NAME}Config.cmake
         ${${PROJECT_NAME}_INSTALL_DIR}/${PROJECT_NAME}ConfigVersion.cmake
         DESTINATION ${ConfigPackageLocation})
#add_executable(${PROJECT_NAME} src/main.cpp)
#target_link_libraries(${PROJECT_NAME}
#        ${PROJECT_NAME})
