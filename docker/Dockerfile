# FROM ubuntu:20.04
FROM osrf/ros:noetic-desktop-full

ENV CERES_VERSION="1.12.0"
ENV PCL_VERSION="1.8.0"
ENV GTSAM_VERSION="4.2a8"

RUN apt-get update && apt-get install -y curl lsb locales wget ntp sudo net-tools nano

# RUN locale-gen en_US.UTF-8
# ENV LANG en_US.UTF-8
# ENV LANGUAGE en_US:en
# ENV LC_ALL en_US.UTF-8

    # setup processors number used to compile library
# RUN if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; else export USE_PROC=$(($(nproc)/2)) ; fi && \
#     # Install dependencies
#       apt-get update && apt-get install -y \
#       cmake \
#       libatlas-base-dev \
#       libeigen3-dev \
#       libgoogle-glog-dev \
#       libsuitesparse-dev \
#       python3-catkin-tools \
#       ros-${ROS_DISTRO}-cv-bridge \
#       ros-${ROS_DISTRO}-image-transport \
#       ros-${ROS_DISTRO}-message-filters \
#       ros-${ROS_DISTRO}-tf \
#       git && \
#     rm -rf /var/lib/apt/lists/*
    # Build and install Ceres
# RUN git clone https://ceres-solver.googlesource.com/ceres-solver && \
#     cd ceres-solver && \
#     git checkout tags/${CERES_VERSION} && \
#     mkdir build && cd build && \
#     cmake .. && \
#     make -j8 install && \
#     cd ../.. && \
#     rm -rf ./ceres-solver
# RUN git clone https://github.com/borglab/gtsam.git && \
#     cd gtsam && \
#     git checkout tags/${GTSAM_VERSION} && \
#     mkdir build && cd build && \
#     cmake .. && \
#     make -j8 install && \
#     cd ../.. && \
#     rm -rf ./gtsam
# RUN cd /root/docker_ws/ros/catkin_ws/ && \
#     catkin build -j8 && \
#     source devel/setup.bash && \
#     echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
#     echo "source /root/docker_ws/ros/catkin_ws/devel/setup.bash" >> /root/.bashrc
