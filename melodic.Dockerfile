FROM ros:melodic-ros-base

ARG DEBIAN_FRONTEND=noninteractive 
ARG PROJ_PATH=/esim
ARG ESIM_PATH=${PROJ_PATH}/src/esim

# Installing some essential system packages
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
RUN apt-get update
RUN apt-get install -y \
      python3-vcstool \
      python3-catkin-tools \
      python3-pip \
      libproj-dev \
      libglm-dev

RUN apt-get clean autoclean &&\
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install -U pip

RUN mkdir -p ${ESIM_PATH}
COPY ./ ${ESIM_PATH}
WORKDIR ${ESIM_PATH}
RUN catkin init
RUN catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN vcs-import < ${ESIM_PATH}/dependencies.yaml

WORKDIR ${ESIM_PATH}/ze_oss
RUN touch ze_oss/imp_3rdparty_cuda_toolkit/CATKIN_IGNORE \
      ze_oss/imp_app_pangolin_example/CATKIN_IGNORE \
      ze_oss/imp_benchmark_aligned_allocator/CATKIN_IGNORE \
      ze_oss/imp_bridge_pangolin/CATKIN_IGNORE \
      ze_oss/imp_cu_core/CATKIN_IGNORE \
      ze_oss/imp_cu_correspondence/CATKIN_IGNORE \
      ze_oss/imp_cu_imgproc/CATKIN_IGNORE \
      ze_oss/imp_ros_rof_denoising/CATKIN_IGNORE \
      ze_oss/imp_tools_cmd/CATKIN_IGNORE \
      ze_oss/ze_data_provider/CATKIN_IGNORE \
      ze_oss/ze_geometry/CATKIN_IGNORE \
      ze_oss/ze_imu/CATKIN_IGNORE \
      ze_oss/ze_trajectory_analysis/CATKIN_IGNORE

RUN catkin build esim_ros &&\
    . ${PROJ_PATH}/devel/setup.bash
