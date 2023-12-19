FROM ros:melodic-ros-base

ARG DEBIAN_FRONTEND=noninteractive 
ARG PROJ_PATH=/home/user/sim_ws
ARG ESIM_PATH=$PROJ_PATH/src/rpg_esim

# Installing some essential system packages
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
RUN apt-get update
RUN apt-get install -y \
      python3-vcstool \
      python3-catkin-tools \
      python3-pip \
      libproj-dev \
      libglm-dev \
      libopencv-dev \
      ros-melodic-image-transport \
      ros-melodic-cv-bridge \
      autoconf \
      libyaml-cpp-dev \
      ros-melodic-eigen-conversions \
      libtool \
      ros-melodic-tf-conversions \
      ros-melodic-tf \
      ros-melodic-pcl-ros \
      libglfw3-dev \
      libassimp-dev
      #ros-melodic-assimp-devel

RUN apt-get clean autoclean &&\
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install -U pip

WORKDIR $PROJ_PATH
RUN catkin init
RUN catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN mkdir -p $ESIM_PATH
COPY ./ $ESIM_PATH
WORKDIR $PROJ_PATH/src
RUN vcs-import < $ESIM_PATH/dependencies.yaml

WORKDIR $PROJ_PATH/src/ze_oss
RUN touch \
	imp_3rdparty_cuda_toolkit/CATKIN_IGNORE \
       imp_app_pangolin_example/CATKIN_IGNORE \
       imp_benchmark_aligned_allocator/CATKIN_IGNORE \
       imp_bridge_pangolin/CATKIN_IGNORE \
       imp_cu_core/CATKIN_IGNORE \
       imp_cu_correspondence/CATKIN_IGNORE \
       imp_cu_imgproc/CATKIN_IGNORE \
       imp_ros_rof_denoising/CATKIN_IGNORE \
       imp_tools_cmd/CATKIN_IGNORE \
       ze_data_provider/CATKIN_IGNORE \
       ze_geometry/CATKIN_IGNORE \
       ze_imu/CATKIN_IGNORE \
       ze_trajectory_analysis/CATKIN_IGNORE

WORKDIR $PROJ_PATH
RUN catkin build esim_ros
#ENTRYPOINT  [". /home/user/sim_ws/devel/setup.bash"]
