FROM ros:kinetic-ros-base

ARG DEBIAN_FRONTEND=noninteractive 
ARG ESIM_PATH=/esim


# Installing some essential system packages
#RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
RUN apt-get update
RUN apt-get install -y \
      python3-vcstool \
      python3-pip \
      ros-kinetic-dynamic-reconfigure \
      ros-kinetic-message-filters \
      ros-kinetic-nodelet \
      ros-kinetic-nodelet-topic-tools \
      ros-kinetic-pcl-conversions \
      ros-kinetic-pcl-msgs \
      ros-kinetic-pluginlib \
      ros-kinetic-rosbag \
      ros-kinetic-roscpp \
      ros-kinetic-sensor-msgs \
      ros-kinetic-std-msgs \
      ros-kinetic-tf \
      ros-kinetic-tf2-eigen \
      ros-kinetic-pcl-ros \
      libproj-dev \
      libglm-dev

RUN apt-get clean autoclean &&\
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install -U pip catkin-tools

RUN mkdir -p ${ESIM_PATH}/src
COPY ./ /esim/src
WORKDIR ${ESIM_PATH}/src
RUN catkin init
RUN catkin config --extend /opt/ros/kinetic --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN vcs-import < rpg_esim/dependencies.yaml

