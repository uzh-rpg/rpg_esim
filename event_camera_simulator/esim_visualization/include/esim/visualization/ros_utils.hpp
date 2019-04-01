#pragma once

#include <esim/common/types.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <dvs_msgs/EventArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <esim_msgs/OpticFlow.h>

namespace event_camera_simulator {

inline std::string getTopicName(int i, const std::string& suffix)
{
  std::stringstream ss;
  ss << "cam" << i << "/" << suffix;
  return ss.str();
}

inline std::string getTopicName(const std::string& prefix, int i, const std::string& suffix)
{
  std::stringstream ss;
  ss << prefix << "/" << getTopicName(i, suffix);
  return ss.str();
}

inline ros::Time toRosTime(Time t)
{
  ros::Time ros_time;
  ros_time.fromNSec(t);
  return ros_time;
}

void pointCloudToMsg(const PointCloud& pointcloud, const std::string& frame_id, Time t,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& msg);

void imageToMsg(const ColorImage& image, Time t, sensor_msgs::ImagePtr& msg);

void depthmapToMsg(const Depthmap& depthmap, Time t, sensor_msgs::ImagePtr& msg);

void opticFlowToMsg(const OpticFlow& flow, Time t, esim_msgs::OpticFlowPtr& msg);

void eventsToMsg(const Events& events, int width, int height, dvs_msgs::EventArrayPtr& msg);

sensor_msgs::Imu imuToMsg(const Vector3& acc, const Vector3& gyr, Time t);

geometry_msgs::TwistStamped twistToMsg(const AngularVelocity& w, const LinearVelocity& v, Time t);

void cameraToMsg(const ze::Camera::Ptr& camera, Time t, sensor_msgs::CameraInfoPtr& msg);


} // namespace event_camera_simulator
