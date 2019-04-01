#pragma once

#include <esim/common/types.hpp>
#include <esim/visualization/publisher_interface.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace event_camera_simulator {

class RosPublisher : public Publisher
{
public:
  RosPublisher(size_t num_cameras);
  ~RosPublisher();

  virtual void imageCallback(const ColorImagePtrVector& images, Time t) override;
  virtual void imageCorruptedCallback(const ColorImagePtrVector& corrupted_images, Time t) override;
  virtual void depthmapCallback(const DepthmapPtrVector& depthmaps, Time t) override;
  virtual void opticFlowCallback(const OpticFlowPtrVector& optic_flows, Time t) override;
  virtual void eventsCallback(const EventsVector& events) override;
  virtual void poseCallback(const Transformation& T_W_B, const TransformationVector& T_W_Cs, Time t) override;
  virtual void twistCallback(const AngularVelocityVector& ws, const LinearVelocityVector& vs, Time t) override;
  virtual void imuCallback(const Vector3& acc, const Vector3& gyr, Time t) override;
  virtual void cameraInfoCallback(const ze::CameraRig::Ptr& camera_rig, Time t) override;
  virtual void pointcloudCallback(const PointCloudVector& pointclouds, Time t) override;

private:
  size_t num_cameras_;
  std::vector<cv::Size> sensor_sizes_;

  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<image_transport::ImageTransport> it_;

  std::vector< std::shared_ptr<ros::Publisher> > event_pub_;
  std::shared_ptr<ros::Publisher> pose_pub_;
  std::shared_ptr<ros::Publisher> imu_pub_;
  std::vector< std::shared_ptr<ros::Publisher> > pointcloud_pub_;
  std::vector< std::shared_ptr<ros::Publisher> > camera_info_pub_;
  std::vector< std::shared_ptr<image_transport::Publisher> > image_pub_;
  std::vector< std::shared_ptr<image_transport::Publisher> > image_corrupted_pub_;
  std::vector< std::shared_ptr<image_transport::Publisher> > depthmap_pub_;
  std::vector< std::shared_ptr<ros::Publisher> > optic_flow_pub_;
  std::vector< std::shared_ptr<ros::Publisher> > twist_pub_;
  std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;

  Time last_published_camera_info_time_;
  Time last_published_image_time_;
  Time last_published_corrupted_image_time_;
  Time last_published_depthmap_time_;
  Time last_published_optic_flow_time_;
  Time last_published_pointcloud_time_;

};

} // namespace event_camera_simulator
