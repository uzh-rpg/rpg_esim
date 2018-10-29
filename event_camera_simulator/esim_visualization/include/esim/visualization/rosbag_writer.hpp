#pragma once

#include <esim/common/types.hpp>
#include <esim/visualization/publisher_interface.hpp>
#include <rosbag/bag.h>

namespace event_camera_simulator {

class RosbagWriter : public Publisher
{
public:
  RosbagWriter(const std::string& path_to_output_bag,
               size_t num_cameras);
  ~RosbagWriter();

  virtual void imageCallback(const ImagePtrVector& images, Time t) override;
  virtual void imageCorruptedCallback(const ImagePtrVector& corrupted_images, Time t) override;
  virtual void depthmapCallback(const DepthmapPtrVector& depthmaps, Time t) override;
  virtual void opticFlowCallback(const OpticFlowPtrVector& optic_flows, Time t) override;
  virtual void eventsCallback(const EventsVector& events) override;
  virtual void poseCallback(const Transformation& T_W_B, const TransformationVector& T_W_Cs, Time t) override;
  virtual void twistCallback(const AngularVelocityVector& ws, const LinearVelocityVector& vs, Time t) override;
  virtual void imuCallback(const Vector3& acc, const Vector3& gyr, Time t) override;
  virtual void cameraInfoCallback(const ze::CameraRig::Ptr& camera_rig, Time t) override;
  virtual void pointcloudCallback(const PointCloudVector& pointclouds, Time t) override;

  static Publisher::Ptr createBagWriterFromGflags(size_t num_cameras);

private:
  size_t num_cameras_;
  std::vector<cv::Size> sensor_sizes_;
  rosbag::Bag bag_;

  const std::string topic_name_prefix_ = "";

  Time last_published_camera_info_time_;
  Time last_published_image_time_;
  Time last_published_corrupted_image_time_;
  Time last_published_depthmap_time_;
  Time last_published_optic_flow_time_;
  Time last_published_pointcloud_time_;

};

} // namespace event_camera_simulator
