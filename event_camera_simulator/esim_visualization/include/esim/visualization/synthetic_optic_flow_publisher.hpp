#pragma once

#include <esim/common/types.hpp>
#include <esim/visualization/publisher_interface.hpp>

#include <fstream>

namespace event_camera_simulator {

class SyntheticOpticFlowPublisher : public Publisher
{
public:
  SyntheticOpticFlowPublisher(const std::string &output_folder);

  ~SyntheticOpticFlowPublisher();

  virtual void imageCallback(const ImagePtrVector& images, Time t) override {
    CHECK_EQ(images.size(), 1);
    if(sensor_size_.width == 0 || sensor_size_.height == 0)
    {
      sensor_size_ = images[0]->size();
    }
  }

  virtual void eventsCallback(const EventsVector& events) override;
  virtual void opticFlowCallback(const OpticFlowPtrVector& optic_flows, Time t)  override {}

  virtual void imageCorruptedCallback(const ImagePtrVector& corrupted_images, Time t) override {}
  virtual void depthmapCallback(const DepthmapPtrVector& depthmaps, Time t) override {}
  virtual void poseCallback(const Transformation& T_W_B, const TransformationVector& T_W_Cs, Time t) override {}
  virtual void twistCallback(const AngularVelocityVector& ws, const LinearVelocityVector& vs, Time t) override {}
  virtual void imuCallback(const Vector3& acc, const Vector3& gyr, Time t) override {}
  virtual void cameraInfoCallback(const ze::CameraRig::Ptr& camera_rig, Time t) override {}
  virtual void pointcloudCallback(const PointCloudVector& pointclouds, Time t) override {}

  static Publisher::Ptr createFromGflags();

private:
  std::string output_folder_;
  cv::Size sensor_size_;
  std::ofstream events_file_;
  Events events_; // buffer containing all the events since the beginning
};

} // namespace event_camera_simulator
