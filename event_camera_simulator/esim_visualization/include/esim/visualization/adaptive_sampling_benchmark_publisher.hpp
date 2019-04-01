#pragma once

#include <esim/common/types.hpp>
#include <esim/visualization/publisher_interface.hpp>

#include <fstream>

namespace event_camera_simulator {

class AdaptiveSamplingBenchmarkPublisher : public Publisher
{
public:

  using PixelLocation = std::pair<int,int>;
  using PixelLocations = std::vector<PixelLocation>;

  AdaptiveSamplingBenchmarkPublisher(const std::string &benchmark_folder,
                                     const std::string &pixels_to_record_filename);

  ~AdaptiveSamplingBenchmarkPublisher();

  virtual void imageCallback(const ColorImagePtrVector& images, Time t) override;
  virtual void eventsCallback(const EventsVector& events) override;
  virtual void opticFlowCallback(const OpticFlowPtrVector& optic_flows, Time t) override;

  virtual void imageCorruptedCallback(const ColorImagePtrVector& corrupted_images, Time t) override {}
  virtual void depthmapCallback(const DepthmapPtrVector& depthmaps, Time t) override {}
  virtual void poseCallback(const Transformation& T_W_B, const TransformationVector& T_W_Cs, Time t) override {}
  virtual void twistCallback(const AngularVelocityVector& ws, const LinearVelocityVector& vs, Time t) override {}
  virtual void imuCallback(const Vector3& acc, const Vector3& gyr, Time t) override {}
  virtual void cameraInfoCallback(const ze::CameraRig::Ptr& camera_rig, Time t) override {}
  virtual void pointcloudCallback(const PointCloudVector& pointclouds, Time t) override {}

  static Publisher::Ptr createFromGflags();

private:
  std::ofstream events_file_;
  std::ofstream images_file_;
  std::ofstream pixel_intensities_file_;
  std::ofstream optic_flows_file_;
  size_t image_index_;
  PixelLocations pixels_to_record_;
};

} // namespace event_camera_simulator
