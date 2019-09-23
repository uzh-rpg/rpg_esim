#pragma once

#include <esim/common/types.hpp>
#include <ze/common/macros.hpp>

namespace event_camera_simulator {

class Publisher
{
public:
  ZE_POINTER_TYPEDEFS(Publisher);

  Publisher() = default;
  virtual ~Publisher() = default;

  virtual void imageCallback(const ImagePtrVector& images, Time t) {}
  virtual void imageCorruptedCallback(const ImagePtrVector& corrupted_images, Time t) {}
  virtual void depthmapCallback(const DepthmapPtrVector& depthmaps, Time t) {}
  virtual void opticFlowCallback(const OpticFlowPtrVector& optic_flows, Time t) {}
  virtual void eventsCallback(const EventsVector& events) {}
  virtual void poseCallback(const Transformation& T_W_B, const TransformationVector& T_W_Cs, Time t) {}
  virtual void twistCallback(const AngularVelocityVector& ws, const LinearVelocityVector& vs, Time t) {}
  virtual void imuCallback(const Vector3& acc, const Vector3& gyr, Time t) {}
  virtual void cameraInfoCallback(const ze::CameraRig::Ptr& camera_rig, Time t) {}
  virtual void pointcloudCallback(const PointCloudVector& pointclouds, Time t) {}

};

} // namespace event_camera_simulator
