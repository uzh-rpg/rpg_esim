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

  virtual void imageCallback(const ImagePtrVector& images, Time t) = 0;
  virtual void imageCorruptedCallback(const ImagePtrVector& corrupted_images, Time t) = 0;
  virtual void depthmapCallback(const DepthmapPtrVector& depthmaps, Time t) = 0;
  virtual void opticFlowCallback(const OpticFlowPtrVector& optic_flows, Time t) = 0;
  virtual void eventsCallback(const EventsVector& events) = 0;
  virtual void poseCallback(const Transformation& T_W_B, const TransformationVector& T_W_Cs, Time t) = 0;
  virtual void twistCallback(const AngularVelocityVector& ws, const LinearVelocityVector& vs, Time t) = 0;
  virtual void imuCallback(const Vector3& acc, const Vector3& gyr, Time t) = 0;
  virtual void cameraInfoCallback(const ze::CameraRig::Ptr& camera_rig, Time t) = 0;
  virtual void pointcloudCallback(const PointCloudVector& pointclouds, Time t) = 0;

};

} // namespace event_camera_simulator
