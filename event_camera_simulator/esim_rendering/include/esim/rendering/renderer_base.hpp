#pragma once

#include <ze/common/macros.hpp>
#include <esim/common/types.hpp>

namespace event_camera_simulator {

//! Represents a rendering engine that generates images (and other outputs,
//! such as depth maps, or optical flow maps) given a scene and a camera position.
class Renderer
{
public:
  ZE_POINTER_TYPEDEFS(Renderer);

  Renderer() {}

  //! Render an image at a given pose.
  virtual void render(const Transformation& T_W_C,
                      const std::vector<Transformation>& T_W_OBJ,
                      const ColorImagePtr& out_image,
                      const DepthmapPtr& out_depthmap) const = 0;


  //! Returns true if the rendering engine can compute optic flow, false otherwise
  virtual bool canComputeOpticFlow() const = 0;

  //! Render an image + depth map + optic flow map at a given pose,
  //! given the camera linear and angular velocity
  virtual void renderWithFlow(const Transformation& T_W_C,
                      const LinearVelocity& v_WC,
                      const AngularVelocity& w_WC,
                      const std::vector<Transformation>& T_W_OBJ,
                      const std::vector<LinearVelocity>& linear_velocity_obj,
                      const std::vector<AngularVelocity>& angular_velocity_obj,
                      const ColorImagePtr& out_image,
                      const DepthmapPtr& out_depthmap,
                      const OpticFlowPtr& optic_flow_map) const {}

  //! Sets the camera
  virtual void setCamera(const ze::Camera::Ptr& camera) = 0;

  //! Get the camera rig
  const ze::Camera::Ptr& getCamera() const { return camera_; }

protected:
  ze::Camera::Ptr camera_;
};

} // namespace event_camera_simulator
