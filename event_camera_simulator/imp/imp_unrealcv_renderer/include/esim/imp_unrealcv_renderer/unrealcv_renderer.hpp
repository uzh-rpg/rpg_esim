#pragma once

#include <esim/rendering/renderer_base.hpp>

namespace event_camera_simulator {

class UnrealCvClient; // fwd

class UnrealCvRenderer : public Renderer
{
public:

  UnrealCvRenderer();

  //! Render image and depth map for a given camera pose
  virtual void render(const Transformation& T_W_C, const ImagePtr &out_image, const DepthmapPtr &out_depthmap) const;

  void render(const Transformation& T_W_C, const std::vector<Transformation>& T_W_OBJ, const ImagePtr &out_image, const DepthmapPtr &out_depthmap) const
  {
      render(T_W_C, out_image, out_depthmap);
  }

  //! Returns true if the rendering engine can compute optic flow, false otherwise
  virtual bool canComputeOpticFlow() const override { return false; }

  virtual void setCamera(const ze::Camera::Ptr& camera) override;

private:
  std::shared_ptr<UnrealCvClient> client_;
  mutable size_t frame_idx_;
};


}
