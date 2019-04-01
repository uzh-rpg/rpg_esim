#pragma once

#include <esim/rendering/renderer_base.hpp>

namespace event_camera_simulator {

//! A rendering engine for planar scenes
class PlanarRenderer : public Renderer
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PlanarRenderer(const ColorImage& texture,
                 const Camera::Ptr& cam_src,
                 const Transformation &T_W_P,
                 FloatType z_min,
                 FloatType z_max,
                 bool extend_border);

  ~PlanarRenderer();

  //! Render image and depth map for a given camera pose
  virtual void render(const Transformation& T_W_C, const ColorImagePtr &out_image, const DepthmapPtr &out_depthmap) const;
  void render(const Transformation& T_W_C, const std::vector<Transformation>& T_W_OBJ, const ColorImagePtr &out_image, const DepthmapPtr &out_depthmap) const override
  {
      render(T_W_C, out_image, out_depthmap);
  }

  //! Returns true if the rendering engine can compute optic flow, false otherwise
  virtual bool canComputeOpticFlow() const override { return false; }

  virtual void setCamera(const ze::Camera::Ptr& camera) override;

protected:

  void precomputePixelToBearingLookupTable();

  // Texture mapped on the plane
  ColorImage texture_;
  Camera::Ptr cam_src_;
  CalibrationMatrix K_src_, K_src_inv_;

  // Plane parameters
  Transformation T_W_P_;

  // Clipping parameters
  FloatType z_min_;
  FloatType z_max_;

  bool extend_border_;

  // Precomputed lookup table from keypoint -> bearing vector
  ze::Bearings bearings_C_;

  // Preallocated warping matrices
  mutable cv::Mat_<float> map_x_, map_y_;
};

} // namespace event_camera_simulator
