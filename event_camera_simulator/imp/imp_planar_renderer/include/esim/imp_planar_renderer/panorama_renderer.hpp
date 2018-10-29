#pragma once

#include <esim/rendering/renderer_base.hpp>

namespace event_camera_simulator {

//! A rendering engine for planar scenes
class PanoramaRenderer : public Renderer
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PanoramaRenderer(const Image& texture,
                   const Transformation::Rotation& R_W_P);

  ~PanoramaRenderer();

  //! Render image and depth map for a given camera pose
  virtual void render(const Transformation& T_W_C, const ImagePtr &out_image, const DepthmapPtr &out_depthmap) const override;

  virtual void setCamera(const ze::Camera::Ptr& camera) override;

protected:

  void precomputePixelToBearingLookupTable();
  void projectToPanorama(const Eigen::Ref<const ze::Bearing>& f, ze::Keypoint* keypoint) const;

  // Texture mapped on the plane
  Image texture_;
  Transformation::Rotation R_W_P_;

  // Intrinsic parameters of the panorama camera
  FloatType fx_, fy_;
  ze::Keypoint principal_point_;

  // Precomputed lookup table from keypoint -> bearing vector
  ze::Bearings bearings_C_;

  // Preallocated warping matrices
  mutable cv::Mat_<float> map_x_, map_y_;
};

} // namespace event_camera_simulator
