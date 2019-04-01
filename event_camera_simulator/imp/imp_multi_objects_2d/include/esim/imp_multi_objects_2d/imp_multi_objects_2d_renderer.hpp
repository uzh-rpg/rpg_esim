#pragma once

#include <esim/rendering/simple_renderer_base.hpp>
#include <esim/imp_multi_objects_2d/object.hpp>

namespace event_camera_simulator {

//! A rendering engine for planar scenes
class MultiObject2DRenderer : public SimpleRenderer
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MultiObject2DRenderer();

  ~MultiObject2DRenderer();

  virtual bool render(const Time t,
                      const ColorImagePtr& out_image,
                      const OpticFlowPtr& optic_flow_map) const;

  virtual int getWidth() const { return width_; }

  virtual int getHeight() const { return height_; }

protected:

  void outputGroundTruthData();

  std::vector<std::shared_ptr<Object>> objects_;
  int width_, height_;
  ze::real_t tmax_;
};

} // namespace event_camera_simulator
