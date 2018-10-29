#pragma once

#include <ze/common/macros.hpp>
#include <esim/common/types.hpp>

namespace event_camera_simulator {

//! Represents a rendering engine that generates images + optic flow maps
//! The rendering engine takes care of managing the environment and camera trajectory in the environment
class SimpleRenderer
{
public:
  ZE_POINTER_TYPEDEFS(SimpleRenderer);

  SimpleRenderer() {}

  //! Render an image + optic flow map at a given time t.
  //! The rendering engine takes care of generating the camera trajectory, etc.
  virtual bool render(const Time t,
                      const ImagePtr& out_image,
                      const OpticFlowPtr& optic_flow_map) const = 0;

  //! Get the height of the image plane
  virtual int getWidth() const = 0;

  //! Get the width of the image plane
  virtual int getHeight() const = 0;

protected:
};

} // namespace event_camera_simulator
