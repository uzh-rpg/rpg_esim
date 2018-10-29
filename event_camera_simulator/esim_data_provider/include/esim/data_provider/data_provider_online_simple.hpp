#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ze/common/macros.hpp>
#include <ze/common/types.hpp>
#include <esim/data_provider/data_provider_base.hpp>
#include <esim/rendering/simple_renderer_base.hpp>


namespace event_camera_simulator {

/**
 * Simple online data provider, intended to simulate a single event camera
 * based on images + optic flow maps provided by a rendering engine.
 *
 * This data provider does NOT simulate a camera trajectory or an IMU.
 */
class DataProviderOnlineSimple : public DataProviderBase
{
public:
  DataProviderOnlineSimple(ze::real_t simulation_minimum_framerate,
                           int simulation_adaptive_sampling_method,
                           ze::real_t simulation_adaptive_sampling_lambda);

  virtual ~DataProviderOnlineSimple();

  virtual bool spinOnce() override;

  virtual bool ok() const override;

  size_t numCameras() const override;

private:
  bool sampleFrame();
  void setFrameUpdated();

  SimpleRenderer::Ptr renderer_;
  SimulatorData sim_data_;

  ze::real_t t_;
  ze::real_t last_t_frame_; // latest next sampling time in order to guarantee the minimum frame rate
  ze::real_t next_t_adaptive_;  // latest next sampling time in order to guarantee that the adaptive sampling scheme is respected
  ze::real_t dt_frame_;

  ze::real_t simulation_minimum_framerate_;
  int simulation_adaptive_sampling_method_;
  ze::real_t simulation_adaptive_sampling_lambda_;
};

} // namespace event_camera_simulator
