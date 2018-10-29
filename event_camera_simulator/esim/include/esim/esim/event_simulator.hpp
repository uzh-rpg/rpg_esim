#pragma once

#include <esim/common/types.hpp>

namespace event_camera_simulator {

/*
 * The EventSimulator takes as input a sequence of stamped images,
 * assumed to be sampled at a "sufficiently high" framerate,
 * and simulates the principle of operation of an idea event camera
 * with a constant contrast threshold C.
 * Pixel-wise intensity values are linearly interpolated in time.
 *
 * The pixel-wise voltages are reset with the values from the first image
 * which is passed to the simulator.
 */
class EventSimulator
{
public:

  struct Config
  {
    double Cp;
    double Cm;
    double sigma_Cp;
    double sigma_Cm;
    Duration refractory_period_ns;
    bool use_log_image;
    double log_eps;
  };

  using TimestampImage = cv::Mat_<ze::real_t>;

  EventSimulator(const Config& config)
    : config_(config),
      is_initialized_(false),
      current_time_(0)
  {}

  void init(const Image& img, Time time);
  Events imageCallback(const Image& img, Time time);

private:
  bool is_initialized_;
  Time current_time_;
  Image ref_values_;
  Image last_img_;
  TimestampImage last_event_timestamp_;
  cv::Size size_;

  Config config_;
};

} // namespace event_camera_simulator
