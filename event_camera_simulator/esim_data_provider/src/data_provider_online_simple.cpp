#include <esim/data_provider/data_provider_online_simple.hpp>
#include <ze/common/time_conversions.hpp>
#include <esim/data_provider/renderer_factory.hpp>
#include <esim/common/utils.hpp>

DECLARE_double(simulation_post_gaussian_blur_sigma);

namespace event_camera_simulator {

DataProviderOnlineSimple::DataProviderOnlineSimple(ze::real_t simulation_minimum_framerate,
                                                   int simulation_adaptive_sampling_method,
                                                   ze::real_t simulation_adaptive_sampling_lambda)
  : DataProviderBase(DataProviderType::RendererOnline),
    simulation_minimum_framerate_(simulation_minimum_framerate),
    simulation_adaptive_sampling_method_(simulation_adaptive_sampling_method),
    simulation_adaptive_sampling_lambda_(simulation_adaptive_sampling_lambda),
    dt_frame_(1./simulation_minimum_framerate)
{
  CHECK(simulation_adaptive_sampling_method == 0
        || simulation_adaptive_sampling_method == 1);

  renderer_ = loadSimpleRendererFromGflags();

  const size_t num_cameras = 1u;

  for(size_t i=0; i<num_cameras; ++i)
  {
    const cv::Size size = cv::Size(renderer_->getWidth(),
                                   renderer_->getHeight());

    sim_data_.images.emplace_back(ColorImagePtr(new ColorImage(size)));
    sim_data_.optic_flows.emplace_back(OpticFlowPtr(new OpticFlow(size)));

    sim_data_.images[i]->setTo(0);
    sim_data_.optic_flows[i]->setTo(0);
  }

  // At the initial time, we sample a frame + optic flow map
  t_ = 0.;
  sampleFrame();
  setFrameUpdated();
  if(callback_)
  {
    callback_(sim_data_);
  }
}

DataProviderOnlineSimple::~DataProviderOnlineSimple()
{
}

size_t DataProviderOnlineSimple::numCameras() const
{
  return 1u;
}

bool DataProviderOnlineSimple::sampleFrame()
{
  // Sample (i.e. render) a new frame (+ optic flow map),
  // Fill in the appropriate data in sim_data
  // Compute the optic flow and compute the next latest sampling time in order
  // to guarantee that the displacement is bounded by simulation_max_displacement_successive_frames
  CHECK(renderer_);
  bool is_finished = renderer_->render(ze::secToNanosec(t_),
                                       sim_data_.images[0],
                                       sim_data_.optic_flows[0]);

  if(is_finished)
  {
    return true;
  }

  // Optionally blur the rendered images slightly
  if(FLAGS_simulation_post_gaussian_blur_sigma > 0)
  {
    gaussianBlur(sim_data_.images[0], FLAGS_simulation_post_gaussian_blur_sigma);
  }

  // Adaptive sampling scheme based on predicted brightness change
  if(simulation_adaptive_sampling_method_ == 0)
  {
    // Predict brightness change based on image gradient and optic flow
    const FloatType max_dLdt = maxPredictedAbsBrightnessChange(sim_data_.images[0],
                                                               sim_data_.optic_flows[0]);

    VLOG(1) << "max(|dLdt|) = " << max_dLdt << " logDN/s";

    // Compute next sampling time
    // t_{k+1} = t_k + delta_t where
    // delta_t = lambda / max(|dL/dt|)
    const ze::real_t delta_t = simulation_adaptive_sampling_lambda_ / max_dLdt;
    VLOG(1) << "deltaT = " << 1000.0 * delta_t << " ms";

    next_t_adaptive_ = t_ + delta_t;
  }

  // Adaptive sampling scheme based on optic flow
  else {
    const FloatType max_flow_magnitude = maxMagnitudeOpticFlow(sim_data_.optic_flows[0]);

    VLOG(1) << "max(||optic_flow||) = " << max_flow_magnitude << " px/s";

    // Compute next sampling time
    // t_{k+1} = t_k + delta_t where
    // delta_t = lambda / max(||optic_flow||)
    const ze::real_t delta_t = simulation_adaptive_sampling_lambda_ / max_flow_magnitude;
    VLOG(1) << "deltaT = " << 1000.0 * delta_t << " ms";

    next_t_adaptive_ = t_ + delta_t;
  }

  last_t_frame_ = t_;

  return false;
}

void DataProviderOnlineSimple::setFrameUpdated()
{
  // Set all the frame-related flags to true, and all the IMU-related flags to false
  sim_data_.imu_updated         = false;
  sim_data_.twists_updated      = false;
  sim_data_.poses_updated       = false;
  sim_data_.images_updated      = true;
  sim_data_.depthmaps_updated   = false;
  sim_data_.optic_flows_updated = true;
}

bool DataProviderOnlineSimple::spinOnce()
{
  const ze::real_t next_t_frame = last_t_frame_ + dt_frame_;

  VLOG(2) << "t = " << t_;
  VLOG(2) << "next_t_frame = " << next_t_frame;
  VLOG(2) << "next_t_flow = " << next_t_adaptive_;

  if(next_t_adaptive_ < next_t_frame)
  {
    VLOG(2) << "Sample frame (because of optic flow)";
    t_ = next_t_adaptive_;
  }
  else
  {
    VLOG(2) << "Sample frame (because of minimum framerate)";
    t_ = next_t_frame;
  }
  bool is_finished = sampleFrame();
  setFrameUpdated();

  if(is_finished)
  {
    running_ = false;
    return false;
  }

  if(callback_)
  {
    sim_data_.timestamp = static_cast<Time>(ze::secToNanosec(t_));
    callback_(sim_data_);
  }
  else
  {
    LOG_FIRST_N(WARNING, 1) << "No camera callback registered but measurements available.";
  }
  return true;
}

bool DataProviderOnlineSimple::ok() const
{
  if (!running_)
  {
    VLOG(1) << "Data Provider was paused/terminated.";
    return false;
  }
  return true;
}

} // namespace event_camera_simulator
