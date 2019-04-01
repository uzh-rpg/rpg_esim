#include <esim/data_provider/data_provider_online_render.hpp>
#include <esim/trajectory/trajectory_factory.hpp>
#include <esim/trajectory/imu_factory.hpp>
#include <esim/data_provider/renderer_factory.hpp>
#include <esim/common/utils.hpp>

#include <ze/cameras/camera_rig.hpp>
#include <ze/common/time_conversions.hpp>
#include <ze/common/timer_collection.hpp>

DECLARE_TIMER(TimerDataProvider, timers_data_provider_,
              render,
              optic_flow,
              sample_frame,
              sample_imu
              );

DEFINE_double(simulation_post_gaussian_blur_sigma, 0,
              "If sigma > 0, Gaussian blur the renderered images"
              "with a Gaussian filter standard deviation sigma.");

namespace event_camera_simulator {

DataProviderOnlineMoving3DCameraRig::DataProviderOnlineMoving3DCameraRig(ze::real_t simulation_minimum_framerate,
                                                   ze::real_t simulation_imu_rate,
                                                   int simulation_adaptive_sampling_method,
                                                   ze::real_t simulation_adaptive_sampling_lambda)
  : DataProviderBase(DataProviderType::RendererOnline),
    simulation_minimum_framerate_(simulation_minimum_framerate),
    simulation_imu_rate_(simulation_imu_rate),
    simulation_adaptive_sampling_method_(simulation_adaptive_sampling_method),
    simulation_adaptive_sampling_lambda_(simulation_adaptive_sampling_lambda),
    dt_imu_(1./simulation_imu_rate),
    dt_frame_(1./simulation_minimum_framerate)
{
  CHECK(simulation_adaptive_sampling_method == 0
        || simulation_adaptive_sampling_method == 1);

  std::tie(trajectory_, trajectory_dyn_obj_) = loadTrajectorySimulatorFromGflags();
  imu_ = loadImuSimulatorFromGflags(trajectory_);
  camera_rig_ = ze::cameraRigFromGflags();

  // Compute Field-of-Views for information
  const ze::Camera::Ptr camera = camera_rig_->atShared(0);
  const int width = camera->width();
  const int height = camera->height();

  const ze::real_t horizontal_fov =
      180.0 / CV_PI *
      std::acos(camera->backProject(ze::Keypoint(0,height/2)).dot(
                camera->backProject(ze::Keypoint(width-1,height/2))));

  const ze::real_t vertical_fov =
      180.0 / CV_PI *
      std::acos(camera->backProject(ze::Keypoint(width/2,0)).dot(
                camera->backProject(ze::Keypoint(width/2,height-1))));

  const ze::real_t diagonal_fov =
      180.0 / CV_PI *
      std::acos(camera->backProject(ze::Keypoint(0,0)).dot(
                camera->backProject(ze::Keypoint(width-1,height-1))));

  LOG(INFO) << "Horizontal FOV: " << horizontal_fov << " deg";
  LOG(INFO) << "Vertical FOV: " << vertical_fov << " deg";
  LOG(INFO) << "Diagonal FOV: " << diagonal_fov << " deg";

  for(size_t i=0; i<camera_rig_->size(); ++i)
  {
    renderers_.push_back(std::move(loadRendererFromGflags()));
    renderers_[i]->setCamera(camera_rig_->atShared(i));

    optic_flow_helpers_.emplace_back(std::make_shared<OpticFlowHelper>(camera_rig_->atShared(i)));
  }

  const size_t num_cameras = camera_rig_->size();

  sim_data_.groundtruth.T_W_Cs.resize(num_cameras);
  sim_data_.groundtruth.angular_velocities_.resize(num_cameras);
  sim_data_.groundtruth.linear_velocities_.resize(num_cameras);
  for(size_t i=0; i<num_cameras; ++i)
  {
    const cv::Size size = cv::Size(camera_rig_->at(i).width(),
                                   camera_rig_->at(i).height());

    sim_data_.images.emplace_back(ColorImagePtr(new ColorImage(size)));
    sim_data_.depthmaps.emplace_back(DepthmapPtr(new Depthmap(size)));
    sim_data_.optic_flows.emplace_back(OpticFlowPtr(new OpticFlow(size)));

    sim_data_.images[i]->setTo(0);
    sim_data_.depthmaps[i]->setTo(0);
    sim_data_.optic_flows[i]->setTo(0);
  }

  for(size_t i=0; i<trajectory_dyn_obj_.size(); i++)
  {
      sim_data_.groundtruth.T_W_OBJ_.push_back(Transformation());
      sim_data_.groundtruth.linear_velocity_obj_.push_back(LinearVelocity());
      sim_data_.groundtruth.angular_velocity_obj_.push_back(AngularVelocity());
  }

  sim_data_.camera_rig = camera_rig_;
  t_ = trajectory_->start();

  // At the initial time, we sample everything (IMU + frames)
  sampleImu();
  sampleFrame();
  setAllUpdated();
  if(callback_)
  {
    callback_(sim_data_);
  }
}

DataProviderOnlineMoving3DCameraRig::~DataProviderOnlineMoving3DCameraRig()
{
  timers_data_provider_.saveToFile("/tmp", "data_provider_online_render.csv");
}

size_t DataProviderOnlineMoving3DCameraRig::numCameras() const
{
  if(camera_rig_)
  {
    return camera_rig_->size();
  }
  else
  {
    return 0;
  }
}

void DataProviderOnlineMoving3DCameraRig::updateGroundtruth()
{
  const Transformation T_W_B = trajectory_->T_W_B(t_);
  sim_data_.groundtruth.T_W_B = T_W_B;

  const AngularVelocity omega_B = trajectory_->angularVelocity_B(t_);
  const LinearVelocity v_B_W = trajectory_->velocity_W(t_);
  for(size_t i=0; i<camera_rig_->size(); ++i)
  {
    sim_data_.groundtruth.T_W_Cs[i] =
        sim_data_.groundtruth.T_W_B * camera_rig_->T_B_C(i);

    const LinearVelocity v_W = v_B_W + T_W_B.getRotation().rotate(
          ze::skewSymmetric(omega_B) * camera_rig_->T_B_C(i).getPosition());

    const AngularVelocity omega_C = camera_rig_->T_C_B(i).getRotation().rotate(omega_B);
    const LinearVelocity v_C = (camera_rig_->T_C_B(i) * T_W_B.inverse()).getRotation().rotate(v_W);

    sim_data_.groundtruth.angular_velocities_[i] = omega_C;
    sim_data_.groundtruth.linear_velocities_[i] = v_C;
  }

  // update poses of dynamic objects
  for (size_t i = 0; i < trajectory_dyn_obj_.size(); i++)
  {
      sim_data_.groundtruth.T_W_OBJ_[i] = trajectory_dyn_obj_[i]->T_W_B(t_);
      sim_data_.groundtruth.linear_velocity_obj_[i] = trajectory_dyn_obj_[i]->velocity_W(t_);
      sim_data_.groundtruth.angular_velocity_obj_[i] = sim_data_.groundtruth.T_W_OBJ_[i].getRotation().rotate(trajectory_dyn_obj_[i]->angularVelocity_B(t_));
  }
}

void DataProviderOnlineMoving3DCameraRig::sampleImu()
{
  // Sample new IMU (+ poses, twists, etc.) values,
  // Fill in the approriate data in sim_data
  auto t = timers_data_provider_[::TimerDataProvider::sample_imu].timeScope();

  if(t_ > trajectory_->end())
  {
    return;
  }

  updateGroundtruth();
  sim_data_.specific_force_corrupted = imu_->specificForceCorrupted(t_);
  sim_data_.angular_velocity_corrupted = imu_->angularVelocityCorrupted(t_);
  sim_data_.groundtruth.acc_bias = imu_->bias()->accelerometer(t_);
  sim_data_.groundtruth.gyr_bias = imu_->bias()->gyroscope(t_);

  last_t_imu_ = t_;
}

void DataProviderOnlineMoving3DCameraRig::sampleFrame()
{
  // Sample (i.e. render) a new frame (+ depth map),
  // Fill in the appropriate data in sim_data
  // Compute the optic flow and compute the next latest sampling time in order
  // to guarantee that the displacement is bounded by simulation_max_displacement_successive_frames
  auto t = timers_data_provider_[::TimerDataProvider::sample_frame].timeScope();

  if(t_ > trajectory_->end())
  {
    return;
  }

  updateGroundtruth();

  {
    auto t = timers_data_provider_[::TimerDataProvider::render].timeScope();

    for(size_t i=0; i<camera_rig_->size(); ++i)
    {
      CHECK(renderers_[i]);

      // if the renderer provides a function to compute the optic
      // flow (for example, the OpenGL renderer which implements
      // optic flow computation efficiently using a shader), use that.
      // otherwise, compute the optic flow ourselves using the renderer depthmap
      if(renderers_[i]->canComputeOpticFlow())
      {
        renderers_[i]->renderWithFlow(sim_data_.groundtruth.T_W_B * camera_rig_->T_B_C(i),
                              sim_data_.groundtruth.linear_velocities_[i],
                              sim_data_.groundtruth.angular_velocities_[i],
                              sim_data_.groundtruth.T_W_OBJ_,
                              sim_data_.groundtruth.linear_velocity_obj_,
                              sim_data_.groundtruth.angular_velocity_obj_,
                              sim_data_.images[i],
                              sim_data_.depthmaps[i],
                              sim_data_.optic_flows[i]);
      }
      else
      {
        renderers_[i]->render(sim_data_.groundtruth.T_W_B * camera_rig_->T_B_C(i),
                              sim_data_.groundtruth.T_W_OBJ_,
                              sim_data_.images[i],
                              sim_data_.depthmaps[i]);
      }

      // Optionally blur the rendered images slightly
      if(FLAGS_simulation_post_gaussian_blur_sigma > 0)
      {
        gaussianBlur(sim_data_.images[i], FLAGS_simulation_post_gaussian_blur_sigma);
      }
    }
  }

  {
    auto t = timers_data_provider_[::TimerDataProvider::optic_flow].timeScope();
    for(size_t i=0; i<camera_rig_->size(); ++i)
    {
      CHECK(optic_flow_helpers_[i]);
      if(!renderers_[i]->canComputeOpticFlow())
      {
        optic_flow_helpers_[i]->computeFromDepthAndTwist(sim_data_.groundtruth.angular_velocities_[i],
                                                         sim_data_.groundtruth.linear_velocities_[i],
                                                         sim_data_.depthmaps[i], sim_data_.optic_flows[i]);

      }
    }
  }

  // Adaptive sampling scheme based on predicted brightness change
  if(simulation_adaptive_sampling_method_ == 0)
  {
    // Predict brightness change based on image gradient and optic flow
    std::vector<FloatType> max_dLdts;
    for(size_t i=0; i<camera_rig_->size(); ++i)
    {
      max_dLdts.push_back(
            maxPredictedAbsBrightnessChange(sim_data_.images[i],
                                            sim_data_.optic_flows[i]));
    }
    const FloatType max_dLdt = *std::max_element(max_dLdts.begin(),
                                                 max_dLdts.end());

    VLOG(1) << "max(|dLdt|) = " << max_dLdt << " logDN/s";

    // Compute next sampling time
    // t_{k+1} = t_k + delta_t where
    // delta_t = lambda / max(|dL/dt|)
    const ze::real_t delta_t = simulation_adaptive_sampling_lambda_ / max_dLdt;
    VLOG(1) << "deltaT = " << 1000.0 * delta_t << " ms";

    next_t_flow_ = t_ + delta_t;
  }

  // Adaptive sampling scheme based on optic flow
  else {
    std::vector<FloatType> max_flow_magnitudes;
    for(size_t i=0; i<camera_rig_->size(); ++i)
    {
      max_flow_magnitudes.push_back(maxMagnitudeOpticFlow(sim_data_.optic_flows[i]));
    }
    const FloatType max_flow_magnitude = *std::max_element(max_flow_magnitudes.begin(), max_flow_magnitudes.end());

    VLOG(1) << "max(||optic_flow||) = " << max_flow_magnitude << " px/s";

    // Compute next sampling time
    // t_{k+1} = t_k + delta_t where
    // delta_t = lambda / max(||optic_flow||)
    const ze::real_t delta_t = simulation_adaptive_sampling_lambda_ / max_flow_magnitude;
    VLOG(1) << "deltaT = " << 1000.0 * delta_t << " ms";

    next_t_flow_ = t_ + delta_t;
  }

  last_t_frame_ = t_;
}

void DataProviderOnlineMoving3DCameraRig::setImuUpdated()
{
  // Set all the IMU-related flags to true, and all the frame-related flags to false
  sim_data_.imu_updated         = true;
  sim_data_.twists_updated      = true;
  sim_data_.poses_updated       = true;
  sim_data_.images_updated      = false;
  sim_data_.depthmaps_updated   = false;
  sim_data_.optic_flows_updated = false;
}

void DataProviderOnlineMoving3DCameraRig::setFrameUpdated()
{
  // Set all the frame-related flags to true, and all the IMU-related flags to false
  sim_data_.imu_updated         = false;
  sim_data_.twists_updated      = true;
  sim_data_.poses_updated       = true;
  sim_data_.images_updated      = true;
  sim_data_.depthmaps_updated   = true;
  sim_data_.optic_flows_updated = true;
}

void DataProviderOnlineMoving3DCameraRig::setAllUpdated()
{
  // Set all the flags to true to indicated everything has been changed
  sim_data_.imu_updated         = true;
  sim_data_.twists_updated      = true;
  sim_data_.poses_updated       = true;
  sim_data_.images_updated      = true;
  sim_data_.depthmaps_updated   = true;
  sim_data_.optic_flows_updated = true;
}

bool DataProviderOnlineMoving3DCameraRig::spinOnce()
{
  /* At what time do we need to sample "something" (and what "something"?)
     We choose the next sampling time by considering the following constraints:

     1. The IMU sampling rate must be constant (typically, from 200 Hz to 1 kHz)
     2. The frame sampling rate must be greater than a minimum value (typically, from 20 Hz to 100 Hz)
     3. The pixel displacement between two successive frames must be lower than a threshold.

     * If the next sample needs to be an IMU sample, we just sample a new IMU value, without regenerating a frame,
     and transmit only the new IMU (+ poses, twists, IMU bias) to the publisher by setting the approriate "data_changed" flags in the
     sim_data structure.

     * If the next sample needs to be a frame sample, we render a new frame (+ depth map + optic flow), but not a new IMU value.
       This can happen either because
       (i) a new frame must be rendered in order to guarantee that the displacement between frames is bounded, or
       (ii) the frame rate should be higher than a minimum (used-defined) value.

     At the beginning of time (t_ = trajectory_->start()), we sample everything (IMU + frame).
   */

  const ze::real_t next_t_imu = last_t_imu_ + dt_imu_;
  const ze::real_t next_t_frame = last_t_frame_ + dt_frame_;

  VLOG(2) << "t = " << t_;
  VLOG(2) << "next_t_imu = " << next_t_imu;
  VLOG(2) << "next_t_frame = " << next_t_frame;
  VLOG(2) << "next_t_flow = " << next_t_flow_;

  if(next_t_imu < next_t_frame && next_t_imu < next_t_flow_)
  {
    VLOG(2) << "Sample IMU";
    t_ = next_t_imu;
    sampleImu();
    setImuUpdated();
  }
  else if(next_t_flow_ < next_t_imu && next_t_flow_ < next_t_frame)
  {
    VLOG(2) << "Sample frame (because of optic flow)";
    t_ = next_t_flow_;
    sampleFrame();
    setFrameUpdated();
  }
  else if(next_t_frame < next_t_imu && next_t_frame < next_t_flow_)
  {
    VLOG(2) << "Sample frame (because of minimum framerate)";
    t_ = next_t_frame;
    sampleFrame();
    setFrameUpdated();
  }
  else
  {
    VLOG(2) << "Sample IMU and frame";
    t_ = next_t_frame;
    // In that case, we sample everything
    sampleImu();
    sampleFrame();
    setAllUpdated();
  }

  if(t_ > trajectory_->end())
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

bool DataProviderOnlineMoving3DCameraRig::ok() const
{
  if (!running_)
  {
    VLOG(1) << "Data Provider was paused/terminated.";
    return false;
  }
  return true;
}

} // namespace event_camera_simulator
