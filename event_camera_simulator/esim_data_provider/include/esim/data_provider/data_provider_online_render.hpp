#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ze/common/macros.hpp>
#include <ze/common/types.hpp>
#include <esim/data_provider/data_provider_base.hpp>
#include <esim/rendering/renderer_base.hpp>
#include <esim/common/utils.hpp>

#include <ze/vi_simulation/trajectory_simulator.hpp>
#include <ze/vi_simulation/imu_simulator.hpp>
#include <ze/cameras/camera_rig.hpp>


namespace event_camera_simulator {

/**
 * Online data provider intended to simulate a camera rig composed of multiple
 * cameras rigidly attached together, along with an Inertial Measurement Unit (IMU).
 *
 * The camera rig follows a camera trajectory in 3D.
 */
class DataProviderOnlineMoving3DCameraRig : public DataProviderBase
{
public:
  DataProviderOnlineMoving3DCameraRig(ze::real_t simulation_minimum_framerate,
                           ze::real_t simulation_imu_rate,
                           int simulation_adaptive_sampling_method,
                           ze::real_t simulation_adaptive_sampling_lambda);

  virtual ~DataProviderOnlineMoving3DCameraRig();

  virtual bool spinOnce() override;

  virtual bool ok() const override;

  size_t numCameras() const override;

private:

  void updateGroundtruth();
  void sampleImu();
  void sampleFrame();

  void setImuUpdated();
  void setFrameUpdated();
  void setAllUpdated();

  std::vector<Renderer::Ptr> renderers_;
  std::vector<OpticFlowHelper::Ptr> optic_flow_helpers_;
  ze::TrajectorySimulator::Ptr trajectory_;
  ze::ImuSimulator::Ptr imu_;
  SimulatorData sim_data_;

  ze::real_t t_;
  ze::real_t last_t_frame_; // latest next sampling time in order to guarantee the IMU rate
  ze::real_t last_t_imu_;   // latest next sampling time in order to guarantee the minimum frame rate
  ze::real_t next_t_flow_;  // latest next sampling time in order to guarantee that the max pixel displacement since the last frame is bounded
  ze::real_t dt_imu_;
  ze::real_t dt_frame_;

  ze::real_t simulation_minimum_framerate_;
  ze::real_t simulation_imu_rate_;
  int simulation_adaptive_sampling_method_;
  ze::real_t simulation_adaptive_sampling_lambda_;

  // dynamic objects
  std::vector<ze::TrajectorySimulator::Ptr> trajectory_dyn_obj_;
};

} // namespace event_camera_simulator
