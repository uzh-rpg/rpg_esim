#include <ze/common/logging.hpp>
#include <esim/trajectory/imu_factory.hpp>

namespace event_camera_simulator {

ze::ImuSimulator::Ptr loadImuSimulatorFromGflags(const ze::TrajectorySimulator::Ptr& trajectory)
{
  ze::ImuSimulator::Ptr imu;

  const ze::real_t gyr_bias_noise_sigma = 0.0000266;
  const ze::real_t acc_bias_noise_sigma = 0.000433;
  const ze::real_t gyr_noise_sigma = 0.000186;
  const ze::real_t acc_noise_sigma = 0.00186;
  const uint32_t imu_bandwidth_hz = 200;
  const ze::real_t gravity_magnitude = 9.81;

  ze::ImuBiasSimulator::Ptr bias;
  try
  {
    VLOG(1) << "Initialize bias ...";
    bias = std::make_shared<ze::ContinuousBiasSimulator>(
             ze::Vector3::Constant(gyr_bias_noise_sigma),
             ze::Vector3::Constant(acc_bias_noise_sigma),
             trajectory->start(),
             trajectory->end(),
             100); // Results in malloc: (trajectory->end() - trajectory->start()) * imu_bandwidth_hz);
    VLOG(1) << "done.";
  }
  catch (const std::bad_alloc& e)
  {
    LOG(FATAL) << "Could not create bias because number of samples is too high."
               << " Allocation failed: " << e.what();
  }

  VLOG(1) << "Initialize IMU ...";
  imu = std::make_shared<ze::ImuSimulator>(
           trajectory,
           bias,
           ze::RandomVectorSampler<3>::sigmas(ze::Vector3::Constant(acc_noise_sigma)),
           ze::RandomVectorSampler<3>::sigmas(ze::Vector3::Constant(gyr_noise_sigma)),
           imu_bandwidth_hz,
           imu_bandwidth_hz,
           gravity_magnitude);
  VLOG(1) << "done.";

  return imu;
}

} // namespace event_camera_simulator

