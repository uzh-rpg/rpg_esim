#include <esim/esim/simulator.hpp>
#include <esim/visualization/ros_publisher.hpp>
#include <esim/visualization/rosbag_writer.hpp>
#include <esim/visualization/adaptive_sampling_benchmark_publisher.hpp>
#include <esim/visualization/synthetic_optic_flow_publisher.hpp>
#include <esim/data_provider/data_provider_factory.hpp>

#include <glog/logging.h>
#include <gflags/gflags.h>

DEFINE_double(contrast_threshold_pos, 1.0,
              "Contrast threshold (positive)");

DEFINE_double(contrast_threshold_neg, 1.0,
              "Contrast threshold  (negative))");

DEFINE_double(contrast_threshold_sigma_pos, 0.021,
              "Standard deviation of contrast threshold (positive)");

DEFINE_double(contrast_threshold_sigma_neg, 0.021,
              "Standard deviation of contrast threshold  (negative))");

DEFINE_int64(refractory_period_ns, 100000,
             "Refractory period (time during which a pixel cannot fire events just after it fired one), in nanoseconds");

DEFINE_double(exposure_time_ms, 10.0,
              "Exposure time in milliseconds, used to simulate motion blur");

DEFINE_bool(use_log_image, true,
            "Whether to convert images to log images in the preprocessing step.");

DEFINE_double(log_eps, 0.001,
              "Epsilon value used to convert images to log: L = log(eps + I / 255.0).");

DEFINE_bool(simulate_color_events, false,
              "Whether to simulate color events or not (default: false)");

DEFINE_int32(random_seed, 0,
              "Random seed used to generate the trajectories. If set to 0 the current time(0) is taken as seed.");

using namespace event_camera_simulator;

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  if (FLAGS_random_seed == 0) FLAGS_random_seed = (unsigned int) time(0);
  srand(FLAGS_random_seed);

  DataProviderBase::Ptr data_provider_ =
      loadDataProviderFromGflags();
  CHECK(data_provider_);

  EventSimulator::Config event_sim_config;
  event_sim_config.Cp = FLAGS_contrast_threshold_pos;
  event_sim_config.Cm = FLAGS_contrast_threshold_neg;
  event_sim_config.sigma_Cp = FLAGS_contrast_threshold_sigma_pos;
  event_sim_config.sigma_Cm = FLAGS_contrast_threshold_sigma_neg;
  event_sim_config.refractory_period_ns = FLAGS_refractory_period_ns;
  event_sim_config.use_log_image = FLAGS_use_log_image;
  event_sim_config.log_eps = FLAGS_log_eps;
  event_sim_config.simulate_color_events = FLAGS_simulate_color_events;
  std::shared_ptr<Simulator> sim;
  sim.reset(new Simulator(data_provider_->numCameras(),
                          event_sim_config,
                          FLAGS_exposure_time_ms));
  CHECK(sim);

  Publisher::Ptr ros_publisher = std::make_shared<RosPublisher>(data_provider_->numCameras());
  Publisher::Ptr rosbag_writer = RosbagWriter::createBagWriterFromGflags(data_provider_->numCameras());
  Publisher::Ptr adaptive_sampling_benchmark_publisher
      = AdaptiveSamplingBenchmarkPublisher::createFromGflags();

  Publisher::Ptr synthetic_optic_flow_publisher
      = SyntheticOpticFlowPublisher::createFromGflags();

  if(ros_publisher) sim->addPublisher(ros_publisher);
  if(rosbag_writer) sim->addPublisher(rosbag_writer);
  if(adaptive_sampling_benchmark_publisher) sim->addPublisher(adaptive_sampling_benchmark_publisher);
  if(synthetic_optic_flow_publisher) sim->addPublisher(synthetic_optic_flow_publisher);

  data_provider_->registerCallback(
        std::bind(&Simulator::dataProviderCallback, sim.get(),
                  std::placeholders::_1));

  data_provider_->spin();

}
