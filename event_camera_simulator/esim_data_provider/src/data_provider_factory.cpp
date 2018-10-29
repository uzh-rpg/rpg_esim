#include <ze/common/logging.hpp>
#include <esim/data_provider/data_provider_factory.hpp>
#include <esim/data_provider/data_provider_base.hpp>
#include <esim/data_provider/data_provider_online_render.hpp>
#include <esim/data_provider/data_provider_online_simple.hpp>
#include <esim/data_provider/data_provider_from_folder.hpp>
#include <esim/data_provider/data_provider_rosbag.hpp>

DEFINE_int32(data_source, 0, " 0: Online renderer");

DEFINE_double(simulation_minimum_framerate, 30.0,
            "Minimum frame rate, in Hz"
            "Especially useful when the event rate is low, to guarantee"
            "that frames are still published at a minimum framerate");

DEFINE_double(simulation_imu_rate, 1000.0,
              "Fixed IMU sampling frequency, in Hz");

DEFINE_int32(simulation_adaptive_sampling_method, 0,
              "Method to use for adaptive sampling."
              "0: based on predicted absolute brightness change"
              "1: based on optic flow");

DEFINE_double(simulation_adaptive_sampling_lambda, 0.5,
       "Parameter that controls the behavior of the adaptive sampling method."
       "The meaning of this value depends on the adaptive sampling method used:"
       "...based on predicted absolute brightness change: deltaT = lambda / max(|dL/dt|)"
       "...based on optic flow: deltaT = lambda \ max(||du/dt||) where du/dt denotes the 2D optic flow field.");

DEFINE_string(path_to_data_folder, "",
       "Path to folder containing the data.");


// Parameters for the DataProviderRosbag
DEFINE_string(bag_filename, "dataset.bag", "Name of bagfile from which to read.");
DEFINE_string(topic_cam0, "/cam0/image_raw", "");
DEFINE_string(topic_cam1, "/cam1/image_raw", "");
DEFINE_string(topic_cam2, "/cam2/image_raw", "");
DEFINE_string(topic_cam3, "/cam3/image_raw", "");
DEFINE_uint64(num_cams, 1, "Number of normal cameras to read from rosbag.");

namespace event_camera_simulator {

DataProviderBase::Ptr loadDataProviderFromGflags()
{
  // Create data provider.
  DataProviderBase::Ptr data_provider;
  switch (FLAGS_data_source)
  {
    case 0: // Online Renderer for Moving 3D Camera Rig with IMU
    {
      data_provider.reset(
            new DataProviderOnlineMoving3DCameraRig(FLAGS_simulation_minimum_framerate,
                                         FLAGS_simulation_imu_rate,
                                         FLAGS_simulation_adaptive_sampling_method,
                                         FLAGS_simulation_adaptive_sampling_lambda));
      break;
    }
    case 1: // Online Renderer Simple
    {
      data_provider.reset(
            new DataProviderOnlineSimple(FLAGS_simulation_minimum_framerate,
                                         FLAGS_simulation_adaptive_sampling_method,
                                         FLAGS_simulation_adaptive_sampling_lambda));
      break;
    }
    case 2: // Read data from a folder
    {
      data_provider.reset(
            new DataProviderFromFolder(FLAGS_path_to_data_folder));
      break;
    }
    case 3: // Read data (images) from a rosbag
    {
        CHECK_LE(FLAGS_num_cams, 4u);
        CHECK_EQ(FLAGS_num_cams, 1u) << "Only one camera is supported currently";

        // Fill camera topics.
        std::map<std::string, size_t> cam_topics;
        if (FLAGS_num_cams >= 1) cam_topics[FLAGS_topic_cam0] = 0;
        if (FLAGS_num_cams >= 2) cam_topics[FLAGS_topic_cam1] = 1;
        if (FLAGS_num_cams >= 3) cam_topics[FLAGS_topic_cam2] = 2;
        if (FLAGS_num_cams >= 4) cam_topics[FLAGS_topic_cam3] = 3;
        data_provider.reset(
              new DataProviderRosbag(FLAGS_bag_filename,
                                     cam_topics));
        break;
    }
    default:
    {
      LOG(FATAL) << "Data source not known.";
      break;
    }
  }

  return data_provider;
}

} // namespace ze
