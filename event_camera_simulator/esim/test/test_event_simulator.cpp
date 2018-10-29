#include <esim/esim/event_simulator.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <fstream>
#include <ze/common/file_utils.hpp>
#include <ze/common/path_utils.hpp>
#include <ze/common/string_utils.hpp>
#include <ze/common/file_utils.hpp>

#include <ze/common/time_conversions.hpp>
#include <ze/matplotlib/matplotlibcpp.hpp>
#include <opencv2/highgui/highgui.hpp>

#define USE_OPENCV

namespace event_camera_simulator
{

class CSVImageLoader
{
public:
  CSVImageLoader(const std::string& path_to_data_folder)
    : path_to_data_folder_(path_to_data_folder)
  {
    images_in_str_.open(ze::joinPath(path_to_data_folder, "images.csv"));
    CHECK(images_in_str_.is_open());
  }

  bool next(int64_t& stamp, Image& img)
  {
    std::string line;
    if(!getline(images_in_str_, line))
    {
      LOG(INFO) << "Finished reading all the images in the folder";
      return false;
    }

    if('%' != line.at(0) && '#' != line.at(0))
    {
      std::vector<std::string> items = ze::splitString(line, delimiter_);
      stamp = std::stoll(items[0]);

      const std::string& path_to_image
          = ze::joinPath(path_to_data_folder_, "frame", "cam_0", items[1]);

      img = cv::imread(path_to_image, 0);
      CHECK(img.data) << "Error loading image: " << path_to_image;

      return true;
    }
    else
    {
      return next(stamp, img);
    }
  }

private:
  std::ifstream images_in_str_;
  const char delimiter_{','};
  std::string path_to_data_folder_;
};

} // namespace event_camera_simulator

std::string getTestDataDir(const std::string& dataset_name)
{
  using namespace ze;

  const char* datapath_dir = std::getenv("ESIM_TEST_DATA_PATH");
  CHECK(datapath_dir != nullptr)
      << "Did you download the esim_test_data repository and set "
      << "the ESIM_TEST_DATA_PATH environment variable?";

  std::string path(datapath_dir);
  CHECK(isDir(path)) << "Folder does not exist: " << path;
  path = path + "/data/" + dataset_name;
  CHECK(isDir(path)) << "Dataset does not exist: " << path;
  return path;
}

TEST(EventSimulator, testImageReconstruction)
{
  using namespace event_camera_simulator;

  // Load image sequence from folder
  const std::string path_to_data_folder = getTestDataDir("planar_carpet");
  CSVImageLoader reader(path_to_data_folder);

  EventSimulator::Config event_sim_config;
  event_sim_config.Cp = 0.05;
  event_sim_config.Cm = 0.03;
  event_sim_config.sigma_Cp = 0;
  event_sim_config.sigma_Cm = 0;
  event_sim_config.use_log_image = true;
  event_sim_config.log_eps = 0.001;
  EventSimulator simulator(event_sim_config);

  LOG(INFO) << "Testing event camera simulator with C+ = " << event_sim_config.Cp
            << ", C- = " << event_sim_config.Cm;

  const ImageFloatType max_reconstruction_error
      = std::max(event_sim_config.Cp, event_sim_config.Cm);

  bool is_first_image = true;
  Image I, L, L_reconstructed;
  int64_t stamp;
  while(reader.next(stamp, I))
  {
    I.convertTo(I, cv::DataType<ImageFloatType>::type, 1./255.);
    cv::log(event_sim_config.log_eps + I, L);

    if(is_first_image)
    {
      // Initialize reconstructed image from the ground truth image
      L_reconstructed = L.clone();
      is_first_image = false;
    }

    Events events = simulator.imageCallback(I, stamp);

    // Reconstruct next image from previous one using the events in between
    for(const Event& e : events)
    {
      ImageFloatType pol = e.pol ? 1. : -1.;
      const ImageFloatType C = e.pol ? event_sim_config.Cp : event_sim_config.Cm;
      L_reconstructed(e.y,e.x) += pol * C;
    }

    // Check that the reconstruction error is bounded by the contrast thresholds
    for(int y=0; y<I.rows; ++y)
    {
      for(int x=0; x<I.cols; ++x)
      {
        const ImageFloatType reconstruction_error = std::fabs(L_reconstructed(y,x) - L(y,x));
        VLOG_EVERY_N(2, I.rows * I.cols) << reconstruction_error;
        EXPECT_LE(reconstruction_error, max_reconstruction_error);
      }
    }

#ifdef USE_OPENCV
    const ImageFloatType vmin = std::log(event_sim_config.log_eps);
    const ImageFloatType vmax = std::log(1.0 + event_sim_config.log_eps);
    cv::Mat disp = 255.0 * (L_reconstructed - vmin) / (vmax - vmin);
    disp.convertTo(disp, CV_8U);
    cv::imshow("Reconstructed", disp);
    cv::waitKey(1);
#endif
  }
}


TEST(EventSimulator, testEvolutionReconstructionError)
{
  using namespace event_camera_simulator;

  // Load image sequence from folder
  const std::string path_to_data_folder = getTestDataDir("planar_carpet");
  CSVImageLoader reader(path_to_data_folder);

  EventSimulator::Config event_sim_config;
  event_sim_config.Cp = 0.5;
  event_sim_config.Cm = event_sim_config.Cp;
  event_sim_config.sigma_Cp = 0;
  event_sim_config.sigma_Cm = event_sim_config.sigma_Cp;
  event_sim_config.use_log_image = true;
  event_sim_config.log_eps = 0.001;
  EventSimulator simulator(event_sim_config);
  const double contrast_bias = 0.0;

  LOG(INFO) << "Testing event camera simulator with C+ = " << event_sim_config.Cp
            << ", C- = " << event_sim_config.Cm;

  std::vector<ze::real_t> times, mean_errors, stddev_errors;
  bool is_first_image = true;
  Image I, L, L_reconstructed;
  int64_t stamp;
  while(reader.next(stamp, I))
  {
    LOG_EVERY_N(INFO, 50) << "t = " << ze::nanosecToSecTrunc(stamp) << " s";
    I.convertTo(I, cv::DataType<ImageFloatType>::type, 1./255.);
    cv::log(event_sim_config.log_eps + I, L);

    if(is_first_image)
    {
      // Initialize reconstructed image from the ground truth image
      L_reconstructed = L.clone();
      is_first_image = false;
    }

    Events events = simulator.imageCallback(I, stamp);

    // Reconstruct next image from previous one using the events in between
    for(const Event& e : events)
    {
      ImageFloatType pol = e.pol ? 1. : -1.;
      ImageFloatType C = e.pol ? event_sim_config.Cp : event_sim_config.Cm;
      C += contrast_bias;
      L_reconstructed(e.y,e.x) += pol * C;
    }

    // Compute the mean and average reconstruction error over the whole image
    Image error;
    cv::absdiff(L, L_reconstructed, error);
    cv::Scalar mean_error, stddev_error;
    cv::meanStdDev(error, mean_error, stddev_error);
    VLOG(1) << "Mean error: " << mean_error
            << ", Stddev: "   << stddev_error;

    times.push_back(ze::nanosecToSecTrunc(stamp));
    mean_errors.push_back(mean_error[0]);
    stddev_errors.push_back(stddev_error[0]);

#ifdef USE_OPENCV
    const ImageFloatType vmin = std::log(event_sim_config.log_eps);
    const ImageFloatType vmax = std::log(1.0 + event_sim_config.log_eps);
    cv::Mat disp = 255.0 * (L_reconstructed - vmin) / (vmax - vmin);
    disp.convertTo(disp, CV_8U);
    cv::imshow("Reconstructed", disp);
    cv::waitKey(1);
#endif
  }

  // Plot the mean and stddev reconstruction error over time
  ze::plt::plot(times, mean_errors, "r");
  ze::plt::plot(times, stddev_errors, "b--");
  std::stringstream title;
  title << "C = "       << event_sim_config.Cp
        << ", sigma = " << event_sim_config.sigma_Cp
        << ", bias = "  << contrast_bias;
  ze::plt::title(title.str());
  ze::plt::xlabel("Time (s)");
  ze::plt::ylabel("Mean / Stddev reconstruction error");
  ze::plt::grid(true);
  ze::plt::save("/tmp/evolution_reconstruction_error.pdf");
  ze::plt::show();
}

ZE_UNITTEST_ENTRYPOINT
