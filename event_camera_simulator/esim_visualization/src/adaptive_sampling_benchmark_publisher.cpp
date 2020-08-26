#include <esim/visualization/adaptive_sampling_benchmark_publisher.hpp>
#include <esim/common/utils.hpp>
#include <ze/common/path_utils.hpp>
#include <ze/common/file_utils.hpp>
#include <ze/common/time_conversions.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(adaptive_sampling_benchmark_folder, "",
"Folder in which to output the results.");

DEFINE_string(adaptive_sampling_benchmark_pixels_to_record_file, "",
"File containing the pixel locations to record.");

namespace event_camera_simulator {

AdaptiveSamplingBenchmarkPublisher::AdaptiveSamplingBenchmarkPublisher(const std::string& benchmark_folder,
                                                                       const std::string& pixels_to_record_filename)
  : image_index_(0)
{
  ze::openOutputFileStream(ze::joinPath(benchmark_folder, "events.txt"),
                           &events_file_);

  ze::openOutputFileStream(ze::joinPath(benchmark_folder, "images.txt"),
                           &images_file_);

  ze::openOutputFileStream(ze::joinPath(benchmark_folder, "pixel_intensities.txt"),
                           &pixel_intensities_file_);

  ze::openOutputFileStream(ze::joinPath(benchmark_folder, "optic_flows.txt"),
                           &optic_flows_file_);

  // Load and parse the file containing the list of pixel locations
  // whose intensity values to record
  std::ifstream pixels_to_record_file;
  if(pixels_to_record_filename != "")
  {
    ze::openFileStream(pixels_to_record_filename, &pixels_to_record_file);

    int x, y;
    LOG(INFO) << "Pixels that will be recorded: ";
    while(pixels_to_record_file >> x >> y)
    {
      LOG(INFO) << x << " , " << y;
      pixels_to_record_.push_back(PixelLocation(x,y));
    }
  }
}

Publisher::Ptr AdaptiveSamplingBenchmarkPublisher::createFromGflags()
{
  if(FLAGS_adaptive_sampling_benchmark_folder == "")
  {
    LOG(WARNING) << "Empty benchmark folder string: will not write benchmark files";
    return nullptr;
  }

  return std::make_shared<AdaptiveSamplingBenchmarkPublisher>(FLAGS_adaptive_sampling_benchmark_folder,
                                                              FLAGS_adaptive_sampling_benchmark_pixels_to_record_file);
}

AdaptiveSamplingBenchmarkPublisher::~AdaptiveSamplingBenchmarkPublisher()
{
  // finish writing files
  events_file_.close();
  images_file_.close();
  pixel_intensities_file_.close();
  optic_flows_file_.close();
}

void AdaptiveSamplingBenchmarkPublisher::imageCallback(const ImagePtrVector& images, Time t)
{
  CHECK_EQ(images.size(), 1);
  images_file_ << t << std::endl;

  ImagePtr img = images[0];
  cv::Mat img_8bit;
  img->convertTo(img_8bit, CV_8U, 255);

  if(image_index_ == 0)
  {
    static const std::vector<int> compression_params = {cv::IMWRITE_PNG_COMPRESSION, 0};

    std::stringstream ss;
    ss << ze::joinPath(FLAGS_adaptive_sampling_benchmark_folder, "image_");
    ss << image_index_ << ".png";

    LOG(INFO) << ss.str();
    cv::imwrite(ss.str(), img_8bit, compression_params);
  }

  for(const PixelLocation& pixel_loc : pixels_to_record_)
  {
    // write line in the form "x y I(x,y)"
    const int x = pixel_loc.first;
    const int y = pixel_loc.second;
    pixel_intensities_file_ << x << " "
                          << y << " "
                          << (*images[0])(y,x) << std::endl;
  }

  image_index_++;
}

void AdaptiveSamplingBenchmarkPublisher::opticFlowCallback(const OpticFlowPtrVector& optic_flows, Time t)
{
  CHECK_EQ(optic_flows.size(), 1);
  for(const PixelLocation& pixel_loc : pixels_to_record_)
  {
    // write line in the form "x y optic_flow(x,y)[0] optic_flow(x,y)[1]"
    const int x = pixel_loc.first;
    const int y = pixel_loc.second;
    optic_flows_file_ << x << " "
                      << y << " "
                      << (*optic_flows[0])(y,x)[0] << " "
                      << (*optic_flows[0])(y,x)[1]
                      << std::endl;
  }
}


void AdaptiveSamplingBenchmarkPublisher::eventsCallback(const EventsVector& events)
{
  CHECK_EQ(events.size(), 1);

  for(const Event& e : events[0])
  {
     events_file_ << e.t << " " << e.x << " " << e.y << " " << (e.pol? 1 : 0) << std::endl;
  }
}

} // namespace event_camera_simulator
