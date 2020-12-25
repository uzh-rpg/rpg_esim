#include <esim/visualization/synthetic_optic_flow_publisher.hpp>
#include <esim/common/utils.hpp>
#include <ze/common/path_utils.hpp>
#include <ze/common/file_utils.hpp>
#include <ze/common/time_conversions.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(synthetic_optic_flow_output_folder, "",
"Folder in which to output the events.");

namespace event_camera_simulator {

/**
 * This publisher was designed with the purpose of generating simulation data
 * with ground truth labels, for the task of optic flow estimation.
 *
 * It assumes that it will receive a relatively small sequence of events (corresponding, for example,
 * to all the events in between two frames), and will write all the events to disk in its destructor,
 * in three forms:
 *   - an "events.txt" file that contains all the events in "t x y pol" format (one event per line)
 *   - an "event_count.png" image that whose first two channels contain the counts of the positive (resp. negative) event counts at each pixel
 *   - two "timestamps images" in which each pixel contains the timestamp at the last event that fell on the pixel.
 *     (since the timestamp is a floating point value, it is split in 3 8-bit values so that the timestamp images
 *      can be saved in a single 3-channel image).
 */
SyntheticOpticFlowPublisher::SyntheticOpticFlowPublisher(const std::string& output_folder)
  : output_folder_(output_folder)
{
  ze::openOutputFileStream(ze::joinPath(output_folder, "events.txt"),
                           &events_file_);
}

Publisher::Ptr SyntheticOpticFlowPublisher::createFromGflags()
{
  if(FLAGS_synthetic_optic_flow_output_folder == "")
  {
    LOG(WARNING) << "Empty output folder string: will not write synthetic optic flow files";
    return nullptr;
  }

  return std::make_shared<SyntheticOpticFlowPublisher>(FLAGS_synthetic_optic_flow_output_folder);
}

SyntheticOpticFlowPublisher::~SyntheticOpticFlowPublisher()
{
  // Create an event count image using all the events collected
  cv::Mat event_count_image = cv::Mat::zeros(sensor_size_, CV_8UC3);

  // Create two event timestamps images using all the events collected
  cv::Mat timestamps_pos = cv::Mat::zeros(sensor_size_, CV_8UC3);
  cv::Mat timestamps_neg = cv::Mat::zeros(sensor_size_, CV_8UC3);

  int remapped_timestamp_fraction;
  double timestamp_fraction;
  for(Event e : events_)
  {
    event_count_image.at<cv::Vec3b>(e.y,e.x)[int(e.pol)]++;

     cv::Mat& curr_timestamp_image = e.pol ? timestamps_pos : timestamps_neg;

     // remap value
     timestamp_fraction = double(e.t - events_[0].t) / (events_[events_.size()-1].t - events_[0].t);
     remapped_timestamp_fraction = timestamp_fraction * std::pow(2,24); // remap 0-1 to 0 - 2^24

     // distribute the 24 bit number (remapped_timestamp_fraction) to 3 channel 8 bit image
     for (int i=0; i<3; i++)
     {
       curr_timestamp_image.at<cv::Vec3b>(e.y,e.x)[i] = (int) remapped_timestamp_fraction & 0xFF; // bit mask of 0000 0000 0000 0000 1111 1111
       remapped_timestamp_fraction = remapped_timestamp_fraction >> 8;  // shifts bits to right by 8
     }
  }

  // Write event count image + the two timestamps images to disk
  std::string path_event_count_image = ze::joinPath(output_folder_, "event_count.png");
  std::string path_timestamps_pos = ze::joinPath(output_folder_, "event_time_stamps_pos.png");
  std::string path_timestamps_neg = ze::joinPath(output_folder_, "event_time_stamps_neg.png");

  std::vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(0);

  cv::imwrite(path_event_count_image, event_count_image, compression_params);
  cv::imwrite(path_timestamps_pos, timestamps_pos, compression_params);
  cv::imwrite(path_timestamps_neg, timestamps_neg, compression_params);

  // Finish writing event file
  events_file_.close();
}

void SyntheticOpticFlowPublisher::eventsCallback(const EventsVector& events)
{
  CHECK_EQ(events.size(), 1);

  // Simply aggregate the events into the events_ buffer.
  // At the destruction of this object, everything will be saved to disk.
  for(const Event& e : events[0])
  {
     events_file_ << e.t << " " << e.x << " " << e.y << " " << (e.pol? 1 : 0) << std::endl;
     events_.push_back(e);
  }
}

} // namespace event_camera_simulator
