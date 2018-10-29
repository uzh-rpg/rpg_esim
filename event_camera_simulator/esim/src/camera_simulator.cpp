#include <esim/esim/camera_simulator.hpp>

namespace event_camera_simulator {

void ImageBuffer::addImage(Time t, const Image& img)
{
  if(!data_.empty())
  {
    // Check that the image timestamps are monotonically increasing
    CHECK_GT(t, data_.back().stamp);
  }

  Duration exposure_time = data_.empty() ? 0 : t - data_.back().stamp;
  VLOG(2) << "Adding image to buffer with stamp: " << t
          << " and exposure time " << exposure_time;
  data_.push_back(ImageData(img.clone(), t, exposure_time));

  // Remove all the images with timestamp older than t - buffer_size_ns_
  auto first_valid_element = std::lower_bound(data_.begin(), data_.end(), t - buffer_size_ns_,
          [](ImageData lhs, Time rhs) -> bool { return lhs.stamp < rhs; });

  data_.erase(data_.begin(), first_valid_element);
  VLOG(3) << "first/last element in buffer: "
          << data_.front().stamp
          << " " << data_.back().stamp;
  VLOG(3) << "number of images in the buffer: " << data_.size();

  CHECK_LE(data_.back().stamp - data_.front().stamp, buffer_size_ns_);
}


bool CameraSimulator::imageCallback(const Image &img, Time time,
                                         const ImagePtr& camera_image)
{
  CHECK(camera_image);
  CHECK_EQ(camera_image->size(), img.size());

  buffer_->addImage(time, img);

  static const Time initial_time = time;
  if(time - initial_time < exposure_time_)
  {
    LOG_FIRST_N(WARNING, 1) << "The images do not cover a time span long enough to simulate the exposure time accurately.";
    return false;
  }

  // average all the images in the buffer to simulate motion blur
  camera_image->setTo(0);
  ze::real_t denom = 0.;
  for(const ImageBuffer::ImageData& img : buffer_->getRawBuffer())
  {
    *camera_image += ze::nanosecToMillisecTrunc(img.exposure_time) * img.image;
    denom += ze::nanosecToMillisecTrunc(img.exposure_time);
  }
  *camera_image /= denom;
  cv::Mat disp;
  camera_image->convertTo(disp, CV_8U, 255);

  return true;
}

} // namespace event_camera_simulator
