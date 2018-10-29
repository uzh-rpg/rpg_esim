#include <esim/data_provider/data_provider_from_folder.hpp>
#include <ze/common/file_utils.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

namespace event_camera_simulator {

DataProviderFromFolder::DataProviderFromFolder(const std::string& path_to_data_folder)
  : DataProviderBase(DataProviderType::Folder),
    path_to_data_folder_(path_to_data_folder)
{
  // Load CSV image file
  images_in_str_.open(ze::joinPath(path_to_data_folder, "images.csv"));
  CHECK(images_in_str_.is_open());

  // Load camera rig
  camera_rig_ = ze::cameraRigFromYaml(ze::joinPath(path_to_data_folder, "calib.yaml"));
  CHECK(camera_rig_);
  CHECK_EQ(camera_rig_->size(), 1u) << "Only one camera in the rig is supported at the moment";

  // Allocate memory for image data
  sim_data_.images.emplace_back(ImagePtr(new Image(
                                           cv::Size(camera_rig_->at(0).width(),
                                                    camera_rig_->at(0).height()))));

  sim_data_.camera_rig = camera_rig_;
  sim_data_.images_updated = true;
  sim_data_.depthmaps_updated = false;
  sim_data_.optic_flows_updated = false;
  sim_data_.twists_updated = false;
  sim_data_.poses_updated = false;
  sim_data_.imu_updated = false;
}

int64_t DataProviderFromFolder::getTimeStamp(const std::string& ts_str) const
{
  return std::stoll(ts_str);
}

size_t DataProviderFromFolder::numCameras() const
{
  return camera_rig_->size();
}

bool DataProviderFromFolder::spinOnce()
{
  std::string line;
  if(!getline(images_in_str_, line))
  {
    return false;
  }

  if('%' != line.at(0) && '#' != line.at(0))
  {
    std::vector<std::string> items = ze::splitString(line, delimiter_);
    CHECK_GE(items.size(), num_tokens_in_line_);
    int64_t stamp = getTimeStamp(items[0]);

    const std::string& path_to_image = ze::joinPath(path_to_data_folder_, "frame", "cam_0", items[1]);
    cv::Mat image = cv::imread(path_to_image, 0);
    CHECK(image.data) << "Could not load image from file: " << path_to_image;
    CHECK(image.rows == camera_rig_->at(0).height()
          && image.cols == camera_rig_->at(0).width()) << "The image size in the data folder and the image size"
                                                          "specified in the camera rig do not match";

    VLOG(3) << "Read image from file: " << path_to_image;
    image.convertTo(*sim_data_.images[0], cv::DataType<ImageFloatType>::type, 1./255.);

    if(callback_)
    {
      sim_data_.timestamp = static_cast<Time>(stamp);
      callback_(sim_data_);
    }
  }

  return true;
}

bool DataProviderFromFolder::ok() const
{
  if (!running_)
  {
    VLOG(1) << "Data Provider was paused/terminated.";
    return false;
  }
  return true;
}

} // namespace event_camera_simulator
