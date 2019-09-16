#include <esim/data_provider/data_provider_from_folder.hpp>
#include <ze/common/file_utils.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <ze/cameras/camera_impl.hpp>

namespace event_camera_simulator {

DataProviderFromFolder::DataProviderFromFolder(const std::string& path_to_data_folder)
  : DataProviderBase(DataProviderType::Folder),
    path_to_data_folder_(path_to_data_folder),
    finished_parsing_(false)
{
  // Load CSV image file
  images_in_str_.open(ze::joinPath(path_to_data_folder, "images.csv"));
  CHECK(images_in_str_.is_open());

  // Create dummy camera rig
  // these intrinsic values are filled with garbage (width = height = 1) since the actual values are not known
  ze::CameraVector cameras;
  cameras.emplace_back(ze::createEquidistantCameraShared(1, 1, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  ze::TransformationVector extrinsics;
  extrinsics.push_back(ze::Transformation());
  camera_rig_ = std::make_shared<ze::CameraRig>(extrinsics, cameras, "camera");

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
    VLOG(1) << "Finished parsing images.csv file";
    finished_parsing_ = true;
    return false;
  }

  if('%' != line.at(0) && '#' != line.at(0))
  {
    std::vector<std::string> items = ze::splitString(line, delimiter_);
    CHECK_GE(items.size(), num_tokens_in_line_);
    int64_t stamp = getTimeStamp(items[0]);

    const std::string& path_to_image = ze::joinPath(path_to_data_folder_, items[1]);
    cv::Mat image = cv::imread(path_to_image, 0);
    CHECK(image.data) << "Could not load image from file: " << path_to_image;

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
  if (!running_ || finished_parsing_)
  {
    VLOG(1) << "Data Provider was paused/terminated.";
    return false;
  }
  return true;
}

} // namespace event_camera_simulator
