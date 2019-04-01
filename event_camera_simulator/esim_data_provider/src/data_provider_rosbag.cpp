#include <esim/data_provider/data_provider_rosbag.hpp>

#include <ze/common/logging.hpp>
#include <rosbag/query.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <ze/common/time_conversions.hpp>
#include <ze/common/string_utils.hpp>
#include <ze/common/path_utils.hpp>

DEFINE_int32(data_source_stop_after_n_frames, -1,
             "How many frames should be processed?");
DEFINE_double(data_source_start_time_s, 0.0,
              "Start time in seconds");
DEFINE_double(data_source_stop_time_s, 0.0,
              "Stop time in seconds");

namespace event_camera_simulator {

DataProviderRosbag::DataProviderRosbag(
    const std::string& bag_filename,
    const std::map<std::string, size_t>& img_topic_camidx_map)
  : DataProviderBase(DataProviderType::Rosbag)
  , img_topic_camidx_map_(img_topic_camidx_map)
{
  loadRosbag(bag_filename);

  std::vector<std::string> topics;
  for (auto it : img_topic_camidx_map_)
  {
    VLOG(1) << "Subscribing to: " << it.first;
    topics.push_back(it.first);
    sim_data_.images.emplace_back(ColorImagePtr(new ColorImage()));
  }

  initBagView(topics);
}

void DataProviderRosbag::loadRosbag(const std::string& bag_filename)
{
  CHECK(ze::fileExists(bag_filename)) << "File does not exist: " << bag_filename;
  VLOG(1) << "Opening rosbag: " << bag_filename << " ...";
  bag_.reset(new rosbag::Bag);
  try
  {
    bag_->open(bag_filename, rosbag::bagmode::Read);
  }
  catch (const std::exception e)
  {
    LOG(FATAL) << "Could not open rosbag " << bag_filename << ": " << e.what();
  }
}

void DataProviderRosbag::initBagView(const std::vector<std::string>& topics)
{
  bag_view_.reset(new rosbag::View(*bag_, rosbag::TopicQuery(topics)));
  if (FLAGS_data_source_start_time_s != 0.0 ||
      FLAGS_data_source_stop_time_s != 0.0)
  {
    CHECK_GE(FLAGS_data_source_start_time_s, 0);
    CHECK_GE(FLAGS_data_source_stop_time_s, 0);

    // Retrieve begin and end times from the bag file (given the topic query).
    const ros::Time absolute_time_offset = bag_view_->getBeginTime();
    VLOG(2) << "Bag begin time: " << absolute_time_offset;
    const ros::Time absolute_end_time = bag_view_->getEndTime();
    VLOG(2) << "Bag end time: " << absolute_end_time;
    if (absolute_end_time < absolute_time_offset)
    {
      LOG(FATAL) << "Invalid bag end time: "
                 << absolute_end_time
                 << ". Check that the bag file is properly indexed"
                 << " by running 'rosbag reindex file.bag'.";
    }

    // Compute start and stop time.
    LOG(INFO) << "Starting rosbag at time: " << FLAGS_data_source_start_time_s;
    const ros::Duration data_source_start_time(FLAGS_data_source_start_time_s);
    const ros::Time absolute_start_time =
        data_source_start_time.isZero() ?
          absolute_time_offset : absolute_time_offset + data_source_start_time;
    const ros::Duration data_source_stop_time(FLAGS_data_source_stop_time_s);
    const ros::Time absolute_stop_time =
        data_source_stop_time.isZero() ?
          absolute_end_time : absolute_time_offset + data_source_stop_time;

    // Ensure that the provided stop time is valid.
    // When a bag file is corrupted / invalid the bag end time
    // cannot be retrieved. Run rosbag info to check if the bag file
    // is properly indexed.
    if (absolute_stop_time < absolute_start_time)
    {
      LOG(ERROR) << "Provided stop time is less than bag begin time. "
                 << "Please make sure to provide a valid stop time and "
                 << "check that the bag file is properly indexed "
                 << "by running 'rosbag reindex file.bag'.";
    }
    else if (absolute_stop_time > absolute_end_time)
    {
      LOG(ERROR) << "Provided stop time is greater than bag end time. "
                 << "Please make sure to provide a valid stop time and "
                 << "check that the bag file is properly indexed "
                 << "by running 'rosbag reindex file.bag'.";
    }
    else
    {
      VLOG(1) << "Absolute start time set to " << absolute_start_time;
      VLOG(1) << "Absolute stop time set to " << absolute_stop_time;
    }

    // Reset the bag View
    CHECK_GT(absolute_stop_time, absolute_start_time);
    CHECK_LE(absolute_stop_time, absolute_end_time);
    bag_view_.reset(new rosbag::View(*bag_, rosbag::TopicQuery(topics),
                                     absolute_start_time, absolute_stop_time));
  }
  bag_view_it_ = bag_view_->begin();

  // Ensure that topics exist
  // The connection info only contains topics that are available in the bag
  // If a topic is requested that is not avaiable, it does not show up in the info.
  std::vector<const rosbag::ConnectionInfo*> connection_infos =
      bag_view_->getConnections();
  if (topics.size() != connection_infos.size())
  {
    LOG(ERROR) << "Successfully connected to " << connection_infos.size() << " topics:";
    for (const rosbag::ConnectionInfo* info : connection_infos)
    {
      LOG(ERROR) << "*) " << info->topic;
    }
    LOG(ERROR) << "Requested " << topics.size() << " topics:";
    for (const std::string topic : topics)
    {
      LOG(ERROR) << "*) " << topic;
    }
    LOG(FATAL) << "Not all requested topics founds in bagfile. "
               << "Is topic_cam0, topic_imu0, etc. set correctly? "
               << "Maybe removing/adding a slash as prefix solves the problem.";
  }
}

size_t DataProviderRosbag::numCameras() const
{
  return img_topic_camidx_map_.size();
}

bool DataProviderRosbag::spinOnce()
{
  if (bag_view_it_ != bag_view_->end())
  {
    const rosbag::MessageInstance m = *bag_view_it_;

    // Camera Messages:
    const sensor_msgs::ImageConstPtr m_img = m.instantiate<sensor_msgs::Image>();
    if (m_img && callback_)
    {
      if (!cameraSpin(m_img, m))
      {
        return false;
      }
    }
    else
    {
      LOG_FIRST_N(WARNING, 1) << "No camera callback registered but measurements available";
    }

    ++bag_view_it_;
    return true;
  }
  return false;
}

bool DataProviderRosbag::cameraSpin(sensor_msgs::ImageConstPtr m_img,
                                    const rosbag::MessageInstance& m)
{
  auto it = img_topic_camidx_map_.find(m.getTopic());
  if (it != img_topic_camidx_map_.end())
  {
    ++n_processed_images_;
    if (FLAGS_data_source_stop_after_n_frames > 0 &&
        n_processed_images_ > FLAGS_data_source_stop_after_n_frames)
    {
      LOG(WARNING) << "Data source has reached max number of desired frames.";
      running_ = false;
      return false;
    }

    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(m_img, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      LOG(WARNING) << "cv_bridge exception: %s", e.what();
      return false;
    }

    cv_ptr->image.convertTo(*(sim_data_.images[0]), cv::DataType<ImageFloatType>::type, 1./255.);
    sim_data_.timestamp = static_cast<Time>(m_img->header.stamp.toNSec());

    sim_data_.imu_updated = false;
    sim_data_.images_updated = true;
    sim_data_.depthmaps_updated = false;
    sim_data_.optic_flows_updated = false;
    sim_data_.twists_updated = false;
    sim_data_.poses_updated = false;

    callback_(sim_data_);
  }
  else
  {
    LOG_FIRST_N(WARNING, 1) << "Topic in bag that is not subscribed: " << m.getTopic();
  }

  return true;
}

bool DataProviderRosbag::ok() const
{
  if (!running_)
  {
    VLOG(1) << "Data Provider was paused/terminated.";
    return false;
  }
  if (bag_view_it_ == bag_view_->end())
  {
    VLOG(1) << "All data processed.";
    return false;
  }
  return true;
}

size_t DataProviderRosbag::size() const
{
  CHECK(bag_view_);
  return bag_view_->size();
}

} // namespace event_camera_simulator
