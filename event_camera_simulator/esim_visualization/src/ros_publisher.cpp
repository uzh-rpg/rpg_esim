#include <esim/visualization/ros_publisher.hpp>
#include <esim/common/utils.hpp>
#include <ze/common/time_conversions.hpp>
#include <esim/visualization/ros_utils.hpp>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_double(ros_publisher_camera_info_rate, 0,
"Camera info (maximum) publish rate, in Hz");

DEFINE_double(ros_publisher_frame_rate, 30,
"(Maximum) frame rate, in Hz");

DEFINE_double(ros_publisher_depth_rate, 0,
"(Maximum) depthmap publish rate, in Hz");

DEFINE_double(ros_publisher_pointcloud_rate, 0,
"(Maximum) point cloud publish rate, in Hz");

DEFINE_double(ros_publisher_optic_flow_rate, 0,
"(Maximum) optic flow map publish rate, in Hz");

namespace event_camera_simulator {

RosPublisher::RosPublisher(size_t num_cameras)
{
  CHECK_GE(num_cameras, 1);
  num_cameras_ = num_cameras;
  sensor_sizes_ = std::vector<cv::Size>(num_cameras_);

  // Initialize ROS if it was not initialized before.
  if(!ros::isInitialized())
  {
    VLOG(1) << "Initializing ROS";
    int argc = 0;
    ros::init(argc, nullptr, std::string("ros_publisher"), ros::init_options::NoSigintHandler);
  }

  // Create node and subscribe.
  nh_.reset(new ros::NodeHandle(""));
  it_.reset(new image_transport::ImageTransport(*nh_));

  // Setup ROS publishers for images, events, poses, depth maps, camera info, etc.
  for(size_t i=0; i<num_cameras_; ++i)
  {
    event_pub_.emplace_back(
          new ros::Publisher(
            nh_->advertise<dvs_msgs::EventArray> (getTopicName(i, "events"), 0)));

    image_pub_.emplace_back(
          new image_transport::Publisher(
            it_->advertise(getTopicName(i, "image_raw"), 0)));

    image_corrupted_pub_.emplace_back(
          new image_transport::Publisher(
            it_->advertise(getTopicName(i, "image_corrupted"), 0)));

    depthmap_pub_.emplace_back(
          new image_transport::Publisher(
            it_->advertise(getTopicName(i, "depthmap"), 0)));

    optic_flow_pub_.emplace_back(
          new ros::Publisher(
            nh_->advertise<esim_msgs::OpticFlow> (getTopicName(i, "optic_flow"), 0)));

    camera_info_pub_.emplace_back(
          new ros::Publisher(
            nh_->advertise<sensor_msgs::CameraInfo> (getTopicName(i, "camera_info"), 0)));

    twist_pub_.emplace_back(
          new ros::Publisher(
            nh_->advertise<geometry_msgs::TwistStamped> (getTopicName(i, "twist"), 0)));

    pointcloud_pub_.emplace_back(
          new ros::Publisher(
            nh_->advertise<pcl::PointCloud<pcl::PointXYZ>> (getTopicName(i, "pointcloud"), 0)));
  }

  pose_pub_.reset(new ros::Publisher(nh_->advertise<geometry_msgs::PoseStamped> ("pose", 0)));
  imu_pub_.reset(new ros::Publisher(nh_->advertise<sensor_msgs::Imu> ("imu", 0)));
  tf_broadcaster_.reset(new tf::TransformBroadcaster());

  last_published_camera_info_time_ = 0;
  last_published_image_time_ = 0;
  last_published_corrupted_image_time_ = 0;
  last_published_depthmap_time_ = 0;
  last_published_optic_flow_time_ = 0;
  last_published_pointcloud_time_ = 0;
}

RosPublisher::~RosPublisher()
{
  for(size_t i=0; i<num_cameras_; ++i)
  {
    event_pub_[i]->shutdown();
    image_pub_[i]->shutdown();
    image_corrupted_pub_[i]->shutdown();
    depthmap_pub_[i]->shutdown();
    optic_flow_pub_[i]->shutdown();
    camera_info_pub_[i]->shutdown();
    twist_pub_[i]->shutdown();
    pointcloud_pub_[i]->shutdown();
  }
  pose_pub_->shutdown();

  ros::shutdown();
}

void RosPublisher::pointcloudCallback(const PointCloudVector& pointclouds, Time t)
{
  CHECK_EQ(pointcloud_pub_.size(), num_cameras_);
  CHECK_EQ(pointclouds.size(), num_cameras_);

  for(size_t i=0; i<num_cameras_; ++i)
  {
    const PointCloud& pcl_camera = pointclouds[i];

    CHECK(pointcloud_pub_[i]);
    if(pointcloud_pub_[i]->getNumSubscribers() == 0)
    {
      continue;
    }

    Duration min_time_interval_between_published_pointclouds_
        = ze::secToNanosec(1.0 / FLAGS_ros_publisher_pointcloud_rate);
    if(last_published_pointcloud_time_ > 0 && t - last_published_pointcloud_time_ < min_time_interval_between_published_pointclouds_)
    {
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr msg (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::stringstream ss; ss << "cam" << i;
    pointCloudToMsg(pointclouds[i], ss.str(), t, msg);
    pointcloud_pub_[i]->publish(*msg);
  }

  last_published_pointcloud_time_ = t;
}

void RosPublisher::imageCallback(const ImagePtrVector& images, Time t)
{
  CHECK_EQ(image_pub_.size(), num_cameras_);

  for(size_t i=0; i<num_cameras_; ++i)
  {
    sensor_sizes_[i] = images[i]->size();

    CHECK(image_pub_[i]);
    if(image_pub_[i]->getNumSubscribers() == 0)
    {
      continue;
    }

    static const Duration min_time_interval_between_published_images_
        = ze::secToNanosec(1.0 / FLAGS_ros_publisher_frame_rate);
    if(last_published_image_time_ > 0 && t - last_published_image_time_ < min_time_interval_between_published_images_)
    {
      return;
    }

    if(images[i])
    {
      sensor_msgs::ImagePtr msg;
      imageToMsg(*images[i], t, msg);
      image_pub_[i]->publish(msg);
    }
  }

  last_published_image_time_ = t;
}

void RosPublisher::imageCorruptedCallback(const ImagePtrVector& corrupted_images, Time t)
{
  CHECK_EQ(image_corrupted_pub_.size(), num_cameras_);

  for(size_t i=0; i<num_cameras_; ++i)
  {
    CHECK(image_corrupted_pub_[i]);
    if(image_corrupted_pub_[i]->getNumSubscribers() == 0)
    {
      continue;
    }

    static const Duration min_time_interval_between_published_images_
        = ze::secToNanosec(1.0 / FLAGS_ros_publisher_frame_rate);
    if(last_published_corrupted_image_time_ > 0 && t - last_published_corrupted_image_time_ < min_time_interval_between_published_images_)
    {
      return;
    }

    if(corrupted_images[i])
    {
      sensor_msgs::ImagePtr msg;
      imageToMsg(*corrupted_images[i], t, msg);
      image_corrupted_pub_[i]->publish(msg);
    }
  }

  last_published_corrupted_image_time_ = t;
}

void RosPublisher::depthmapCallback(const DepthmapPtrVector& depthmaps, Time t)
{
  CHECK_EQ(depthmap_pub_.size(), num_cameras_);

  for(size_t i=0; i<num_cameras_; ++i)
  {
    CHECK(depthmap_pub_[i]);
    if(depthmap_pub_[i]->getNumSubscribers() == 0)
    {
      continue;
    }

    static const Duration min_time_interval_between_published_depthmaps_
        = ze::secToNanosec(1.0 / FLAGS_ros_publisher_depth_rate);
    if(last_published_depthmap_time_ > 0 && t - last_published_depthmap_time_ < min_time_interval_between_published_depthmaps_)
    {
      return;
    }

    if(depthmaps[i])
    {
      sensor_msgs::ImagePtr msg;
      depthmapToMsg(*depthmaps[i], t, msg);
      depthmap_pub_[i]->publish(msg);
    }
  }

  last_published_depthmap_time_ = t;
}


void RosPublisher::opticFlowCallback(const OpticFlowPtrVector& optic_flows, Time t)
{
  CHECK_EQ(optic_flow_pub_.size(), num_cameras_);

  for(size_t i=0; i<num_cameras_; ++i)
  {
    CHECK(optic_flow_pub_[i]);
    if(optic_flow_pub_[i]->getNumSubscribers() == 0)
    {
      continue;
    }

    static const Duration min_time_interval_between_published_optic_flows_
        = (min_time_interval_between_published_optic_flows_ > 0) ? ze::secToNanosec(1.0 / FLAGS_ros_publisher_optic_flow_rate) : 0;
    if(min_time_interval_between_published_optic_flows_ > 0 && last_published_optic_flow_time_ > 0 && t - last_published_optic_flow_time_ < min_time_interval_between_published_optic_flows_)
    {
      return;
    }

    if(optic_flows[i])
    {
      esim_msgs::OpticFlow::Ptr msg;
      msg.reset(new esim_msgs::OpticFlow);
      opticFlowToMsg(*optic_flows[i], t, msg);
      optic_flow_pub_[i]->publish(msg);
    }
  }

  last_published_optic_flow_time_ = t;
}

void RosPublisher::eventsCallback(const EventsVector& events)
{
  CHECK_EQ(event_pub_.size(), num_cameras_);

  for(size_t i=0; i<num_cameras_; ++i)
  {
    if(sensor_sizes_[i].width == 0 || sensor_sizes_[i].height == 0)
    {
      continue;
    }

    if(events[i].empty())
    {
      continue;
    }

    CHECK(event_pub_[i]);
    if(event_pub_[i]->getNumSubscribers() == 0)
    {
      continue;
    }

    dvs_msgs::EventArrayPtr msg;
    msg.reset(new dvs_msgs::EventArray);
    eventsToMsg(events[i], sensor_sizes_[i].width, sensor_sizes_[i].height, msg);
    event_pub_[i]->publish(msg);
  }
}

void RosPublisher::poseCallback(const Transformation& T_W_B,
                                const TransformationVector& T_W_Cs,
                                Time t)
{
  if(T_W_Cs.size() != num_cameras_)
  {
    LOG(WARNING) << "Number of poses is different than number of cameras."
                 << "Will not output poses.";
    return;
  }

  // Publish to tf
  tf::StampedTransform bt;
  bt.child_frame_id_ = "body";
  bt.frame_id_ = "map";
  bt.stamp_ = toRosTime(t);
  tf::transformKindrToTF(T_W_B, &bt);
  tf_broadcaster_->sendTransform(bt);

  for(size_t i=0; i<num_cameras_; ++i)
  {
    std::stringstream ss;
    ss << "cam" << i;
    tf::StampedTransform bt;
    bt.child_frame_id_ = ss.str();
    bt.frame_id_ = "map";
    bt.stamp_ = toRosTime(t);
    tf::transformKindrToTF(T_W_Cs[i], &bt);
    tf_broadcaster_->sendTransform(bt);
  }

  // Publish pose message
  geometry_msgs::PoseStamped pose_stamped_msg;
  tf::poseStampedKindrToMsg(T_W_B, toRosTime(t), "map", &pose_stamped_msg);
  pose_pub_->publish(pose_stamped_msg);
}

void RosPublisher::twistCallback(const AngularVelocityVector &ws, const LinearVelocityVector &vs, Time t)
{
  if(ws.size() != num_cameras_
     || vs.size() != num_cameras_)
  {
    LOG(WARNING) << "Number of twists is different than number of cameras."
                 << "Will not output twists.";
    return;
  }
  CHECK_EQ(ws.size(), num_cameras_);
  CHECK_EQ(vs.size(), num_cameras_);
  CHECK_EQ(twist_pub_.size(), num_cameras_);

  for(size_t i=0; i<num_cameras_; ++i)
  {
    CHECK(twist_pub_[i]);
    if(twist_pub_[i]->getNumSubscribers() == 0)
    {
      continue;
    }

    const geometry_msgs::TwistStamped msg = twistToMsg(ws[i], vs[i], t);
    twist_pub_[i]->publish(msg);
  }
}

void RosPublisher::imuCallback(const Vector3& acc, const Vector3& gyr, Time t)
{
  if(imu_pub_->getNumSubscribers() == 0)
  {
    return;
  }

  const sensor_msgs::Imu msg = imuToMsg(acc, gyr, t);
  imu_pub_->publish(msg);
}

void RosPublisher::cameraInfoCallback(const ze::CameraRig::Ptr& camera_rig, Time t)
{
  CHECK(camera_rig);
  CHECK_EQ(camera_rig->size(), num_cameras_);

  static const Duration min_time_interval_between_published_camera_info_
      = ze::secToNanosec(1.0 / FLAGS_ros_publisher_camera_info_rate);
  if(last_published_camera_info_time_ > 0 && t - last_published_camera_info_time_ < min_time_interval_between_published_camera_info_)
  {
    return;
  }

  for(size_t i=0; i<num_cameras_; ++i)
  {
    CHECK(camera_info_pub_[i]);
    if(camera_info_pub_[i]->getNumSubscribers() == 0)
    {
      continue;
    }

    sensor_msgs::CameraInfoPtr msg;
    msg.reset(new sensor_msgs::CameraInfo);
    cameraToMsg(camera_rig->atShared(i), t, msg);
    camera_info_pub_[i]->publish(msg);
  }

  last_published_camera_info_time_ = t;

}

} // namespace event_camera_simulator
