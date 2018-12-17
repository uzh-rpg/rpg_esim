#include <esim/visualization/ros_utils.hpp>
#include <esim/common/utils.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>

namespace event_camera_simulator {

void pointCloudToMsg(const PointCloud& pointcloud, const std::string& frame_id, Time t, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& msg)
{
  CHECK(msg);
  msg->header.frame_id = frame_id;
  msg->height = pointcloud.size();
  msg->width = 1;
  for(auto& p_c : pointcloud)
  {
    pcl::PointXYZRGB p;
    p.x = p_c.xyz(0);
    p.y = p_c.xyz(1);
    p.z = p_c.xyz(2);
    p.r = p_c.rgb(0);
    p.g = p_c.rgb(1);
    p.b = p_c.rgb(2);
    msg->points.push_back(p);
  }

  pcl_conversions::toPCL(toRosTime(t), msg->header.stamp);
}

void imageToMsg(const Image& image, Time t, sensor_msgs::ImagePtr& msg)
{
  cv_bridge::CvImage cv_image;
  image.convertTo(cv_image.image, CV_8U, 255.0);
  cv_image.encoding = "mono8";
  cv_image.header.stamp = toRosTime(t);
  msg = cv_image.toImageMsg();
}

void depthmapToMsg(const Depthmap& depthmap, Time t, sensor_msgs::ImagePtr& msg)
{
  cv_bridge::CvImage cv_depthmap;
  depthmap.convertTo(cv_depthmap.image, CV_32FC1);
  cv_depthmap.encoding = "32FC1";
  cv_depthmap.header.stamp = toRosTime(t);
  msg = cv_depthmap.toImageMsg();
}

void opticFlowToMsg(const OpticFlow& flow, Time t, esim_msgs::OpticFlowPtr& msg)
{
  CHECK(msg);
  msg->header.stamp = toRosTime(t);

  const int height = flow.rows;
  const int width = flow.cols;
  msg->height = height;
  msg->width = width;

  msg->flow_x.resize(height * width);
  msg->flow_y.resize(height * width);
  for(int y=0; y<height; ++y)
  {
    for(int x=0; x<width; ++x)
    {
      msg->flow_x[x + y * width] = static_cast<float>(flow(y,x)[0]);
      msg->flow_y[x + y * width] = static_cast<float>(flow(y,x)[1]);
    }
  }
}

void eventsToMsg(const Events& events, int width, int height, dvs_msgs::EventArrayPtr& msg)
{
  CHECK(msg);
  std::vector<dvs_msgs::Event> events_list;
  for(const Event& e : events)
  {
    dvs_msgs::Event ev;
    ev.x = e.x;
    ev.y = e.y;
    ev.ts = toRosTime(e.t);
    ev.polarity = e.pol;

    events_list.push_back(ev);
  }

  msg->events = events_list;
  msg->height = height;
  msg->width = width;
  msg->header.stamp = events_list.back().ts;
}

sensor_msgs::Imu imuToMsg(const Vector3& acc, const Vector3& gyr, Time t)
{
  sensor_msgs::Imu imu;
  imu.header.stamp = toRosTime(t);

  imu.linear_acceleration.x = acc(0);
  imu.linear_acceleration.y = acc(1);
  imu.linear_acceleration.z = acc(2);

  imu.angular_velocity.x = gyr(0);
  imu.angular_velocity.y = gyr(1);
  imu.angular_velocity.z = gyr(2);

  return imu;
}

geometry_msgs::TwistStamped twistToMsg(const AngularVelocity& w, const LinearVelocity& v, Time t)
{
  geometry_msgs::TwistStamped twist;
  twist.header.stamp = toRosTime(t);

  twist.twist.angular.x = w(0);
  twist.twist.angular.y = w(1);
  twist.twist.angular.z = w(2);

  twist.twist.linear.x = v(0);
  twist.twist.linear.y = v(1);
  twist.twist.linear.z = v(2);

  return twist;
}

void cameraToMsg(const ze::Camera::Ptr& camera, Time t, sensor_msgs::CameraInfoPtr& msg)
{
  CHECK(msg);
  msg->width = camera->width();
  msg->height = camera->height();
  msg->header.stamp = toRosTime(t);

  CalibrationMatrix K = calibrationMatrixFromCamera(camera);
  boost::array<double, 9> K_vec;
  std::vector<double> D_vec;
  for(int i=0; i<3; ++i)
  {
    for(int j=0; j<3; ++j)
    {
      K_vec[j+i*3] = static_cast<double>(K(i,j));
    }
  }

  switch(camera->type())
  {
    case ze::CameraType::PinholeRadialTangential:
    case ze::CameraType::Pinhole:
      msg->distortion_model = "plumb_bob";
      break;
    case ze::CameraType::PinholeEquidistant:
      msg->distortion_model = "equidistant";
      break;
    case ze::CameraType::PinholeFov:
      msg->distortion_model = "fov";
      break;
    default:
      LOG(WARNING) << "Unknown camera distortion model";
      msg->distortion_model = "";
      break;
  }

  for(int j=0; j<camera->distortionParameters().rows(); ++j)
  {
    D_vec.push_back(static_cast<double>(camera->distortionParameters()(j))); // @TODO: use the distortion params from the camera
  }

  msg->K = K_vec;
  msg->D = D_vec;
  msg->P = {K(0,0), 0,      K(0,2), 0,
            0,      K(1,1), K(1,2), 0,
            0,      0,      1,      0};
  msg->R = {1, 0, 0,
            0, 1, 0,
            0, 0, 1};
}


} // namespace event_camera_simulator
