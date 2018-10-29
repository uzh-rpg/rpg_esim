#include <esim/imp_unrealcv_renderer/unrealcv_renderer.hpp>
#include <esim/unrealcv_bridge/unrealcv_bridge.hpp>
#include <esim/imp_unrealcv_renderer/utils.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ze/common/path_utils.hpp>
#include <iomanip>

DEFINE_double(x_offset, 0, "X offset of the trajectory, in meters");
DEFINE_double(y_offset, 0, "Y offset of the trajectory, in meters");
DEFINE_double(z_offset, 0, "Z offset of the trajectory, in meters");
DECLARE_double(trajectory_multiplier_x);
DECLARE_double(trajectory_multiplier_y);
DECLARE_double(trajectory_multiplier_z);

DEFINE_int32(unrealcv_post_median_blur, 0,
             "If > 0, median blur the raw UnrealCV images"
             "with a median filter of this size");

DEFINE_double(unrealcv_post_gaussian_blur_sigma, 0,
              "If sigma > 0, Gaussian blur the raw UnrealCV images"
              "with a Gaussian filter standard deviation sigma.");

DEFINE_string(unrealcv_output_directory, "",
              "Output directory in which to output the raw RGB images.");

namespace event_camera_simulator {

UnrealCvRenderer::UnrealCvRenderer()
{
  client_ = std::make_shared<UnrealCvClient>("localhost", "9000");
  frame_idx_ = 0;
}

void UnrealCvRenderer::setCamera(const ze::Camera::Ptr& camera)
{
  camera_ = camera;

  // compute the horizontal field of view of the camera
  ze::VectorX intrinsics = camera_->projectionParameters();
  const FloatType fx = intrinsics(0);
  const FloatType hfov_deg = 2 * std::atan(0.5 * (FloatType) camera_->width() / fx) * 180. / CV_PI;

  client_->setCameraFOV(static_cast<float>(hfov_deg));
  client_->setCameraSize(camera->width(), camera->height());
}


void UnrealCvRenderer::render(const Transformation& T_W_C, const ImagePtr& out_image, const DepthmapPtr& out_depthmap) const
{
  CHECK_EQ(out_image->rows, camera_->height());
  CHECK_EQ(out_image->cols, camera_->width());

  VLOG(1) << "T_W_C (ZE) = " << T_W_C;

  const Transformation::TransformationMatrix mT_ZE_C = T_W_C.getTransformationMatrix();
  Transformation::TransformationMatrix mT_UE_ZE;
  mT_UE_ZE << 1, 0,  0, 0,
              0, -1, 0, 0,
              0, 0,  1, 0,
              0, 0,  0, 1;

  Transformation::TransformationMatrix mT_C_UEC;
  mT_C_UEC << 0, 1, 0,  0,
              0, 0, -1, 0,
              1, 0, 0,  0,
              0, 0, 0,  1;

  // rotate 90 deg to the right so that R_W_C = Identity <=> euler angle (0,0,0) in UnrealCV
//  Transformation::TransformationMatrix mT_rotate_90_right;
//  mT_rotate_90_right << 0, -1, 0, 0,
//                        1, 0, 0, 0,
//                        0, 0, 1, 0,
//                        0, 0, 0, 1;
//  mT_rotate_90_right << 1, 0, 0, 0,
//                        0, 1, 0, 0,
//                        0, 0, 1, 0,
//                        0, 0, 0, 1;
//  const Transformation::TransformationMatrix mT_UE_UEC = mT_rotate_90_right * mT_UE_ZE * mT_ZE_C * mT_C_UEC;
  const Transformation::TransformationMatrix mT_UE_UEC = mT_UE_ZE * mT_ZE_C * mT_C_UEC;
  const Transformation T_UE_UEC(mT_UE_UEC);

  VLOG(1) << "T_ZE_C = " << mT_ZE_C;
  VLOG(1) << "T_UE_UEC = " << T_UE_UEC;

  FloatType yaw, pitch, roll;
  quaternionToEulerUnrealEngine(T_UE_UEC.getRotation(), yaw, pitch, roll);

  const FloatType x_offset = static_cast<FloatType>(FLAGS_x_offset);
  const FloatType y_offset = static_cast<FloatType>(FLAGS_y_offset);
  const FloatType z_offset = static_cast<FloatType>(FLAGS_z_offset);

  const FloatType x = 100. * (FLAGS_trajectory_multiplier_x * T_UE_UEC.getPosition()[0] + x_offset);
  const FloatType y = 100. * (FLAGS_trajectory_multiplier_y * T_UE_UEC.getPosition()[1] + y_offset);
  const FloatType z = 100. * (FLAGS_trajectory_multiplier_z * T_UE_UEC.getPosition()[2] + z_offset);

  VLOG(1) << yaw << " " << pitch << " " << roll;

  CameraData cam_data = {0,
                         pitch,
                         yaw,
                         roll,
                         x,
                         y,
                         z};

  client_->setCamera(cam_data);
  cv::Mat img = client_->getImage(0);
  VLOG(5) << "Got image from the UnrealCV client";

  // (optionally) save raw RGB image to the output directory
  if(FLAGS_unrealcv_output_directory != "")
  {
    std::stringstream ss_nr;
    ss_nr << std::setw(10) << std::setfill('0') << frame_idx_;
    std::string path_frame = ze::joinPath(FLAGS_unrealcv_output_directory, "frame_" + ss_nr.str() +  ".png");
    VLOG(1) << "Saving raw RGB image to: " << path_frame;
    cv::imwrite(path_frame, img, {CV_IMWRITE_PNG_COMPRESSION, 9});
  }

  cv::Mat img_gray;
  cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

  if(FLAGS_unrealcv_post_median_blur > 0)
  {
    cv::medianBlur(img_gray, img_gray, FLAGS_unrealcv_post_median_blur);
  }

  if(FLAGS_unrealcv_post_gaussian_blur_sigma > 0)
  {
    cv::GaussianBlur(img_gray, img_gray, cv::Size(-1,-1), FLAGS_unrealcv_post_gaussian_blur_sigma);
  }

  cv::resize(img_gray, img_gray, cv::Size(camera_->width(), camera_->height()));
  img_gray.convertTo(*out_image, cv::DataType<ImageFloatType>::type, 1./255.);

  cv::Mat depth = client_->getDepth(0);
  VLOG(5) << "Got depth map from the UnrealCV client";
  CHECK_EQ(depth.type(), CV_32F);

  cv::resize(depth, depth, cv::Size(camera_->width(), camera_->height()));
  depth.convertTo(*out_depthmap, cv::DataType<ImageFloatType>::type);

  // the depth provided by the unrealcv client is the distance from the scene to the camera center,
  // we need to convert it to the distance to image plane (see Github issue: https://github.com/unrealcv/unrealcv/issues/14)
  const ImageFloatType yc = 0.5 * static_cast<ImageFloatType>(camera_->height()) - 1.0;
  const ImageFloatType xc = 0.5 * static_cast<ImageFloatType>(camera_->width()) - 1.0;
  const ImageFloatType f = static_cast<ImageFloatType>(camera_->projectionParameters()(0));
  for(int y=0; y<camera_->height(); ++y)
  {
    for(int x=0; x<camera_->width(); ++x)
    {
      const ImageFloatType point_depth = (*out_depthmap)(y,x);
      const ImageFloatType dx = static_cast<ImageFloatType>(x) - xc;
      const ImageFloatType dy = static_cast<ImageFloatType>(y) - yc;
      const ImageFloatType distance_from_center = std::sqrt(dx*dx + dy*dy);
      const ImageFloatType plane_depth = point_depth / std::sqrt(1.0 + std::pow(distance_from_center / f, 2));
      (*out_depthmap)(y,x) = plane_depth;
    }
  }

  frame_idx_++;
}

} // namespace event_camera_simulator
