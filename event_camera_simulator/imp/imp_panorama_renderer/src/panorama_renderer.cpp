#include <esim/common/utils.hpp>
#include <esim/imp_panorama_renderer/panorama_renderer.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <glog/logging.h>
#include <opencv2/core/eigen.hpp>

namespace event_camera_simulator {

PanoramaRenderer::PanoramaRenderer(const Image& texture,
                                   const Transformation::Rotation &R_W_P)
  : texture_(texture),
    R_W_P_(R_W_P)
{
  principal_point_ = ze::Keypoint(0.5 * texture_.cols,
                                  0.5 * texture_.rows);
  fx_ = texture_.cols / (2.0 * CV_PI);
  fy_ = texture_.rows / CV_PI;
  LOG(INFO) << "Initialized panoramic camera with size: "
            << texture_.cols << "x" << texture_.rows
            << ", principal point: " << principal_point_.transpose()
            << ", fx = " << fx_ << "px , fy = " << fy_ << " px";
}

PanoramaRenderer::~PanoramaRenderer()
{
}

void PanoramaRenderer::setCamera(const ze::Camera::Ptr& camera)
{
  CHECK(camera);
  camera_ = camera;

  precomputePixelToBearingLookupTable();

  // Preallocate memory for the warping maps
  cv::Size sensor_size(camera->width(), camera->height());
  map_x_ = cv::Mat_<float>::zeros(sensor_size);
  map_y_ = cv::Mat_<float>::zeros(sensor_size);
}


void PanoramaRenderer::precomputePixelToBearingLookupTable()
{
  // points_C is a matrix containing the coordinates of each pixel coordinate in the image plane
  ze::Keypoints points_C(2, camera_->width() * camera_->height());
  for(int y=0; y<camera_->height(); ++y)
  {
    for(int x=0; x<camera_->width(); ++x)
    {
      points_C.col(x + camera_->width() * y) = ze::Keypoint(x,y);
    }
  }
  bearings_C_ = camera_->backProjectVectorized(points_C);
  bearings_C_.array().rowwise() /= bearings_C_.row(2).array();
}


void PanoramaRenderer::projectToPanorama(const Eigen::Ref<const ze::Bearing>& f, ze::Keypoint* keypoint) const
{
  CHECK(keypoint);
  static constexpr FloatType kEpsilon = 1e-10;

  const FloatType X = f[0];
  const FloatType Y = f[1];
  const FloatType Z = f[2];

  const FloatType rho2 = X*X+Y*Y+Z*Z;
  const FloatType rho = std::sqrt(rho2);

  CHECK_GE(rho, kEpsilon);

  const FloatType phi = std::atan2(X, Z);
  const FloatType theta = std::asin(-Y/rho);

  *keypoint = principal_point_ + ze::Keypoint(phi * fx_, -theta * fy_);
}


void PanoramaRenderer::render(const Transformation& T_W_C, const ImagePtr& out_image, const DepthmapPtr& out_depthmap) const
{
  CHECK_EQ(out_image->rows, camera_->height());
  CHECK_EQ(out_image->cols, camera_->width());

  const Transformation::RotationMatrix R_P_C = R_W_P_.inverse().getRotationMatrix() * T_W_C.getRotationMatrix();

  ze::Bearings bearings_P = R_P_C * bearings_C_;
  ze::Keypoint keypoint;
  for(int y=0; y<camera_->height(); ++y)
  {
    for(int x=0; x<camera_->width(); ++x)
    {
      const ze::Bearing& f = bearings_P.col(x + camera_->width() * y);

      // project bearing vector to panorama image
      projectToPanorama(f, &keypoint);

      map_x_(y,x) = static_cast<float>(keypoint[0]);
      map_y_(y,x) = static_cast<float>(keypoint[1]);
    }
  }

  cv::remap(texture_, *out_image, map_x_, map_y_, cv::INTER_LINEAR, cv::BORDER_REFLECT_101);

  if(out_depthmap)
  {
    static constexpr ImageFloatType infinity = 1e10;
    out_depthmap->setTo(infinity);
  }
}

} // namespace event_camera_simulator
