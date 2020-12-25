#include <esim/common/utils.hpp>
#include <esim/imp_planar_renderer/planar_renderer.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <glog/logging.h>
#include <opencv2/core/eigen.hpp>
#include <ze/common/timer_collection.hpp>

DECLARE_TIMER(TimerPlanarRenderer, timers_planar_renderer,
              compute_remap_maps,
              inverse_homography,
              remap,
              compute_depthmap
              );

namespace event_camera_simulator {

PlanarRenderer::PlanarRenderer(const Image& texture,
                               const Camera::Ptr &cam_src,
                               const Transformation& T_W_P,
                               FloatType z_min,
                               FloatType z_max,
                               bool extend_border)
  : texture_(texture),
    cam_src_(cam_src),
    T_W_P_(T_W_P),
    z_min_(z_min),
    z_max_(z_max),
    extend_border_(extend_border)
{
  K_src_ = calibrationMatrixFromCamera(cam_src_);
  K_src_inv_ = K_src_.inverse();
  VLOG(1) << "K_src = " << K_src_;

  LOG(INFO) << "T_W_P = " << T_W_P_;
}

PlanarRenderer::~PlanarRenderer()
{
  timers_planar_renderer.saveToFile("/tmp", "planar_renderer.csv");
}

void PlanarRenderer::setCamera(const ze::Camera::Ptr& camera)
{
  CHECK(camera);
  camera_ = camera;

  precomputePixelToBearingLookupTable();

  // Preallocate memory for the warping maps
  cv::Size sensor_size(camera->width(), camera->height());
  map_x_ = cv::Mat_<float>::zeros(sensor_size);
  map_y_ = cv::Mat_<float>::zeros(sensor_size);
}


void PlanarRenderer::precomputePixelToBearingLookupTable()
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

void PlanarRenderer::render(const Transformation& T_W_C, const ImagePtr& out_image, const DepthmapPtr& out_depthmap) const
{
  CHECK_EQ(out_image->rows, camera_->height());
  CHECK_EQ(out_image->cols, camera_->width());

  const Transformation T_C_W = T_W_C.inverse();
  const Transformation T_C_P = T_C_W * T_W_P_;
  const Transformation T_P_C = T_C_P.inverse();

  ze::Matrix33 tmp;
  tmp.col(0) = T_C_P.getRotationMatrix().col(0);
  tmp.col(1) = T_C_P.getRotationMatrix().col(1);
  tmp.col(2) = T_C_P.getPosition();

  VLOG_EVERY_N(3, 100) << "T_C_P = " << T_C_P;
  VLOG_EVERY_N(3, 100) << "tmp = " << tmp;
  HomographyMatrix H_C_P = tmp;

  HomographyMatrix H_P_C;
  {
    auto t = timers_planar_renderer[TimerPlanarRenderer::inverse_homography].timeScope();
    H_P_C = H_C_P.inverse();
  }

  {
    auto t = timers_planar_renderer[TimerPlanarRenderer::compute_remap_maps].timeScope();
    ze::Bearings bearings_P = H_P_C * bearings_C_;
    for(int y=0; y<camera_->height(); ++y)
    {
      for(int x=0; x<camera_->width(); ++x)
      {
        const ze::Bearing& f = bearings_P.col(x + camera_->width() * y);
        map_x_(y,x) = static_cast<float>(K_src_(0,0) * f[0]/f[2] + K_src_(0,2));
        map_y_(y,x) = static_cast<float>(K_src_(1,1) * f[1]/f[2] + K_src_(1,2));
      }
    }
  }

  int border = (extend_border_) ? cv::BORDER_REFLECT : cv::BORDER_CONSTANT;
  {
    auto t = timers_planar_renderer[TimerPlanarRenderer::remap].timeScope();
    cv::remap(texture_, *out_image, map_x_, map_y_, cv::INTER_LINEAR, border, 0.8);
  }

  // Compute depth map in dst
  {
    auto t = timers_planar_renderer[TimerPlanarRenderer::compute_depthmap].timeScope();
    Vector3 X;
    for(int y=0; y<camera_->height(); ++y)
    {
      for(int x=0; x<camera_->width(); ++x)
      {
        X = bearings_C_.col(x + camera_->width() * y);
        // @TODO: derive this formula for the depth to explain it
        const FloatType numer = -T_P_C.getPosition()[2];
        const FloatType denom = T_P_C.getRotationMatrix().row(2) * X;
        const FloatType z = numer / denom;

        if(out_depthmap)
        {
          (*out_depthmap)(y,x) = (ImageFloatType) z;
        }

        // clip depth to a reasonable depth range
        if(z < z_min_ || z > z_max_)
        {
          (*out_image)(y,x) = 0;
        }
      }
    }
  }
}

} // namespace event_camera_simulator
