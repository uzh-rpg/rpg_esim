#pragma once

#include <esim/common/types.hpp>

namespace event_camera_simulator {

bool loadTexture(const std::string& path_to_img, cv::Mat *img,
                 double median_blur, double gaussian_blur);

using Affine = cv::Matx<FloatType, 3,3>;
using AffineWithJacobian = std::pair<Affine, Affine>;

class MotionParameters
{
public:
  MotionParameters(FloatType tmax,
                   FloatType theta0_deg, FloatType theta1_deg,
                   FloatType x0, FloatType x1,
                   FloatType y0, FloatType y1,
                   FloatType sx0, FloatType sx1,
                   FloatType sy0, FloatType sy1)
    : tmax_(tmax),
      x0_(x0),
      x1_(x1),
      y0_(y0),
      y1_(y1),
      theta0_(theta0_deg * CV_PI / 180.),
      theta1_(theta1_deg * CV_PI / 180.),
      sx0_(sx0),
      sx1_(sx1),
      sy0_(sy0),
      sy1_(sy1)
  {
  }

  AffineWithJacobian getAffineTransformationWithJacobian(ze::real_t t)
  {
    // constants
    const ze::real_t dtheta = theta1_ - theta0_;
    const ze::real_t dx = x1_ - x0_;
    const ze::real_t dy = y1_ - y0_;
    const ze::real_t dsx = sx1_ - sx0_;
    const ze::real_t dsy = sy1_ - sy0_;

    // computation of parameter(t)
    const ze::real_t theta = theta0_ + t/tmax_ * dtheta;
    const ze::real_t x = x0_ + t/tmax_ * dx;
    const ze::real_t y = y0_ + t/tmax_ * dy;
    const ze::real_t sx = sx0_ + t/tmax_ * dsx;
    const ze::real_t sy = sy0_ + t/tmax_ * dsy;
    const ze::real_t stheta = std::sin(theta);
    const ze::real_t ctheta = std::cos(theta);

    Affine A;
    A << sx * ctheta, -sy * stheta, x,
         sx * stheta, sy * ctheta,  y,
           0,               0,                1;

    // computation of dparameter_dt(t)
    const ze::real_t dtheta_dt = 1./tmax_ * dtheta;
    const ze::real_t dx_dt = 1./tmax_ * dx;
    const ze::real_t dy_dt = 1./tmax_ * dy;
    const ze::real_t dsx_dt = 1./tmax_ * dsx;
    const ze::real_t dsy_dt = 1./tmax_ * dsy;

    cv::Matx<FloatType, 3, 3> dAdt;
    dAdt << dsx_dt * ctheta - dtheta_dt * stheta * sx, -dsy_dt * stheta - dtheta_dt * ctheta * sy, dx_dt,
            dsx_dt * stheta + dtheta_dt * ctheta * sx, dsy_dt * ctheta - dtheta_dt * stheta * sy, dy_dt,
            0.0,                                       0.0,                                       0.0;

    return AffineWithJacobian(A, dAdt);
  }

  FloatType tmax_;
  FloatType x0_, x1_;
  FloatType y0_, y1_;
  FloatType theta0_, theta1_;
  FloatType sx0_, sx1_;
  FloatType sy0_, sy1_;
};

class Object
{
public:
  Object(const std::string path_to_texture, const cv::Size& dst_size, const MotionParameters& motion_params,
         double median_blur, double gaussian_blur);

  void draw(Time t, bool is_first_layer = false);

  cv::Mat canvas_;
  OpticFlow flow_;

  MotionParameters getMotionParameters() const { return p_; }

  Affine getK0() const { return K0_; }
  Affine getK1() const { return K1_; }

private:
  cv::Mat texture_;
  cv::Size dst_size_;
  MotionParameters p_;

  Affine K0_, K1_;
};

void getIntensityAndAlpha(const cv::Mat& image,
                          int x, int y,
                          ImageFloatType* intensity,
                          ImageFloatType* alpha);

inline ImageFloatType bgrToGrayscale(uchar b,
                                     uchar g,
                                     uchar r)
{
  // https://www.johndcook.com/blog/2009/08/24/algorithms-convert-color-grayscale/
  return 0.07 * static_cast<ImageFloatType>(b)
      + 0.72 * static_cast<ImageFloatType>(g)
      + 0.21 * static_cast<ImageFloatType>(r);
}

} // namespace event_camera_simulator
