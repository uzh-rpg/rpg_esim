#include <esim/imp_multi_objects_2d/object.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <ze/common/time_conversions.hpp>

namespace event_camera_simulator {

bool loadTexture(const std::string& path_to_img, cv::Mat* img,
                 double median_blur, double gaussian_blur)
{
  CHECK(img);
  VLOG(1) << "Loading texture file from file: " << path_to_img << ".";

  *img = cv::imread(path_to_img, cv::IMREAD_UNCHANGED);

  if(!img->data)
  {
    LOG(FATAL) << "Could not open image at: " << path_to_img << ".";
    return false;
  }

  if(median_blur > 1)
  {
    VLOG(1) << "Pre-filtering the texture with median filter of size: "
            << median_blur << ".";
    cv::medianBlur(*img, *img, median_blur);
  }

  if(gaussian_blur > 0)
  {
    VLOG(1) << "Pre-filtering the texture with gaussian filter of size: "
            << gaussian_blur << ".";
    cv::GaussianBlur(*img, *img, cv::Size(21,21), gaussian_blur);
  }
  return true;
}

Object::Object(const std::string path_to_texture, const cv::Size& dst_size, const MotionParameters& motion_params,
               double median_blur, double gaussian_blur)
  : dst_size_(dst_size),
    p_(motion_params)
{
  loadTexture(path_to_texture, &texture_,
              median_blur,
              gaussian_blur);

  K0_ << texture_.cols, 0, 0.5 * texture_.cols,
      0, texture_.rows, 0.5 * texture_.rows,
      0, 0, 1;

  K1_ << dst_size_.width, 0, 0.5 * dst_size_.width,
      0, dst_size_.height, 0.5 * dst_size_.height,
      0, 0, 1;

  canvas_ = cv::Mat(dst_size, CV_8UC4);
  canvas_.setTo(0);
  flow_ = OpticFlow(dst_size);
  flow_.setTo(0);
}

void Object::draw(Time t, bool is_first_layer)
{
  canvas_.setTo(0);

  ze::real_t ts = ze::nanosecToSecTrunc(t);

  ts = std::min(ts, p_.tmax_);

  AffineWithJacobian A10_jac = p_.getAffineTransformationWithJacobian(ts);
  Affine& A10 = A10_jac.first;
  Affine& dA10dt = A10_jac.second;

  // test jacobian
//    const ze::real_t h = 1e-5;
//    AffineWithJacobian A = p_.getAffineTransformationWithJacobian(ts);
//    AffineWithJacobian Ah = p_.getAffineTransformationWithJacobian(ts+h);
//    Affine dAdt_numeric = 1./h * (Ah.first-A.first);

//    LOG(INFO) << dAdt_numeric;
//    LOG(INFO) << A.second;

  Affine M_10 = K1_ * A10 * K0_.inv();
  Affine dM10_dt = K1_ * dA10dt * K0_.inv();

  // warpAffine requires M_dst_src unless the WARP_INVERSE_MAP flag is passed
  // in which case it will require M_src_dst
  // TODO: can we do something more efficient than fully warping the 4 channels (BGR+alpha)?
  int border_mode = is_first_layer ? cv::BORDER_REFLECT101 : cv::BORDER_CONSTANT;
  cv::warpPerspective(texture_,
                      canvas_,
                      M_10,
                      canvas_.size(),
                      CV_INTER_LINEAR,
                      border_mode);

  cv::Matx<FloatType, 3, 3> SS = dM10_dt * M_10.inv();

  for(int y=0; y<flow_.rows; ++y)
  {
    for(int x=0; x<flow_.cols; ++x)
    {
      flow_(y,x)[0] = SS(0,0) * x + SS(0,1) * y + SS(0,2);
      flow_(y,x)[1] = SS(1,0) * x + SS(1,1) * y + SS(1,2);
    }
  }
}

void getIntensityAndAlpha(const cv::Mat& image,
                          int x, int y,
                          cv::Vec<ImageFloatType, 3> *intensity,
                          ImageFloatType *alpha)
{
  CHECK(image.type() == CV_8UC3 || image.type() == CV_8UC4);

  if(image.type() == CV_8UC3)
  {
    cv::Vec3b val = image.at<cv::Vec3b>(y,x);
    *intensity = cv::Vec<ImageFloatType, 3>(static_cast<ImageFloatType>(val[0]),
                                            static_cast<ImageFloatType>(val[1]),
                                            static_cast<ImageFloatType>(val[2])) / 255.;
    *alpha = 1.;
  }
  else
  {
    cv::Vec4b val = image.at<cv::Vec4b>(y,x);
    *intensity = cv::Vec<ImageFloatType, 3>(static_cast<ImageFloatType>(val[0]),
                                            static_cast<ImageFloatType>(val[1]),
                                            static_cast<ImageFloatType>(val[2])) / 255.;
    *alpha = static_cast<ImageFloatType>(val[3]) / 255.;
  }
}

} // namespace event_camera_simulator
