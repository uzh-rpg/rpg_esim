#include <esim/common/utils.hpp>
#include <ze/cameras/camera_models.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace event_camera_simulator {

OpticFlowHelper::OpticFlowHelper(const ze::Camera::Ptr& camera)
  : camera_(camera)
{
  CHECK(camera_);
  precomputePixelToBearingLookupTable();
}

void OpticFlowHelper::precomputePixelToBearingLookupTable()
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

void OpticFlowHelper::computeFromDepthAndTwist(const ze::Vector3& w_WC, const ze::Vector3& v_WC,
                                                   const DepthmapPtr& depthmap, OpticFlowPtr& flow)
{
  CHECK(depthmap);
  CHECK_EQ(depthmap->rows, camera_->height());
  CHECK_EQ(depthmap->cols, camera_->width());
  CHECK(depthmap->isContinuous());

  const ze::Vector3 w_CW = -w_WC; // rotation speed of the world wrt the camera
  const ze::Vector3 v_CW = -v_WC; // speed of the world wrt the camera
  const ze::Matrix33 R_CW = ze::skewSymmetric(w_CW);

  Eigen::Map<const Eigen::Matrix<ImageFloatType, 1, Eigen::Dynamic, Eigen::RowMajor>> depths(depthmap->ptr<ImageFloatType>(), 1, depthmap->rows * depthmap->cols);
  ze::Positions Xs = bearings_C_;
  Xs.array().rowwise() *= depths.cast<FloatType>().array();

  ze::Matrix6X dproject_dX =
      camera_->dProject_dLandmarkVectorized(Xs);

  for(int y=0; y<camera_->height(); ++y)
  {
    for(int x=0; x<camera_->width(); ++x)
    {
      const Vector3 X = Xs.col(x + camera_->width() * y);
      ze::Matrix31 dXdt = R_CW * X + v_CW;
      ze::Vector2 flow_vec
          = Eigen::Map<ze::Matrix23>(dproject_dX.col(x + camera_->width() * y).data()) * dXdt;

      (*flow)(y,x) = cv::Vec<FloatType, 2>(flow_vec(0), flow_vec(1));
    }
  }
}

void OpticFlowHelper::computeFromDepthCamTwistAndObjDepthAndTwist(const ze::Vector3& w_WC, const ze::Vector3& v_WC, const DepthmapPtr& depthmap,
                                            const ze::Vector3& r_COBJ, const ze::Vector3& w_WOBJ, const ze::Vector3& v_WOBJ, OpticFlowPtr& flow)
{
  CHECK(depthmap);
  CHECK_EQ(depthmap->rows, camera_->height());
  CHECK_EQ(depthmap->cols, camera_->width());
  CHECK(depthmap->isContinuous());

  const ze::Matrix33 w_WC_tilde = ze::skewSymmetric(w_WC);
  const ze::Matrix33 w_WOBJ_tilde = ze::skewSymmetric(w_WOBJ);

  Eigen::Map<const Eigen::Matrix<ImageFloatType, 1, Eigen::Dynamic, Eigen::RowMajor>> depths(depthmap->ptr<ImageFloatType>(), 1, depthmap->rows * depthmap->cols);
  ze::Positions Xs = bearings_C_;
  Xs.array().rowwise() *= depths.cast<FloatType>().array();

  ze::Matrix6X dproject_dX =
      camera_->dProject_dLandmarkVectorized(Xs);

  for(int y=0; y<camera_->height(); ++y)
  {
    for(int x=0; x<camera_->width(); ++x)
    {
      const Vector3 r_CX = Xs.col(x + camera_->width() * y);
      const Vector3 r_OBJX = r_CX - r_COBJ;

      ze::Matrix31 dXdt = v_WOBJ - v_WC - w_WC_tilde*r_CX + w_WOBJ_tilde*r_OBJX;
      ze::Vector2 flow_vec
          = Eigen::Map<ze::Matrix23>(dproject_dX.col(x + camera_->width() * y).data()) * dXdt;

      (*flow)(y,x) = cv::Vec<FloatType, 2>(flow_vec(0), flow_vec(1));
    }
  }
}

FloatType maxMagnitudeOpticFlow(const OpticFlowPtr& flow)
{
  CHECK(flow);
  FloatType max_squared_magnitude = 0;
  for(int y=0; y<flow->rows; ++y)
  {
    for(int x=0; x<flow->cols; ++x)
    {
      const FloatType squared_magnitude = cv::norm((*flow)(y,x), cv::NORM_L2SQR);
      if(squared_magnitude > max_squared_magnitude)
      {
        max_squared_magnitude = squared_magnitude;
      }
    }
  }
  return std::sqrt(max_squared_magnitude);
}

FloatType maxPredictedAbsBrightnessChange(const ImagePtr& I, const OpticFlowPtr& flow)
{
  const size_t height = I->rows;
  const size_t width = I->cols;

  Image Ix, Iy; // horizontal/vertical gradients of I
  // the factor 1/8 accounts for the scaling introduced by the Sobel filter mask
  //cv::Sobel(*I, Ix, cv::DataType<ImageFloatType>::type, 1, 0, 3, 1./8.);
  //cv::Sobel(*I, Iy, cv::DataType<ImageFloatType>::type, 0, 1, 3, 1./8.);

  // the factor 1/32 accounts for the scaling introduced by the Scharr filter mask
  cv::Scharr(*I, Ix, cv::DataType<ImageFloatType>::type, 1, 0, 1./32.);
  cv::Scharr(*I, Iy, cv::DataType<ImageFloatType>::type, 0, 1, 1./32.);

  Image Lx, Ly; // horizontal/vertical gradients of log(I). d(logI)/dx = 1/I * dI/dx
  static const ImageFloatType eps = 1e-3; // small additive term to avoid problems at I=0
  cv::divide(Ix, *I+eps, Lx);
  cv::divide(Iy, *I+eps, Ly);

  Image dLdt(height, width);
  for(int y=0; y<height; ++y)
  {
    for(int x=0; x<width; ++x)
    {
      // dL/dt ~= - <nablaL, flow>
      const ImageFloatType dLdt_at_xy =
          Lx(y,x) * (*flow)(y,x)[0] +
          Ly(y,x) * (*flow)(y,x)[1]; // "-" sign ignored since we are interested in the absolute value...
      dLdt(y,x) = std::fabs(dLdt_at_xy);
    }
  }
  double min_dLdt, max_dLdt;
  int min_idx, max_idx;
  cv::minMaxIdx(dLdt, &min_dLdt, &max_dLdt,
                &min_idx, &max_idx);

  return static_cast<FloatType>(max_dLdt);
}

void gaussianBlur(ImagePtr& I, FloatType sigma)
{
  cv::GaussianBlur(*I, *I, cv::Size(15,15), sigma, sigma);
}

CalibrationMatrix calibrationMatrixFromCamera(const Camera::Ptr& camera)
{
  CHECK(camera);
  const ze::VectorX params = camera->projectionParameters();
  CalibrationMatrix K;
  K << params(0), 0,         params(2),
       0,         params(1), params(3),
       0,         0,     1;
  return K;
}

PointCloud eventsToPointCloud(const Events& events, const Depthmap& depthmap, const ze::Camera::Ptr& camera)
{
  PointCloud pcl_camera;
  for(const Event& ev : events)
  {
    Vector3 X_c = camera->backProject(ze::Keypoint(ev.x,ev.y));
    X_c[0] /= X_c[2];
    X_c[1] /= X_c[2];
    X_c[2] = 1.;
    const ImageFloatType z = depthmap(ev.y,ev.x);
    Vector3 P_c = z * X_c;
    Vector3i rgb;
    static const Vector3i red(255, 0, 0);
    static const Vector3i blue(0, 0, 255);
    rgb = (ev.pol) ? blue : red;
    PointXYZRGB P_c_intensity(P_c, rgb);
    pcl_camera.push_back(P_c_intensity);
  }
  return pcl_camera;
}

} // namespace event_camera_simulator
