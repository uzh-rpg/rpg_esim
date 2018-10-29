#pragma once

#include <esim/common/types.hpp>

namespace ze {
  class Camera;
}

namespace event_camera_simulator {

inline double degToRad(double deg)
{
  return deg * CV_PI / 180.0;
}

inline double hfovToFocalLength(double hfov_deg, int W)
{
  return 0.5 * static_cast<double>(W) / std::tan(0.5 * degToRad(hfov_deg));
}

CalibrationMatrix calibrationMatrixFromCamera(const Camera::Ptr& camera);

PointCloud eventsToPointCloud(const Events& events, const Depthmap& depthmap, const ze::Camera::Ptr& camera);

FloatType maxMagnitudeOpticFlow(const OpticFlowPtr& flow);

FloatType maxPredictedAbsBrightnessChange(const ImagePtr& I, const OpticFlowPtr& flow);

void gaussianBlur(ImagePtr& I, FloatType sigma);

// Helper class to compute optic flow from a twist vector and depth map
// Precomputes a lookup table for pixel -> bearing vector correspondences
// to accelerate the computation
class OpticFlowHelper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ZE_POINTER_TYPEDEFS(OpticFlowHelper);

  OpticFlowHelper(const ze::Camera::Ptr& camera);

  void computeFromDepthAndTwist(const ze::Vector3& w_WC, const ze::Vector3& v_WC,
                                    const DepthmapPtr& depthmap, OpticFlowPtr& flow);

  void computeFromDepthCamTwistAndObjDepthAndTwist(const ze::Vector3& w_WC, const ze::Vector3& v_WC, const DepthmapPtr& depthmap,
                                            const ze::Vector3& r_COBJ, const ze::Vector3& w_WOBJ, const ze::Vector3& v_WOBJ,
                                            OpticFlowPtr& flow);

private:

  void precomputePixelToBearingLookupTable();

  ze::Camera::Ptr camera_;

  // Precomputed lookup table from keypoint -> bearing vector
  ze::Bearings bearings_C_;
};
} // namespace event_camera_simulator
