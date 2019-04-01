#pragma once

#include <ze/common/transformation.hpp>
#include <opencv2/core/core.hpp>
#include <ze/cameras/camera_rig.hpp>
#include <memory>

// FloatType defines the floating point accuracy (single or double precision)
// for the geometric operations (computing rotation matrices, point projection, etc.).
// This should typically be double precision (highest accuracy).
#define FloatType ze::real_t

// ImageFloatType defines the floating point accuracy (single or double precision)
// of the intensity images (and depth images).
// Single precision should be enough there in most cases.
#define ImageFloatType float

namespace event_camera_simulator {

using Translation = ze::Position;
using Vector3 = ze::Vector3;
using Vector4 = ze::Vector4;
using Vector3i = Eigen::Vector3i;

using Transformation = ze::Transformation;
using TransformationVector = ze::TransformationVector;
using TransformationPtr = std::shared_ptr<Transformation>;

using Normal = ze::Vector3;
using CalibrationMatrix = ze::Matrix3;
using RotationMatrix = ze::Matrix3;
using HomographyMatrix = ze::Matrix3;

using AngularVelocity = ze::Vector3;
using LinearVelocity = ze::Vector3;
using AngularVelocityVector = std::vector<AngularVelocity>;
using LinearVelocityVector = std::vector<LinearVelocity>;

using uint16_t = ze::uint16_t;

using Time = ze::int64_t;
using Duration = ze::uint64_t;
using Image = cv::Mat_<ImageFloatType>;
using ColorImage = cv::Mat_<cv::Vec<ImageFloatType, 3>>;
using ImagePtr = std::shared_ptr<Image>;
using ColorImagePtr = std::shared_ptr<ColorImage>;
using Depthmap = cv::Mat_<ImageFloatType>;
using OpticFlow = cv::Mat_< cv::Vec<ImageFloatType, 2> >;
using OpticFlowPtr = std::shared_ptr<OpticFlow>;
using DepthmapPtr = std::shared_ptr<Depthmap>;

using ImagePtrVector = std::vector<ImagePtr>;
using ColorImagePtrVector = std::vector<ColorImagePtr>;
using DepthmapPtrVector = std::vector<DepthmapPtr>;
using OpticFlowPtrVector = std::vector<OpticFlowPtr>;

using Camera = ze::Camera;

struct Event
{
  Event(uint16_t x, uint16_t y, Time t, bool pol)
    : x(x),
      y(y),
      t(t),
      pol(pol)
  {

  }

  uint16_t x;
  uint16_t y;
  Time t;
  bool pol;
};

using Events = std::vector<Event>;
using EventsVector = std::vector<Events>;
using EventsPtr = std::shared_ptr<Events>;

struct PointXYZRGB
{
  PointXYZRGB(FloatType x, FloatType y, FloatType z,
              int red, int green, int blue)
    : xyz(x, y, z),
      rgb(red, green, blue) {}

  PointXYZRGB(const Vector3& xyz)
    : xyz(xyz) {}

  PointXYZRGB(const Vector3& xyz, const Vector3i& rgb)
    : xyz(xyz),
      rgb(rgb) {}

  Vector3 xyz;
  Vector3i rgb;
};

using PointCloud = std::vector<PointXYZRGB>;
using PointCloudVector = std::vector<PointCloud>;

struct SimulatorData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! Nanosecond timestamp.
  Time timestamp;

  //! Camera images.
  ColorImagePtrVector images;

  //! Depth maps.
  DepthmapPtrVector depthmaps;

  //! Optic flow maps.
  OpticFlowPtrVector optic_flows;

  //! Camera
  ze::CameraRig::Ptr camera_rig;

  //! An accelerometer measures the specific force (incl. gravity),
  //! corrupted by noise and bias.
  Vector3 specific_force_corrupted;

  //! The angular velocity (in the body frame) corrupted by noise and bias.
  Vector3 angular_velocity_corrupted;

  //! Groundtruth states.
  struct Groundtruth
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Pose of the body (i.e. the IMU) expressed in the world frame.
    Transformation T_W_B;

    //! Accelerometer and gyro bias
    Vector3 acc_bias;
    Vector3 gyr_bias;

    //! Poses of the cameras in the rig expressed in the world frame.
    TransformationVector T_W_Cs;

    //! Linear and angular velocities (i.e. twists) of the cameras in the rig,
    //! expressed in each camera's local coordinate frame.
    LinearVelocityVector linear_velocities_;
    AngularVelocityVector angular_velocities_;

    // dynamic objects
    std::vector<Transformation> T_W_OBJ_;
    std::vector<LinearVelocity> linear_velocity_obj_;
    std::vector<AngularVelocity> angular_velocity_obj_;
  };
  Groundtruth groundtruth;

  // Flags to indicate whether a value has been updated or not
  bool images_updated;
  bool depthmaps_updated;
  bool optic_flows_updated;
  bool twists_updated;
  bool poses_updated;
  bool imu_updated;
};

} // namespace event_camera_simulator
