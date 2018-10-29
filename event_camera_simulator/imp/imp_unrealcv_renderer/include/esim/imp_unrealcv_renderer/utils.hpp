#pragma once

#include <esim/common/types.hpp>

namespace event_camera_simulator {

/**
 * https://github.com/EpicGames/UnrealEngine/blob/dbced2dd59f9f5dfef1d7786fd67ad2970adf95f/Engine/Source/Runtime/Core/Public/Math/Rotator.h#L580
 * Helper function for eulerFromQuatSingularityTest, angles are expected to be given in degrees
 **/
inline FloatType clampAxis(FloatType angle)
{
  // returns angle in the range (-360,360)
  angle = std::fmod(angle, 360.f);

  if (angle < 0.f)
  {
    // shift to [0,360) range
    angle += 360.f;
  }

  return angle;
}

/**
 * https://github.com/EpicGames/UnrealEngine/blob/dbced2dd59f9f5dfef1d7786fd67ad2970adf95f/Engine/Source/Runtime/Core/Public/Math/Rotator.h#L595$
 * Helper function for eulerFromQuatSingularityTest, angles are expected to be given in degrees
 **/
inline FloatType normalizeAxis(FloatType angle)
{
  angle = clampAxis(angle);
  if(angle > 180.f)
  {
    // shift to (-180,180]
    angle -= 360.f;
  }

  return angle;
}


/**
  *
  * https://github.com/EpicGames/UnrealEngine/blob/f794321ffcad597c6232bc706304c0c9b4e154b2/Engine/Source/Runtime/Core/Private/Math/UnrealMath.cpp#L540
  * Quaternion given in (x,y,z,w) representation
  **/
void quaternionToEulerUnrealEngine(const Transformation::Rotation& q, FloatType& yaw, FloatType& pitch, FloatType& roll)
{
  const FloatType X = q.x();
  const FloatType Y = q.y();
  const FloatType Z = q.z();
  const FloatType W = q.w();

  const FloatType SingularityTest = Z*X-W*Y;
  const FloatType YawY = 2.f*(W*Z+X*Y);
  const FloatType YawX = (1.f-2.f*(Y*Y + Z*Z));

  // reference
  // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/

  // this value was found from experience, the above websites recommend different values
  // but that isn't the case for us, so I went through different testing, and finally found the case
  // where both of world lives happily.
  const FloatType SINGULARITY_THRESHOLD = 0.4999995;
  const FloatType RAD_TO_DEG = (180.0)/CV_PI;

  if (SingularityTest < -SINGULARITY_THRESHOLD)
  {
    pitch = -90.;
    yaw = std::atan2(YawY, YawX) * RAD_TO_DEG;
    roll = normalizeAxis(-yaw - (2.f * std::atan2(X, W) * RAD_TO_DEG));
  }
  else if (SingularityTest > SINGULARITY_THRESHOLD)
  {
    pitch = 90.;
    yaw = std::atan2(YawY, YawX) * RAD_TO_DEG;
    roll = normalizeAxis(yaw - (2.f * std::atan2(X, W) * RAD_TO_DEG));
  }
  else
  {
    pitch = std::asin(2.f*(SingularityTest)) * RAD_TO_DEG;
    yaw = std::atan2(YawY, YawX) * RAD_TO_DEG;
    roll = std::atan2(-2.f*(W*X+Y*Z), (1.f-2.f*(X*X + Y*Y))) * RAD_TO_DEG;
  }
}

} // namespace event_camera_simulator
