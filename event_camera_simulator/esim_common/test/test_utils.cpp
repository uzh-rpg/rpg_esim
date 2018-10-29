#include <esim/common/utils.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/common/file_utils.hpp>
#include <ze/common/path_utils.hpp>
#include <ze/common/string_utils.hpp>
#include <ze/cameras/camera_rig.hpp>
#include <ze/common/random.hpp>

std::string getTestDataDir(const std::string& dataset_name)
{
  using namespace ze;

  const char* datapath_dir = std::getenv("ESIM_TEST_DATA_PATH");
  CHECK(datapath_dir != nullptr)
      << "Did you download the esim_test_data repository and set "
      << "the ESIM_TEST_DATA_PATH environment variable?";

  std::string path(datapath_dir);
  CHECK(isDir(path)) << "Folder does not exist: " << path;
  path = path + "/data/" + dataset_name;
  CHECK(isDir(path)) << "Dataset does not exist: " << path;
  return path;
}

namespace event_camera_simulator {

void computeOpticFlowFiniteDifference(const ze::Camera::Ptr& camera,
                                      const ze::Vector3& angular_velocity,
                                      const ze::Vector3& linear_velocity,
                                      const DepthmapPtr& depth,
                                      OpticFlowPtr& flow)
{
  CHECK(flow);
  CHECK_EQ(flow->rows, camera->height());
  CHECK_EQ(flow->cols, camera->width());

  const FloatType dt = 0.001;

  for(int y=0; y<flow->rows; ++y)
  {
    for(int x=0; x<flow->cols; ++x)
    {
      ze::Keypoint u_t(x,y);
      ze::Bearing f = camera->backProject(u_t);
      const FloatType z = static_cast<FloatType>((*depth)(y,x));
      ze::Position Xc_t = z * ze::Position(f[0]/f[2], f[1]/f[2], 1.);

      ze::Transformation::Rotation dR = ze::Transformation::Rotation::exp(-angular_velocity * dt);
      ze::Transformation::Position dT = -linear_velocity * dt;

      // Transform Xc(t) to Xc(t+dt)
      ze::Transformation T_tdt_t;
      T_tdt_t.getRotation() = dR;
      T_tdt_t.getPosition() = dT;
      VLOG(5) << T_tdt_t;

      ze::Position Xc_t_dt = T_tdt_t.transform(Xc_t);

      // Project Xc(t+dt) in the image plane
      ze::Keypoint u_t_dt = camera->project(Xc_t_dt);
      VLOG(5) << u_t;
      VLOG(5) << u_t_dt;

      ze::Vector2 flow_vec = (u_t_dt - u_t) / dt;
      (*flow)(y,x) = cv::Vec<FloatType, 2>(flow_vec(0), flow_vec(1));
    }
  }
}

void computeOpticFlowFiniteDifference(const ze::Camera::Ptr& camera,
                                      const ze::Vector3& angular_velocity,
                                      const ze::Vector3& linear_velocity,
                                      const DepthmapPtr& depth,
                                      const ze::Vector3& r_COBJ,
                                      const ze::Vector3& angular_velocity_obj,
                                      const ze::Vector3& linear_velocity_obj,
                                      OpticFlowPtr& flow)
{
  CHECK(flow);
  CHECK_EQ(flow->rows, camera->height());
  CHECK_EQ(flow->cols, camera->width());

  const FloatType dt = 0.001;

  for(int y=0; y<flow->rows; ++y)
  {
    for(int x=0; x<flow->cols; ++x)
    {
      ze::Keypoint u_t(x,y);
      ze::Bearing f = camera->backProject(u_t);
      const FloatType z = static_cast<FloatType>((*depth)(y,x));
      ze::Position Xc_t = z * ze::Position(f[0]/f[2], f[1]/f[2], 1.);
      ze::Position r_OBJX = Xc_t - r_COBJ;
      ze::Matrix33 w_WOBJ_tilde = ze::skewSymmetric(angular_velocity_obj);

      ze::Transformation::Rotation dR = ze::Transformation::Rotation::exp(-angular_velocity * dt);
      ze::Transformation::Position dT = linear_velocity_obj*dt - linear_velocity * dt + w_WOBJ_tilde*r_OBJX*dt;

      // Transform Xc(t) to Xc(t+dt)
      ze::Transformation T_tdt_t;
      T_tdt_t.getRotation() = dR;
      T_tdt_t.getPosition() = dT;
      VLOG(5) << T_tdt_t;

      ze::Position Xc_t_dt = T_tdt_t.transform(Xc_t);

      // Project Xc(t+dt) in the image plane
      ze::Keypoint u_t_dt = camera->project(Xc_t_dt);
      VLOG(5) << u_t;
      VLOG(5) << u_t_dt;

      ze::Vector2 flow_vec = (u_t_dt - u_t) / dt;
      (*flow)(y,x) = cv::Vec<FloatType, 2>(flow_vec(0), flow_vec(1));
    }
  }
}

} // event_camera_simulator

TEST(Utils, testOpticFlowComputation)
{
  using namespace event_camera_simulator;

  // Load camera calib from folder
  const std::string path_to_data_folder = getTestDataDir("camera_calibs");
  ze::CameraRig::Ptr camera_rig
      = ze::cameraRigFromYaml(ze::joinPath(path_to_data_folder, "pinhole_mono.yaml"));

  CHECK(camera_rig);

  const ze::Camera::Ptr camera = camera_rig->atShared(0);
  cv::Size sensor_size(camera->width(), camera->height());

  OpticFlowPtr flow_analytic =
      std::make_shared<OpticFlow>(sensor_size);

  // Sample random depth map
  const ImageFloatType z_mean = 5.0;
  const ImageFloatType z_stddev = 0.5;
  DepthmapPtr depth = std::make_shared<Depthmap>(sensor_size);
  for(int y=0; y<sensor_size.height; ++y)
  {
    for(int x=0; x<sensor_size.width; ++x)
    {
      (*depth)(y,x) = ze::sampleNormalDistribution(true, z_mean, z_stddev);
    }
  }

  // Sample random linear and angular velocity
  ze::Vector3 angular_velocity, linear_velocity;
  angular_velocity.setRandom();
  linear_velocity.setRandom();

  LOG(INFO) << "w = " << angular_velocity;
  LOG(INFO) << "v = " << linear_velocity;

  // Compute optic flow on the image plane according
  // to the sampled angular/linear velocity
  OpticFlowHelper optic_flow_helper(camera);
  optic_flow_helper.computeFromDepthAndTwist(angular_velocity, linear_velocity,
                                             depth, flow_analytic);

  OpticFlowPtr flow_finite_diff =
      std::make_shared<OpticFlow>(sensor_size);

  computeOpticFlowFiniteDifference(camera, angular_velocity, linear_velocity,
                                   depth, flow_finite_diff);

  // Check that the analytical flow and finite-difference flow match
  for(int y=0; y<sensor_size.height; ++y)
  {
    for(int x=0; x<sensor_size.width; ++x)
    {
      EXPECT_NEAR((*flow_analytic)(y,x)[0], (*flow_finite_diff)(y,x)[0], 0.1);
      EXPECT_NEAR((*flow_analytic)(y,x)[1], (*flow_finite_diff)(y,x)[1], 0.1);
    }
  }

  /**********************************************/
  /* repeat optic flow test for dynamic objects */
  /**********************************************/

  // sample random obj position and linear and angular velocity
  ze::Vector3 r_COBJ;
  r_COBJ.setRandom();
  r_COBJ(2) = ze::sampleNormalDistribution(true, z_mean, z_stddev); // assume object is in front of camera

  ze::Vector3 angular_velocity_obj, linear_velocity_obj;
  angular_velocity_obj.setRandom();
  linear_velocity_obj.setRandom();

  // Compute optic flow on the image plane according
  // to the sampled angular/linear velocity
  optic_flow_helper.computeFromDepthCamTwistAndObjDepthAndTwist(angular_velocity, linear_velocity, depth,
                                                                r_COBJ, angular_velocity_obj, linear_velocity_obj,
                                                                flow_analytic);

  computeOpticFlowFiniteDifference(camera, angular_velocity, linear_velocity, depth,
                                    r_COBJ, angular_velocity_obj, linear_velocity_obj,
                                    flow_finite_diff);

  // Check that the analytical flow and finite-difference flow match
  for(int y=0; y<sensor_size.height; ++y)
  {
    for(int x=0; x<sensor_size.width; ++x)
    {
      EXPECT_NEAR((*flow_analytic)(y,x)[0], (*flow_finite_diff)(y,x)[0], 0.1);
      EXPECT_NEAR((*flow_analytic)(y,x)[1], (*flow_finite_diff)(y,x)[1], 0.1);
    }
  }
}

ZE_UNITTEST_ENTRYPOINT
