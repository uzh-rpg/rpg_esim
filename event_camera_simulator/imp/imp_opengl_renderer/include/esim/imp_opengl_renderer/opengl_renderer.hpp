#pragma once

#include <esim/rendering/renderer_base.hpp>

class Shader; // fwd
class Model; // fwd
class GLFWwindow; // fwd

namespace event_camera_simulator {

//! Rendering engine based on OpenGL
class OpenGLRenderer : public Renderer
{
public:
  ZE_POINTER_TYPEDEFS(Renderer);

  OpenGLRenderer();

  ~OpenGLRenderer();

  //! Render an image at a given pose.
  virtual void render(const Transformation& T_W_C,
                      const std::vector<Transformation>& T_W_OBJ,
                      const ImagePtr& out_image,
                      const DepthmapPtr& out_depthmap) const;

  //! Returns true if the rendering engine can compute optic flow, false otherwise
  virtual bool canComputeOpticFlow() const override { return true; }

  //! Render an image + depth map + optic flow map at a given pose,
  //! given the camera linear and angular velocity
  virtual void renderWithFlow(const Transformation& T_W_C,
                            const LinearVelocity& v_WC,
                            const AngularVelocity& w_WC,
                            const std::vector<Transformation>& T_W_OBJ,
                            const std::vector<LinearVelocity>& linear_velocity_obj,
                            const std::vector<AngularVelocity>& angular_velocity_obj,
                            const ImagePtr& out_image,
                            const DepthmapPtr& out_depthmap,
                            const OpticFlowPtr& optic_flow_map) const override;

  //! Sets the camera
  virtual void setCamera(const ze::Camera::Ptr& camera);

protected:

  void init();

  /**
   @brief basic function to produce an OpenGL projection matrix and associated viewport parameters
   which match a given set of camera intrinsics. This is currently written for the Eigen linear
   algebra library, however it should be straightforward to port to any 4x4 matrix class.
   @param[out] frustum Eigen::Matrix4d projection matrix.  Eigen stores these matrices in column-major (i.e. OpenGL) order.
   @param[in]  alpha x-axis focal length, from camera intrinsic matrix
   @param[in]  alpha y-axis focal length, from camera intrinsic matrix
   @param[in]  skew  x and y axis skew, from camera intrinsic matrix
   @param[in]  u0 image origin x-coordinate, from camera intrinsic matrix
   @param[in]  v0 image origin y-coordinate, from camera intrinsic matrix
   @param[in]  img_width image width, in pixels
   @param[in]  img_height image height, in pixels
   @param[in]  near_clip near clipping plane z-location, can be set arbitrarily > 0, controls the mapping of z-coordinates for OpenGL
   @param[in]  far_clip  far clipping plane z-location, can be set arbitrarily > near_clip, controls the mapping of z-coordinate for OpenGL

   Code adapted from:
      - http://ksimek.github.io/2013/06/03/calibrated_cameras_in_opengl/
      - https://pastebin.com/h8nYNWJY
*/
  void build_opengl_projection_for_intrinsics(Eigen::Matrix4d &frustum, double alpha, double beta, double u0, double v0, int img_width, int img_height, double near_clip, double far_clip) const {

      // These parameters define the final viewport that is rendered into by
      // the camera.
      double L = 0;
      double R = img_width;
      double B = 0;
      double T = img_height;

      // near and far clipping planes, these only matter for the mapping from
      // world-space z-coordinate into the depth coordinate for OpenGL
      double N = near_clip;
      double F = far_clip;

      double skew = 0.0;

      // construct an orthographic matrix which maps from projected
      // coordinates to normalized device coordinates in the range
      // [-1, 1].  OpenGL then maps coordinates in NDC to the current
      // viewport
      Eigen::Matrix4d ortho = Eigen::Matrix4d::Zero();
      ortho(0,0) =  2.0/(R-L); ortho(0,3) = -(R+L)/(R-L);
      ortho(1,1) =  2.0/(T-B); ortho(1,3) = -(T+B)/(T-B);
      ortho(2,2) = -2.0/(F-N); ortho(2,3) = -(F+N)/(F-N);
      ortho(3,3) =  1.0;

      // construct a projection matrix, this is identical to the
      // projection matrix computed for the intrinsicx, except an
      // additional row is inserted to map the z-coordinate to
      // OpenGL.
      Eigen::Matrix4d tproj = Eigen::Matrix4d::Zero();
      tproj(0,0) = alpha; tproj(0,1) = skew; tproj(0,2) = -u0;
                          tproj(1,1) = beta; tproj(1,2) = -v0;
                                             tproj(2,2) = N+F; tproj(2,3) = N*F;
                                             tproj(3,2) = -1.0;

      // resulting OpenGL frustum is the product of the orthographic
      // mapping to normalized device coordinates and the augmented
      // camera intrinsic matrix
      frustum = ortho*tproj;
  }

  GLFWwindow* window;
  std::unique_ptr<Shader> shader;
  std::unique_ptr<Shader> optic_flow_shader;
  std::unique_ptr<Model> our_model;
  std::vector<std::unique_ptr<Model>> dynamic_objects_model;

  bool is_initialized_;

  unsigned int width_;
  unsigned int height_;

  unsigned int texture1;
  unsigned int texture2;

  unsigned int VBO, VAO;
  unsigned int multisampled_fbo, multisampled_color_buf, multisampled_depth_buf;
  unsigned int fbo, color_buf, depth_buf, depth_buf_of; // framebuffer for color and depth images
  unsigned int fbo_of, of_buf; // framebuffer for optic flow

  float zmin = 0.1f;
  float zmax = 10.0f;
};

} // namespace event_camera_simulator
