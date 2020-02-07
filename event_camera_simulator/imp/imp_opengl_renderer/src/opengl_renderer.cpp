#include <esim/imp_opengl_renderer/opengl_renderer.hpp>

#include <glad/glad.h>
#include <learnopengl/shader.h>
#include <learnopengl/model.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <learnopengl/filesystem.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace event_camera_simulator {

DEFINE_string(renderer_scene, "",
              "Path to scene file (.obj) which will be used as simulation environment");

DEFINE_double(renderer_zmin, 0.1, "Near clipping distance.");

DEFINE_double(renderer_zmax, 10, "Far clipping distance.");

DEFINE_double(renderer_alpha_ambient, 0.3, "Ambient light proportion (remaining is diffuse light)");

DEFINE_string(renderer_dynamic_objects_dir, "", "Path to directory that contains files of objects (.obj) that will be simulated");
DEFINE_string(renderer_dynamic_objects, "", "Files to be included as dynamic objects (.obj), separated by ;");

OpenGLRenderer::OpenGLRenderer()
  : is_initialized_(false),
    zmin(FLAGS_renderer_zmin),
    zmax(FLAGS_renderer_zmax)
{
}

void OpenGLRenderer::init()
{
  CHECK_GT(width_, 0);
  CHECK_GT(height_, 0);

  // glfw: initialize and configure
  // ------------------------------
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_VISIBLE, GL_FALSE);

#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // uncomment this statement to fix compilation on OS X
#endif

  // glfw window creation
  // --------------------
  window = glfwCreateWindow(width_, height_, "OpenGLRenderer", NULL, NULL);
  if (window == NULL)
  {
    LOG(FATAL) << "Failed to create GLFW window";
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(0);

  // glad: load all OpenGL function pointers
  // ---------------------------------------
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
  {
    LOG(FATAL) << "Failed to initialize GLAD";
    return;
  }

  // load models
  // -----------
  our_model.reset(new Model(FLAGS_renderer_scene));


  // load dynamics objects
  // ---------------------
  if (!FLAGS_renderer_dynamic_objects_dir.empty() && !FLAGS_renderer_dynamic_objects.empty())
  {
      size_t p_start, p_end;
      p_start = 0;
      while ((p_end = FLAGS_renderer_dynamic_objects.find(";",p_start)) != std::string::npos)
      {
          dynamic_objects_model.push_back(std::unique_ptr<Model>
          (new Model(FLAGS_renderer_dynamic_objects_dir + "/" + FLAGS_renderer_dynamic_objects.substr(p_start, p_end - p_start))));

          p_start = p_end + 1;
      }
      dynamic_objects_model.push_back(std::unique_ptr<Model>
          (new Model(FLAGS_renderer_dynamic_objects_dir + "/" + FLAGS_renderer_dynamic_objects.substr(p_start, p_end - p_start))));
  }


  // create multisampled framebuffer object
  static const int num_samples = 16;
  glGenFramebuffers(1, &multisampled_fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, multisampled_fbo);

  // create and attach a multisampled color buffer
  glGenRenderbuffers(1, &multisampled_color_buf);
  glBindRenderbuffer(GL_RENDERBUFFER, multisampled_color_buf);
  glRenderbufferStorageMultisample(GL_RENDERBUFFER, num_samples, GL_RGBA8, width_, height_); // set format
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, multisampled_color_buf); // attach color buffer to FBO

  // create and attach a multisampled depth buffer
  glGenRenderbuffers(1, &multisampled_depth_buf);
  glBindRenderbuffer(GL_RENDERBUFFER, multisampled_depth_buf);
  glRenderbufferStorageMultisample(GL_RENDERBUFFER, num_samples, GL_DEPTH_COMPONENT24, width_, height_); // set format
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, multisampled_depth_buf); // attach depth buffer to FBO

  GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  if(status != GL_FRAMEBUFFER_COMPLETE)
  {
    LOG(FATAL) << "ERROR: failed to set up multisampled framebuffer";
  }
  LOG(INFO) << "Successfully set up multisampled color and depth framebuffers";

  // create framebuffer object
  glGenFramebuffers(1, &fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo);

  // create and attach a color buffer
  glGenRenderbuffers(1, &color_buf);
  glBindRenderbuffer(GL_RENDERBUFFER, color_buf);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, width_, height_); // set format
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color_buf); // attach color buffer to FBO

  // create and attach a depth buffer
  glGenRenderbuffers(1, &depth_buf);
  glBindRenderbuffer(GL_RENDERBUFFER, depth_buf);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width_, height_); // set format
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_buf); // attach depth buffer to FBO

  status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  if(status != GL_FRAMEBUFFER_COMPLETE)
  {
    LOG(FATAL) << "ERROR: failed to set up framebuffer";
  }
  LOG(INFO) << "Successfully set up color and depth framebuffers";

  // create framebuffer object for optic flow
  glGenFramebuffers(1, &fbo_of);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_of);

  // create and attach a color buffer for optic flow
  glGenRenderbuffers(1, &of_buf);
  glBindRenderbuffer(GL_RENDERBUFFER, of_buf);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA32F, width_, height_); // set format
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, of_buf); // attach optic flow buffer to frame buffer object

  // create and attach a depth buffer for optic flow
  glGenRenderbuffers(1, &depth_buf_of);
  glBindRenderbuffer(GL_RENDERBUFFER, depth_buf_of);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width_, height_); // set format
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_buf_of); // attach depth buffer to FBO

  GLenum status_of = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  if(status_of != GL_FRAMEBUFFER_COMPLETE)
  {
    LOG(FATAL) << "ERROR: failed to set up framebuffer for optic flow";
  }
  LOG(INFO) << "Successfully set up optic flow framebuffer";

  // create shader program
  shader.reset(new Shader(FileSystem::getPath("src/shader.vert").c_str(),
                          FileSystem::getPath("src/shader.frag").c_str()));

  optic_flow_shader.reset(new Shader(FileSystem::getPath("src/shader.vert").c_str(),
                                     FileSystem::getPath("src/optic_flow_shader.frag").c_str()));

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_MULTISAMPLE);

  is_initialized_ = true;
}

OpenGLRenderer::~OpenGLRenderer()
{
  // cleanup
  glDeleteFramebuffers(1, &multisampled_fbo);
  glDeleteFramebuffers(1, &fbo);
  glDeleteFramebuffers(1, &fbo_of);

  glDeleteRenderbuffers(1, &multisampled_color_buf);
  glDeleteRenderbuffers(1, &multisampled_depth_buf);
  glDeleteRenderbuffers(1, &color_buf);
  glDeleteRenderbuffers(1, &depth_buf);
  glDeleteRenderbuffers(1, &of_buf);
  glDeleteRenderbuffers(1, &depth_buf_of);

  // glfw: terminate, clearing all previously allocated GLFW resources.
  // ------------------------------------------------------------------
  glfwTerminate();
}

void OpenGLRenderer::setCamera(const ze::Camera::Ptr& camera)
{
  CHECK(!is_initialized_) << "setCamera() was called but the OpenGL renderer was already setup before";
  CHECK(camera);
  camera_ = camera;
  width_ = camera_->width();
  height_ = camera_->height();

  if(camera->type() != ze::CameraType::Pinhole)
  {
    LOG(WARNING) << "The OpenGL rendering engine does not support camera distortion. The distortion parameters will be ignored.";
    // TODO: actually remove distortion from the camera so
    // that they do not get published on the camera_info topic
  }

  init();
}

void OpenGLRenderer::render(const Transformation& T_W_C,
                            const std::vector<Transformation>& T_W_OBJ,
                            const ImagePtr& out_image,
                            const DepthmapPtr& out_depthmap) const
{
  CHECK(is_initialized_) << "Called render() but the renderer was not initialized yet. Have you first called setCamera()?";
  CHECK(out_image);
  CHECK(out_depthmap);
  CHECK_EQ(out_image->cols, width_);
  CHECK_EQ(out_image->rows, height_);
  CHECK_EQ(out_depthmap->cols, width_);
  CHECK_EQ(out_depthmap->rows, height_);
  CHECK_EQ(out_image->type(), CV_32F);
  CHECK_EQ(out_depthmap->type(), CV_32F);

  // draw to our framebuffer instead of screen
  glBindFramebuffer(GL_FRAMEBUFFER, multisampled_fbo);

  glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  shader->use();

  glm::mat4 model = glm::mat4(1.0f);

  ze::Matrix4 T_C_W = T_W_C.inverse().getTransformationMatrix();
  // We invert the Z axis here because NDC coordinates
  // are left-handed by default in OpenGL
  // (see https://stackoverflow.com/a/12336360)
  T_C_W.block<1,4>(2,0) *= -1.0;

  // view = transformation from point in world to point in camera
  glm::mat4 view =
      glm::make_mat4(T_C_W.data());

  ze::Matrix4 frustum;
  ze::VectorX intrinsics = camera_->projectionParameters();
  const double fx = intrinsics(0);
  const double fy = intrinsics(1);
  const double cx = intrinsics(2);
  const double cy = intrinsics(3);
  build_opengl_projection_for_intrinsics(frustum, fx, fy, cx, cy, width_, height_, zmin, zmax);
  glm::mat4 projection = glm::make_mat4(frustum.data());

  unsigned int modelLoc = glGetUniformLocation(shader->ID, "model");
  glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

  unsigned int viewLoc = glGetUniformLocation(shader->ID, "view");
  glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

  unsigned int projectionLoc = glGetUniformLocation(shader->ID, "projection");
  glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection)); // TODO outside of main loop

  shader->setFloat("alpha_ambient", (float) FLAGS_renderer_alpha_ambient);

  our_model->Draw(*shader);

  // draw dynamic objects
  for (size_t i = 0; i < dynamic_objects_model.size(); i++)
  {
      ze::Matrix4 T_W_OBJ_OPENGL = T_W_OBJ[i].getTransformationMatrix();
      model = glm::make_mat4(T_W_OBJ_OPENGL.data());

      shader->setMat4("model", model);
      dynamic_objects_model[i]->Draw(*shader);
  }

  // now resolve multisampled buffer into the normal fbo
  glBindFramebuffer(GL_READ_FRAMEBUFFER, multisampled_fbo);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);
  glBlitFramebuffer(0, 0, width_, height_, 0, 0, width_, height_, GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);

  // bind fbo back so that we read from it
  glBindFramebuffer(GL_FRAMEBUFFER, fbo);

  // read out what we just rendered
  cv::Mat img_color(height_, width_, CV_8UC3);
  glPixelStorei(GL_PACK_ALIGNMENT, (img_color.step & 3)?1:4);
  glPixelStorei(GL_PACK_ROW_LENGTH, img_color.step/img_color.elemSize());

  glReadPixels(0, 0, img_color.cols, img_color.rows, GL_BGR, GL_UNSIGNED_BYTE, img_color.data);

  GLenum err = glGetError();
  if (err) {
    printf("something went wrong while reading pixels: %x\n", err);
    return;
  }

  // read out depth data
  cv::Mat img_depth(height_, width_, CV_32FC1);
  glPixelStorei(GL_PACK_ALIGNMENT, (img_depth.step & 3)?1:4);
  glPixelStorei(GL_PACK_ROW_LENGTH, img_depth.step/img_depth.elemSize());

  glReadPixels(0, 0, img_depth.cols, img_depth.rows, GL_DEPTH_COMPONENT, GL_FLOAT, img_depth.data);

  err = glGetError();
  if (err) {
    printf("something went wrong while reading depth data: %x\n", err);
    return;
  }

  // convert inverse depth buffer to linear depth between zmin and zmax
  // see the "Learn OpenGL book, page 177
  cv::Mat linear_depth = (2.0 * zmin * zmax) / (zmax + zmin - (2 * img_depth - 1.f) * (zmax - zmin));

  cv::Mat img_grayscale;
  cv::cvtColor(img_color, img_grayscale, cv::COLOR_BGR2GRAY);
  img_grayscale.convertTo(*out_image, CV_32F, 1.f/255.f);

  linear_depth.copyTo(*out_depthmap);
}


void OpenGLRenderer::renderWithFlow(const Transformation& T_W_C,
                                    const LinearVelocity& v_WC,
                                    const AngularVelocity& w_WC,
                                    const std::vector<Transformation>& T_W_OBJ,
                                    const std::vector<LinearVelocity>& linear_velocity_obj,
                                    const std::vector<AngularVelocity>& angular_velocity_obj,
                                    const ImagePtr& out_image,
                                    const DepthmapPtr& out_depthmap,
                                    const OpticFlowPtr& optic_flow_map) const
{
  render(T_W_C, T_W_OBJ, out_image, out_depthmap);

  // draw to our optic flow framebuffer instead of screen
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_of);

  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  optic_flow_shader->use();

  //@TODO: the vertex shaders should be shared
  // set projection matrices for vertex shader
  glm::mat4 model = glm::mat4(1.0f);

  Transformation T_C_W = T_W_C.inverse();
  ze::Matrix4 T_C_W_tilde = T_C_W.getTransformationMatrix();
  // We invert the Z axis here because NDC coordinates
  // are left-handed by default in OpenGL
  // (see https://stackoverflow.com/a/12336360)
  T_C_W_tilde.block<1,4>(2,0) *= -1.0;

  // view = transformation from point in world to point in camera
  glm::mat4 view =
      glm::make_mat4(T_C_W_tilde.data());

  ze::Matrix4 frustum;
  ze::VectorX intrinsics = camera_->projectionParameters();
  const double fx = intrinsics(0);
  const double fy = intrinsics(1);
  const double cx = intrinsics(2);
  const double cy = intrinsics(3);
  build_opengl_projection_for_intrinsics(frustum, fx, fy, cx, cy, width_, height_, zmin, zmax);
  glm::mat4 projection = glm::make_mat4(frustum.data());

  unsigned int modelLoc = glGetUniformLocation(optic_flow_shader->ID, "model");
  glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

  unsigned int viewLoc = glGetUniformLocation(optic_flow_shader->ID, "view");
  glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

  unsigned int projectionLoc = glGetUniformLocation(optic_flow_shader->ID, "projection");
  glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection)); // TODO, remove from main loop

  // compute optic flow and draw it to the screen buffer
  glm::vec3 w = glm::make_vec3(w_WC.data());
  glm::vec3 v = glm::make_vec3(v_WC.data());

  optic_flow_shader->setVec3("w_WC", w);
  optic_flow_shader->setVec3("v_WC", v);

  optic_flow_shader->setBool("dynamic_object", false);

  optic_flow_shader->setFloat("fx", (float) fx);
  optic_flow_shader->setFloat("fy", (float) fy);
  optic_flow_shader->setFloat("cx", (float) cx);
  optic_flow_shader->setFloat("cy", (float) cy);
  optic_flow_shader->setFloat("width", (float) width_);
  optic_flow_shader->setFloat("height", (float) height_);

  optic_flow_shader->setFloat("near", (float) zmin);
  optic_flow_shader->setFloat("far", (float) zmax);

  our_model->Draw(*optic_flow_shader);

  // draw optical flow for dynamic objects
  for (size_t i = 0; i < dynamic_objects_model.size(); i++)
  {
      optic_flow_shader->setBool("dynamic_object", true);

      // relative position (in camera frame)
      ze::Vector3 C_r_C_OBJ = (T_C_W*T_W_OBJ[i]).getPosition();
      glm::vec3 r_C_OBJ = glm::make_vec3(C_r_C_OBJ.data());
      optic_flow_shader->setVec3("r_CB", r_C_OBJ);

      // linear velocity (in camera frame)
      ze::Vector3 C_v_OBJ = T_C_W.getRotation().rotate(linear_velocity_obj[i]);
      glm::vec3 v_obj = glm::make_vec3(C_v_OBJ.data());
      optic_flow_shader->setVec3("v_WB", v_obj);


      // angular velocity (in camera frame)
      ze::Vector3 C_w_OBJ = T_C_W.getRotation().rotate(angular_velocity_obj[i]);
      glm::vec3 w_obj = glm::make_vec3(C_w_OBJ.data());
      optic_flow_shader->setVec3("w_WB", w_obj);

      ze::Matrix4 T_W_OBJ_OPENGL = T_W_OBJ[i].getTransformationMatrix();
      model = glm::make_mat4(T_W_OBJ_OPENGL.data());
      optic_flow_shader->setMat4("model", model);


      dynamic_objects_model[i]->Draw(*optic_flow_shader);
  }

  // read out the optic flow we just rendered
  cv::Mat flow(height_, width_, CV_32FC3);
  glPixelStorei(GL_PACK_ALIGNMENT, (flow.step & 3)?1:4);
  glPixelStorei(GL_PACK_ROW_LENGTH, flow.step/flow.elemSize());

  glReadPixels(0, 0, flow.cols, flow.rows, GL_BGR, GL_FLOAT, flow.data);

  GLenum err = glGetError();
  if (err) {
    LOG(ERROR) << "something went wrong while reading pixels: " << err;
    return;
  }

  for(int y=0; y<height_; ++y)
  {
    for(int x=0; x<width_; ++x)
    {
      (*optic_flow_map)(y,x)
          = cv::Vec<ImageFloatType,2>(flow.at<cv::Vec<ImageFloatType,3>>(y,x)[2],
                                      flow.at<cv::Vec<ImageFloatType,3>>(y,x)[1]);
    }
  }
}


} // namespace event_camera_simulator
