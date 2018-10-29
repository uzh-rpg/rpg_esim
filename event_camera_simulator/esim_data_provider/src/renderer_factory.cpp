#include <esim/common/utils.hpp>
#include <esim/data_provider/renderer_factory.hpp>
#include <esim/imp_planar_renderer/planar_renderer.hpp>
#include <esim/imp_panorama_renderer/panorama_renderer.hpp>
#include <esim/imp_opengl_renderer/opengl_renderer.hpp>
#include <esim/imp_unrealcv_renderer/unrealcv_renderer.hpp>
#include <esim/imp_multi_objects_2d/imp_multi_objects_2d_renderer.hpp>
#include <ze/cameras/camera_models.hpp>
#include <ze/cameras/camera_impl.hpp>
#include <ze/common/logging.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

DEFINE_int32(renderer_type, 0, " 0: Planar renderer, 1: Panorama renderer, 2: OpenGL renderer");

DEFINE_string(renderer_texture, "",
              "Path to image which will be used to texture the plane");

DEFINE_double(renderer_hfov_cam_source_deg, 130.0,
              "Horizontal FoV of the source camera (that captured the image on the plane)");

DEFINE_double(renderer_preprocess_median_blur, 0,
              "Kernel size of the preprocessing median blur.");

DEFINE_double(renderer_preprocess_gaussian_blur, 0,
              "Amount of preprocessing Gaussian blur.");

DEFINE_double(renderer_plane_x, 0.0,
              "x position of the center of the plane, in world coordinates");

DEFINE_double(renderer_plane_y, 0.0,
              "y position of the center of the plane, in world coordinates");

DEFINE_double(renderer_plane_z, -1.0,
              "z position of the center of the plane, in world coordinates");

DEFINE_double(renderer_plane_qw, 0.0,
              "w component of the quaternion q_W_P (orientation of the plane with respect to the world)");

DEFINE_double(renderer_plane_qx, 1.0,
              "x component of the quaternion q_W_P (orientation of the plane with respect to the world)");

DEFINE_double(renderer_plane_qy, 0.0,
              "y component of the quaternion q_W_P (orientation of the plane with respect to the world)");

DEFINE_double(renderer_plane_qz, 0.0,
              "z component of the quaternion q_W_P (orientation of the plane with respect to the world)");

DEFINE_double(renderer_z_min, 0.01,
              "Minimum clipping distance.");

DEFINE_double(renderer_z_max, 10.0,
              "Maximum clipping distance.");

DEFINE_bool(renderer_extend_border, false,
              "Whether to extend the borders of the plane to infinity or not.");

DEFINE_double(renderer_panorama_qw, 0.0,
              "w component of the quaternion q_W_P (orientation of the panorama with respect to the world)");

DEFINE_double(renderer_panorama_qx, 1.0,
              "x component of the quaternion q_W_P (orientation of the panorama with respect to the world)");

DEFINE_double(renderer_panorama_qy, 0.0,
              "y component of the quaternion q_W_P (orientation of the panorama with respect to the world)");

DEFINE_double(renderer_panorama_qz, 0.0,
              "z component of the quaternion q_W_P (orientation of the panorama with respect to the world)");

namespace event_camera_simulator {

bool loadPreprocessedImage(const std::string& path_to_img, cv::Mat* img)
{
  CHECK(img);
  VLOG(1) << "Loading texture file from file: " << FLAGS_renderer_texture << ".";
  *img = cv::imread(path_to_img, 0);
  if(!img->data)
  {
    LOG(FATAL) << "Could not open image at: " << FLAGS_renderer_texture << ".";
    return false;
  }

  if(FLAGS_renderer_preprocess_median_blur > 1)
  {
    VLOG(1) << "Pre-filtering the texture with median filter of size: "
            << FLAGS_renderer_preprocess_median_blur << ".";
    cv::medianBlur(*img, *img, FLAGS_renderer_preprocess_median_blur);
  }

  if(FLAGS_renderer_preprocess_gaussian_blur > 0)
  {
    VLOG(1) << "Pre-filtering the texture with gaussian filter of size: "
            << FLAGS_renderer_preprocess_gaussian_blur << ".";
    cv::GaussianBlur(*img, *img, cv::Size(21,21), FLAGS_renderer_preprocess_gaussian_blur);
  }

  img->convertTo(*img, cv::DataType<ImageFloatType>::type, 1.0/255.0);

  return true;
}

Renderer::Ptr loadRendererFromGflags()
{
  Renderer::Ptr renderer;

  switch (FLAGS_renderer_type)
  {
    case 0: // Planar renderer
    {
      cv::Mat img_src;
      if(!loadPreprocessedImage(FLAGS_renderer_texture, &img_src))
      {
        return nullptr;
      }

      double f_src = hfovToFocalLength(FLAGS_renderer_hfov_cam_source_deg, img_src.cols);
      Camera::Ptr cam_src = std::make_shared<ze::PinholeCamera>(
                      img_src.cols, img_src.rows, ze::CameraType::Pinhole,
                      (Vector4() << f_src, f_src, 0.5 * img_src.cols, 0.5 * img_src.rows).finished(),
                      ze::VectorX());

      Transformation T_W_P;
      T_W_P.getPosition() = ze::Position(FLAGS_renderer_plane_x,
                                         FLAGS_renderer_plane_y,
                                         FLAGS_renderer_plane_z);

      T_W_P.getRotation() = ze::Quaternion(FLAGS_renderer_plane_qw,
                                            FLAGS_renderer_plane_qx,
                                            FLAGS_renderer_plane_qy,
                                            FLAGS_renderer_plane_qz);

      renderer.reset(new PlanarRenderer(img_src, cam_src,
                                        T_W_P,
                                        FLAGS_renderer_z_min,
                                        FLAGS_renderer_z_max,
                                        FLAGS_renderer_extend_border));

      break;
    }
    case 1: // Panorama renderer
    {
       cv::Mat img_src;
       if(!loadPreprocessedImage(FLAGS_renderer_texture, &img_src))
       {
         return nullptr;
       }

        Transformation::Rotation R_W_P;
        R_W_P = ze::Quaternion(FLAGS_renderer_panorama_qw,
                               FLAGS_renderer_panorama_qx,
                               FLAGS_renderer_panorama_qy,
                               FLAGS_renderer_panorama_qz);

        renderer.reset(new PanoramaRenderer(img_src, R_W_P));
      break;
    }
    case 2: // OpenGL renderer
    {
        renderer.reset(new OpenGLRenderer());
      break;
    }
    case 3: // UnrealCV renderer
    {
        renderer.reset(new UnrealCvRenderer());
      break;
    }
    default:
    {
      LOG(FATAL) << "Renderer type is not known.";
      break;
    }
  }

  return renderer;
}

SimpleRenderer::Ptr loadSimpleRendererFromGflags()
{
  SimpleRenderer::Ptr renderer;

  switch (FLAGS_renderer_type)
  {
    case 0: // Multi-objects 2D renderer
    {
      renderer.reset(new MultiObject2DRenderer());

      break;
    }
    default:
    {
      LOG(FATAL) << "Renderer type is not known.";
      break;
    }
  }

  return renderer;
}

} // namespace event_camera_simulator
