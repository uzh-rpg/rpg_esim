#include <esim/common/utils.hpp>
#include <esim/imp_multi_objects_2d/imp_multi_objects_2d_renderer.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <glog/logging.h>
#include <opencv2/core/eigen.hpp>
#include <ze/common/file_utils.hpp>
#include <ze/common/time_conversions.hpp>

DECLARE_double(renderer_preprocess_median_blur);

DECLARE_double(renderer_preprocess_gaussian_blur);

DEFINE_string(path_to_sequence_file, "",
              "Path to the sequence file that describes the 2D, multi-object scene.");

DEFINE_string(path_to_output_folder, "",
              "Path to the output folder where the flow ground truth files will be written");

DEFINE_bool(output_reverse_displacement_map, false,
            "Whether to output the reverse displacement map (i.e., the displacement field that maps pixels at t=tmax to pixels at t=t0");

namespace event_camera_simulator {

MultiObject2DRenderer::MultiObject2DRenderer()
{
  // Load sequence from file
  const std::string path_to_sequence_file = FLAGS_path_to_sequence_file;

  std::ifstream sequence_file;
  ze::openFileStream(path_to_sequence_file, &sequence_file);

  std::string path_to_texture;
  double median_blur, gaussian_blur, theta0, theta1, x0, x1, y0, y1, sx0, sx1, sy0, sy1;

  sequence_file >> width_ >> height_ >> tmax_;

  CHECK_GT(width_, 0);
  CHECK_GT(height_, 0);
  CHECK_GT(tmax_, 0);

  VLOG(1) << "width = " << width_ << ", height = " << height_;
  VLOG(1) << "tmax = " << tmax_;

  while(sequence_file >> path_to_texture
        >> median_blur >> gaussian_blur
        >> theta0 >> theta1
        >> x0 >> x1
        >> y0 >> y1
        >> sx0 >> sx1
        >> sy0 >> sy1)
  {
    MotionParameters params(tmax_,
                            theta0, theta1,
                            x0, x1,
                            y0, y1,
                            sx0, sx1,
                            sy0, sy1);

    objects_.emplace_back(std::make_shared<Object>(path_to_texture,
                                                   cv::Size(width_, height_),
                                                   params,
                                                   median_blur,
                                                   gaussian_blur));
  }


  if(FLAGS_path_to_output_folder != "")
  {
    outputGroundTruthData();
  }
}

MultiObject2DRenderer::~MultiObject2DRenderer()
{
}

bool MultiObject2DRenderer::render(const Time t,
                                   const ImagePtr& out_image,
                                   const OpticFlowPtr& optic_flow_map) const {

  if(ze::nanosecToSecTrunc(t) > tmax_)
  {
    return true;
  }

  CHECK_EQ(out_image->rows, height_);
  CHECK_EQ(out_image->cols, width_);

  out_image->setTo(0);
  optic_flow_map->setTo(0.);

  for(int i=0; i<objects_.size(); ++i)
  {
    const bool is_first_layer = (i == 0);
    objects_[i]->draw(t, is_first_layer);
  }

  // composite the local images drawn by each object
  // start from bottom image, merge it with the upper one
  // then go one level up, merge the resulting image with the upper one
  // and so on...
  ImageFloatType intensity;
  ImageFloatType alpha;

  for(int i=0; i<objects_.size(); ++i)
  {
    const std::shared_ptr<Object>& object = objects_[i];
    const cv::Mat& image = object->canvas_;
    const OpticFlow& flow = object->flow_;

    for(int y=0; y<out_image->rows; ++y)
    {
      for(int x=0; x<out_image->cols; ++x)
      {
        getIntensityAndAlpha(image, x, y, &intensity, &alpha);
        (*out_image)(y,x) = alpha * intensity + (1.-alpha) * (*out_image)(y,x);

        if(alpha > 0)
        {
          (*optic_flow_map)(y,x) = flow(y,x);
        }
      }
    }
  }

  return false;
}

void MultiObject2DRenderer::outputGroundTruthData()
{
  // This function generates some ground truth information and output if to the FLAGS_path_to_output_folder
  // In principle, this information could be transmitted in a SimulationData structure and forwarded
  // to a Publisher object that writes it to disk.
  // However, for technical reasons, it is more convenient to write the displacement maps here, rather than integrate instantaneous
  // optic flow maps in a Publisher.

  // The following ground truth information is computed below and output to the ground truth folder:
  //   - Displacement maps from t=0 to t=tmax (and vice-versa)
  //   - Images 0 and images 1

  VLOG(1) << "Will output ground truth data to folder: " << FLAGS_path_to_output_folder;

  // Notation: the frame at time t=0 is denoted as frame 0, and the frame at time t=tmax is denoted as frame 1
  // Compute the entire displacement field from 0 to 1, for every layer
  const ze::real_t t0 = 0.0;
  const ze::real_t t1 = tmax_;

  // Every entry in this vector is a displacement map that maps pixel locations in image 0
  // to the corresponding pixel location in image 1, for the layer i
  //
  // this individual displacement layers are then merged together to form the final
  // displacement map later
  //
  // Note: the reverse layer-wise displacement map can be computed as follows:
  //   layer_displacement_01[i] = -layer_displacement_10[i]
  std::vector<OpticFlow> layer_displacements_10(objects_.size());
  for(int i=0; i<objects_.size(); ++i)
  {
    layer_displacements_10[i] = OpticFlow(height_, width_);
    const Object object = *(objects_[i]);

    Affine A_t0_src = object.getMotionParameters().getAffineTransformationWithJacobian(t0).first;
    Affine A_t1_src = object.getMotionParameters().getAffineTransformationWithJacobian(t1).first;
    Affine A_t1_t0 = A_t1_src * A_t0_src.inv();

    // M_t1_t0 maps any point on the first image to its position in the last image
    Affine M_t1_t0 = object.getK1() * A_t1_t0 * object.getK1().inv();

    for(int y=0; y<height_; ++y)
    {
      for(int x=0; x<width_; ++x)
      {
        FloatType x_t1 = M_t1_t0(0,0) * x + M_t1_t0(0,1) * y + M_t1_t0(0,2);
        FloatType y_t1 = M_t1_t0(1,0) * x + M_t1_t0(1,1) * y + M_t1_t0(1,2);

        FloatType displacement_x, displacement_y;
        // if the pixel went out of the field of view, we assign a displacement of 0 (convention)
        displacement_x = (x_t1 >= 0 && x_t1 < width_) ? (ImageFloatType) x_t1 - (ImageFloatType) x : 0.;
        displacement_y = (y_t1 >= 0 && y_t1 < height_) ? (ImageFloatType) y_t1 - (ImageFloatType) y : 0.;

        layer_displacements_10[i](y,x)[0] = displacement_x;
        layer_displacements_10[i](y,x)[1] = displacement_y;
      }
    }
  }


  ImageFloatType intensity, alpha;

  // First, merge the displacement map from 0 to 1
  OpticFlow displacement_map_10(height_, width_); // displacement map from 0 to 1
  Image image0(height_, width_);
  image0.setTo(0);
  for(int i=0; i<objects_.size(); ++i)
  {
    const bool is_first_layer = (i == 0);
    objects_[i]->draw(t0, is_first_layer);
  }

  for(int i=0; i<objects_.size(); ++i)
  {
    const std::shared_ptr<Object>& object = objects_[i];
    for(int y=0; y<height_; ++y)
    {
      for(int x=0; x<width_; ++x)
      {
        getIntensityAndAlpha(object->canvas_, x, y, &intensity, &alpha);
        image0(y,x) = alpha * intensity + (1.-alpha) * image0(y,x);
        if(alpha > 0)
        {
          displacement_map_10(y,x) = layer_displacements_10[i](y,x);
        }
      }
    }
  }

  // Second, create displacement map from 1 to 0
  OpticFlow displacement_map_01(height_, width_); // displacement map from 1 to 0
  Image image1(height_, width_);
  image1.setTo(0);
  for(int i=0; i<objects_.size(); ++i)
  {
    const bool is_first_layer = (i == 0);
    objects_[i]->draw(ze::secToNanosec(t1), is_first_layer);
  }

  for(int i=0; i<objects_.size(); ++i)
  {
    for(int y=0; y<height_; ++y)
    {
      for(int x=0; x<width_; ++x)
      {
        getIntensityAndAlpha(objects_[i]->canvas_, x, y, &intensity, &alpha);
        image1(y,x) = alpha * intensity + (1.-alpha) * image1(y,x);
        if(alpha > 0)
        {
          displacement_map_01(y,x) = -layer_displacements_10[i](y,x);
        }
      }
    }
  }

  std::ofstream displacement_file_10, displacement_file_01;
  ze::openOutputFileStream(ze::joinPath(FLAGS_path_to_output_folder, "displacement_10.txt"), &displacement_file_10);

  if(FLAGS_output_reverse_displacement_map)
  {
    ze::openOutputFileStream(ze::joinPath(FLAGS_path_to_output_folder, "displacement_01.txt"), &displacement_file_01);
  }

  for(int y=0; y<height_; ++y)
  {
    for(int x=0; x<width_; ++x)
    {
      displacement_file_10 << displacement_map_10(y,x)[0] << " " << displacement_map_10(y,x)[1] << std::endl;

      if(FLAGS_output_reverse_displacement_map)
      {
        displacement_file_01 << displacement_map_01(y,x)[0] << " " << displacement_map_01(y,x)[1] << std::endl;
      }
    }
  }

  displacement_file_10.close();
  if(FLAGS_output_reverse_displacement_map)
  {
    displacement_file_01.close();
  }

  cv::Mat disp_image0, disp_image1;
  image0.convertTo(disp_image0, CV_8U, 255);
  image1.convertTo(disp_image1, CV_8U, 255);
  cv::imwrite(ze::joinPath(FLAGS_path_to_output_folder, "image0.png"), disp_image0);
  cv::imwrite(ze::joinPath(FLAGS_path_to_output_folder, "image1.png"), disp_image1);
}

} // namespace event_camera_simulator
