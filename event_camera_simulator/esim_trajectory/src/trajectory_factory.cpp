#include <esim/trajectory/trajectory_factory.hpp>
#include <ze/common/csv_trajectory.hpp>
#include <ze/common/logging.hpp>

DEFINE_int32(trajectory_type, 0, " 0: Random spline trajectory, 1: Load trajectory from CSV file");

DEFINE_double(trajectory_length_s, 100.0,
              "Length of the trajectory, in seconds"
              "(used when the trajectory type is random spline)");

DEFINE_int32(trajectory_spline_order, 5,
             "Spline order of the trajectory");

DEFINE_double(trajectory_sampling_frequency_hz, 5,
              "Sampling frequency of the spline trajectory, i.e."
              "number of random poses generated per second along the trajectory");

DEFINE_int32(trajectory_num_spline_segments, 100,
             "Number of spline segments used for the trajectory");

DEFINE_double(trajectory_lambda, 0.1,
              "Smoothing factor for the spline trajectories."
              "Low value = less smooth, high value = more smooth");

DEFINE_double(trajectory_multiplier_x, 1.0,
              "Scaling factor for the X camera axis");

DEFINE_double(trajectory_multiplier_y, 1.0,
              "Scaling factor for the y camera axis");

DEFINE_double(trajectory_multiplier_z, 1.0,
              "Scaling factor for the z camera axis");

DEFINE_double(trajectory_multiplier_wx, 1.0,
              "Scaling factor for the x orientation axis");

DEFINE_double(trajectory_multiplier_wy, 1.0,
              "Scaling factor for the y orientation axis");

DEFINE_double(trajectory_multiplier_wz, 1.0,
              "Scaling factor for the z orientation axis");

DEFINE_double(trajectory_offset_x, 0.0,
              "Offset for the X camera axis");

DEFINE_double(trajectory_offset_y, 0.0,
              "Offset for the y camera axis");

DEFINE_double(trajectory_offset_z, 0.0,
              "Offset for the z camera axis");

DEFINE_double(trajectory_offset_wx, 0.0,
              "Offset for the x orientation axis");

DEFINE_double(trajectory_offset_wy, 0.0,
              "Offset for the y orientation axis");

DEFINE_double(trajectory_offset_wz, 0.0,
              "Offset for the z orientation axis");

DEFINE_string(trajectory_csv_file, "",
              "CSV file containing the trajectory");

DEFINE_int32(trajectory_dynamic_objects_type, 1, " 0: Random spline trajectory, 1: Load trajectory from CSV file");

DEFINE_int32(trajectory_dynamic_objects_spline_order, 5,
             "Spline order of the trajectory");

DEFINE_int32(trajectory_dynamic_objects_num_spline_segments, 100,
             "Number of spline segments used for the trajectory");

DEFINE_double(trajectory_dynamic_objects_lambda, 0.1,
              "Smoothing factor for the spline trajectories."
              "Low value = less smooth, high value = more smooth");

DEFINE_string(trajectory_dynamic_objects_csv_dir, "",
              "Directory containing CSV file of trajectory for dynamic objects");

DEFINE_string(trajectory_dynamic_objects_csv_file, "",
              "CSV file containing the trajectory");

namespace event_camera_simulator {

std::tuple<ze::TrajectorySimulator::Ptr, std::vector<ze::TrajectorySimulator::Ptr>> loadTrajectorySimulatorFromGflags()
{
  ze::TrajectorySimulator::Ptr trajectory;

  std::vector<ze::TrajectorySimulator::Ptr> trajectories_dynamic_objects;
  bool dynamic_object = false;

  size_t p_start, p_end;
  p_start = 0;
  while(1)
  {
    int trajectory_type;
    if (dynamic_object)
    {
        trajectory_type = FLAGS_trajectory_dynamic_objects_type;
        if (trajectory_type != 1)
        {
            LOG(FATAL) << "Only supporting trajectories of type 1 for dynamic objects!";
        }
    }
    else
    {
        trajectory_type = FLAGS_trajectory_type;
    }

    // set path for dynamics objects
    std::string csv_path;
    bool should_break = false;
    
    if (dynamic_object)
    {
        if ((p_end = FLAGS_trajectory_dynamic_objects_csv_file.find(";",p_start)) != std::string::npos)
        {
            csv_path = FLAGS_trajectory_dynamic_objects_csv_dir + "/" + FLAGS_trajectory_dynamic_objects_csv_file.substr(p_start, p_end - p_start);

            p_start = p_end + 1;
        }
        else
        {
            csv_path = FLAGS_trajectory_dynamic_objects_csv_dir + "/" + FLAGS_trajectory_dynamic_objects_csv_file.substr(p_start, p_end - p_start);
            should_break = true;
        }
    }
    else
    {
        csv_path = FLAGS_trajectory_csv_file;
    }

    switch (trajectory_type)
    {
        case 0: // Random spline
        {
            std::shared_ptr<ze::BSplinePoseMinimalRotationVector> pbs =
                std::make_shared<ze::BSplinePoseMinimalRotationVector>(FLAGS_trajectory_spline_order);
            ze::MatrixX poses;
            ze::VectorX times = ze::VectorX::LinSpaced(FLAGS_trajectory_sampling_frequency_hz * FLAGS_trajectory_length_s,
                                                    0,
                                                    FLAGS_trajectory_length_s);
            poses.resize(6, times.size());
            poses.setRandom();
            poses.row(0) *= FLAGS_trajectory_multiplier_x;
            poses.row(1) *= FLAGS_trajectory_multiplier_y;
            poses.row(2) *= FLAGS_trajectory_multiplier_z;
            poses.row(3) *= FLAGS_trajectory_multiplier_wx;
            poses.row(4) *= FLAGS_trajectory_multiplier_wy;
            poses.row(5) *= FLAGS_trajectory_multiplier_wz;
            poses.row(0).array() += FLAGS_trajectory_offset_x;
            poses.row(1).array() += FLAGS_trajectory_offset_y;
            poses.row(2).array() += FLAGS_trajectory_offset_z;
            poses.row(3).array() += FLAGS_trajectory_offset_wx;
            poses.row(4).array() += FLAGS_trajectory_offset_wy;
            poses.row(5).array() += FLAGS_trajectory_offset_wz;
            pbs->initPoseSpline3(times, poses, FLAGS_trajectory_num_spline_segments, FLAGS_trajectory_lambda);
            trajectory.reset(new ze::SplineTrajectorySimulator(pbs));
            break;
        }
        case 1: // Load from CSV file
        {
            // Create trajectory:
            ze::PoseSeries pose_series;

            LOG(INFO) << "Reading trajectory from CSV file: " << csv_path;
            pose_series.load(csv_path);
            
            ze::StampedTransformationVector poses = pose_series.getStampedTransformationVector();

            // Set start time of trajectory to zero.
            const int64_t offset = poses.at(0).first;
            for (ze::StampedTransformation& it : poses)
            {
                it.first -= offset;
            }

            LOG(INFO) << "Initializing spline from trajectory (this may take some sime)...";

            int spline_order, num_spline_segments;
            double lambda;
            if (dynamic_object)
            {
                spline_order = FLAGS_trajectory_dynamic_objects_spline_order;
                num_spline_segments = FLAGS_trajectory_dynamic_objects_num_spline_segments;
                lambda = FLAGS_trajectory_dynamic_objects_lambda;
            }
            else
            {
                spline_order = FLAGS_trajectory_spline_order;
                num_spline_segments = FLAGS_trajectory_num_spline_segments;
                lambda = FLAGS_trajectory_lambda;
            }

            std::shared_ptr<ze::BSplinePoseMinimalRotationVector> bs =
                std::make_shared<ze::BSplinePoseMinimalRotationVector>(spline_order);
            bs->initPoseSplinePoses(poses, num_spline_segments, lambda);
            if (dynamic_object)
            {
                trajectories_dynamic_objects.push_back(std::make_shared<ze::SplineTrajectorySimulator>(bs));
            }
            else
            {
                trajectory = std::make_shared<ze::SplineTrajectorySimulator>(bs);
            }
            LOG(INFO) << "Done!";
            break;
        }
        case 2: // Sinusoidal spline
        {
            std::shared_ptr<ze::BSplinePoseMinimalRotationVector> pbs =
                std::make_shared<ze::BSplinePoseMinimalRotationVector>(FLAGS_trajectory_spline_order);
            ze::MatrixX poses;
            ze::VectorX times = ze::VectorX::LinSpaced(FLAGS_trajectory_sampling_frequency_hz * FLAGS_trajectory_length_s,
                                                    0,
                                                    FLAGS_trajectory_length_s);
            poses.resize(6, times.size());
            for (int i = 0; i < 6; i++){
              poses.row(i) = (M_PI*times).array().sin();
            }
            poses.row(0) *= FLAGS_trajectory_multiplier_x;
            poses.row(1) *= FLAGS_trajectory_multiplier_y;
            poses.row(2) *= FLAGS_trajectory_multiplier_z;
            poses.row(3) *= FLAGS_trajectory_multiplier_wx;
            poses.row(4) *= FLAGS_trajectory_multiplier_wy;
            poses.row(5) *= FLAGS_trajectory_multiplier_wz;
            poses.row(0).array() += FLAGS_trajectory_offset_x;
            poses.row(1).array() += FLAGS_trajectory_offset_y;
            poses.row(2).array() += FLAGS_trajectory_offset_z;
            poses.row(3).array() += FLAGS_trajectory_offset_wx;
            poses.row(4).array() += FLAGS_trajectory_offset_wy;
            poses.row(5).array() += FLAGS_trajectory_offset_wz;
            pbs->initPoseSpline3(times, poses, FLAGS_trajectory_num_spline_segments, FLAGS_trajectory_lambda);
            trajectory.reset(new ze::SplineTrajectorySimulator(pbs));
            break;
        }
        default:
        {
            LOG(FATAL) << "Trajectory type is not known.";
            break;
        }
    }

    if (!dynamic_object)
    { 
        if (!FLAGS_trajectory_dynamic_objects_csv_dir.empty() && !FLAGS_trajectory_dynamic_objects_csv_file.empty())
        {
            dynamic_object = true;
        }
        else
        {
            break;
        }
    }
    
    if (should_break)
    {
        break;
    }
  }

  return std::make_tuple(trajectory, trajectories_dynamic_objects);
}

} // namespace event_camera_simulator
