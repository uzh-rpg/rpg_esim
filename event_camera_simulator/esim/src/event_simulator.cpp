#include <esim/esim/event_simulator.hpp>
#include <ze/common/random.hpp>
#include <glog/logging.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <ze/common/time_conversions.hpp>

namespace event_camera_simulator {

void EventSimulator::init(const Image &img, Time time)
{
  VLOG(1) << "Initialized event camera simulator with sensor size: " << img.size();
  VLOG(1) << "and contrast thresholds: C+ = " << config_.Cp << " , C- = " << config_.Cm;
  is_initialized_ = true;
  last_img_ = img.clone();
  ref_values_ = img.clone();
  last_event_timestamp_ = TimestampImage::zeros(img.size());
  current_time_ = time;
  size_ = img.size();
}

Events EventSimulator::imageCallback(const Image& img, Time time)
{
  CHECK_GE(time, 0);
  Image preprocessed_img = img.clone();
  if(config_.use_log_image)
  {
    LOG_FIRST_N(INFO, 1) << "Converting the image to log image with eps = " << config_.log_eps << ".";
    cv::log(config_.log_eps + img, preprocessed_img);
  }

  if(!is_initialized_)
  {
    init(preprocessed_img, time);
    return {};
  }

  // For each pixel, check if new events need to be generated since the last image sample
  static constexpr ImageFloatType tolerance = 1e-6;
  Events events;
  Duration delta_t_ns = time - current_time_;

  CHECK_GT(delta_t_ns, 0u);
  CHECK_EQ(img.size(), size_);

  for (int y = 0; y < size_.height; ++y)
  {
    for (int x = 0; x < size_.width; ++x)
    {
      ImageFloatType itdt = preprocessed_img(y, x);
      ImageFloatType it = last_img_(y, x);
      ImageFloatType prev_cross = ref_values_(y, x);

      if (std::fabs (it - itdt) > tolerance)
      {
        ImageFloatType pol = (itdt >= it) ? +1.0 : -1.0;
        ImageFloatType C = (pol > 0) ? config_.Cp : config_.Cm;
        ImageFloatType sigma_C = (pol > 0) ? config_.sigma_Cp : config_.sigma_Cm;
        if(sigma_C > 0)
        {
          C += ze::sampleNormalDistribution<ImageFloatType>(false, 0, sigma_C);
          constexpr ImageFloatType minimum_contrast_threshold = 0.01;
          C = std::max(minimum_contrast_threshold, C);
        }
        ImageFloatType curr_cross = prev_cross;
        bool all_crossings = false;

        do
        {
          curr_cross += pol * C;

          if ((pol > 0 && curr_cross > it && curr_cross <= itdt)
              || (pol < 0 && curr_cross < it && curr_cross >= itdt))
          {
            Duration edt = (curr_cross - it) * delta_t_ns / (itdt - it);
            Time t = current_time_ + edt;

            // check that pixel (x,y) is not currently in a "refractory" state
            // i.e. |t-that last_timestamp(x,y)| >= refractory_period
            const Time last_stamp_at_xy = ze::secToNanosec(last_event_timestamp_(y,x));
            CHECK_GE(t, last_stamp_at_xy);
            const Duration dt = t - last_stamp_at_xy;
            if(last_event_timestamp_(y,x) == 0 || dt >= config_.refractory_period_ns)
            {
              events.push_back(Event(x, y, t, pol > 0));
              last_event_timestamp_(y,x) = ze::nanosecToSecTrunc(t);
            }
            else
            {
              VLOG(3) << "Dropping event because time since last event  ("
                      << dt << " ns) < refractory period ("
                      << config_.refractory_period_ns << " ns).";
            }
            ref_values_(y,x) = curr_cross;
          }
          else
          {
            all_crossings = true;
          }
        } while (!all_crossings);
      } // end tolerance
    } // end for each pixel
  }

  // update simvars for next loop
  current_time_ = time;
  last_img_ = preprocessed_img.clone(); // it is now the latest image

  // Sort the events by increasing timestamps, since this is what
  // most event processing algorithms expect
  sort(events.begin(), events.end(),
       [](const Event& a, const Event& b) -> bool
  {
    return a.t < b.t;
  });

  return events;
}

} // namespace event_camera_simulator
