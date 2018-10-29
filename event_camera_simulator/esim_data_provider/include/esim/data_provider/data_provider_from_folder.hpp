#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ze/common/macros.hpp>
#include <ze/common/types.hpp>
#include <esim/data_provider/data_provider_base.hpp>
#include <ze/cameras/camera_rig.hpp>
#include <fstream>

namespace event_camera_simulator {

class DataProviderFromFolder : public DataProviderBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DataProviderFromFolder(const std::string& path_to_data_folder);

  virtual ~DataProviderFromFolder() = default;

  virtual bool spinOnce() override;

  virtual bool ok() const override;

  size_t numCameras() const override;

private:

  int64_t getTimeStamp(const std::string& ts_str) const;

  std::string path_to_data_folder_;
  std::ifstream images_in_str_;
  const char delimiter_{','};
  const size_t num_tokens_in_line_ = 3; // stamp, image, depth

  SimulatorData sim_data_;
};

} // namespace event_camera_simulator
