// This file was adapted from the ze_oss project: https://github.com/zurich-eye/ze_oss/blob/master/ze_data_provider/include/ze/data_provider/data_provider_rosbag.hpp
// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved

#pragma once

#include <map>
#include <string>
#include <memory>
#include <vector>

#include<gflags/gflags.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Image.h>

#include <esim/data_provider/data_provider_base.hpp>

namespace event_camera_simulator {

class DataProviderRosbag : public DataProviderBase
{
public:
  DataProviderRosbag(
      const std::string& bag_filename,
      const std::map<std::string, size_t>& camera_topics);

  virtual ~DataProviderRosbag() = default;

  virtual bool spinOnce() override;

  virtual bool ok() const override;

  virtual size_t numCameras() const;

  size_t size() const;

private:
  void loadRosbag(const std::string& bag_filename);
  void initBagView(const std::vector<std::string>& topics);

  inline bool cameraSpin(const sensor_msgs::ImageConstPtr m_img,
                         const rosbag::MessageInstance& m);

  std::unique_ptr<rosbag::Bag> bag_;
  std::unique_ptr<rosbag::View> bag_view_;
  rosbag::View::iterator bag_view_it_;
  int n_processed_images_ = 0;

  // subscribed topics:
  std::map<std::string, size_t> img_topic_camidx_map_; // camera_topic --> camera_id

  SimulatorData sim_data_;
};

} // namespace event_camera_simulator
