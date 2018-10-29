#pragma once

#include <boost/asio.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <string>
#include <functional>

#include <iostream>
#include <istream>
#include <ostream>

#include <chrono>
#include <thread>

#include <opencv2/core/core.hpp>

namespace event_camera_simulator {

using boost::asio::ip::tcp;

struct CameraData {
  uint32_t id;
  double_t pitch;
  double_t yaw;
  double_t roll;
  double_t x;
  double_t y;
  double_t z;
};


class UnrealCvClient {
public:
  UnrealCvClient(std::string host, std::string port);
  ~UnrealCvClient();

  void saveImage(uint32_t camid, std::string path);
  cv::Mat getImage(uint32_t camid);
  cv::Mat getDepth(uint32_t camid);
  void setCamera(const CameraData & camera);
  void setCameraFOV(float hfov_deg);
  void setCameraSize(int width, int height);

protected:

  void sendCommand(std::string command);

  template<typename Result>
  Result sendCommand(std::string command, std::function<Result(std::istream&, uint32_t)>  mapf);

  void send(std::string msg, uint32_t counter);

  template<typename Result>
  Result receive(std::function<Result(std::istream&, uint32_t)>  parser);

  //must stand before socket_ because of c++ initialization order
  boost::asio::io_service io_service_;
  tcp::socket socket_;
  mutable uint32_t counter_;
  uint32_t delay_;
  boost::format angular_format_;

private:
  void sleep(uint32_t delay);
  void handleError(boost::system::error_code ec);
  std::string istreamToString(std::istream& stream, uint32_t size);

  void parse_npy_header(unsigned char* buffer,
                        size_t& word_size,
                        std::vector<size_t>& shape,
                        bool& fortran_order);

};

} // namespace event_camera_simulator
