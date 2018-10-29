#include <esim/unrealcv_bridge/unrealcv_bridge.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <glog/logging.h>

#include <cmath>
#include <regex>

namespace event_camera_simulator {

using boost::asio::ip::tcp;

// from: https://www.boost.org/doc/libs/1_47_0/doc/html/boost_asio/reference/connect/overload6.html
struct unrealcv_server_connect_condition
{
  template <typename Iterator>
  Iterator operator()(
      const boost::system::error_code& ec,
      Iterator next)
  {
    if(ec)
    {
      LOG(ERROR) << ec.message();
    }
    LOG(INFO) << "Trying: " << next->endpoint();
    return next;
  }
};

UnrealCvClient::UnrealCvClient(std::string host, std::string port)
  : io_service_(),
    socket_(io_service_),
    counter_(0),
    delay_(30){

  tcp::resolver resolver(io_service_);
  tcp::resolver::query query(host, port);
  tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

  boost::system::error_code ec;
  boost::asio::connect(socket_, endpoint_iterator, unrealcv_server_connect_condition(), ec);
  if(ec)
  {
    LOG(FATAL) << "Could not connect to UnrealCV server";
    return;
  }
  sleep(500); // long sleep to initiate

  // receive the first "we are connected" string
  receive<std::string>([this] (std::istream& stream, uint32_t size) -> std::string {
    return this->istreamToString(stream, size);
  });

  sleep(delay_);
  sendCommand("vrun r.AmbientOcclusionLevels 0");
  sendCommand("vrun r.LensFlareQuality 0");
  sendCommand("vrun r.DefaultFeature.AntiAliasing 2");
  sendCommand("vrun r.DefaultFeature.MotionBlur 0");
  sendCommand("vrun r.PostProcessAAQuality 6");
}

UnrealCvClient::~UnrealCvClient() {
  socket_.close();
}

void UnrealCvClient::saveImage(uint32_t camid, std::string path)
{
  std::string req = (boost::format("vget /camera/%i/lit %s") % camid % path).str();
  sendCommand(req);
}

cv::Mat UnrealCvClient::getImage(uint32_t camid)
{
  std::string req = (boost::format("vget /camera/%i/lit png") % camid).str();

  return sendCommand<cv::Mat>(req, [](std::istream& stream, uint32_t size){
    std::vector<char> data(size);
    stream.read(data.data(), size);
    cv::Mat matrixPng = cv::imdecode(cv::Mat(data), 1);
    return matrixPng.clone();
  });
}

cv::Mat UnrealCvClient::getDepth(uint32_t camid)
{
  std::string req = (boost::format("vget /camera/%i/depth npy") % camid).str();

  return sendCommand<cv::Mat>(req, [this](std::istream& stream, uint32_t size){

    std::vector<char> data(size);
    stream.read(data.data(), size);
    unsigned char* buffer = (unsigned char *)data.data();

    /*
     * Gather data from the header
     */
    std::vector<size_t> img_dims; //if appending, the shape of existing + new data
    size_t word_size;
    bool fortran_order;
    parse_npy_header(buffer, word_size, img_dims, fortran_order);

    // https://docs.scipy.org/doc/numpy/neps/npy-format.html
    std::string npy_magic(reinterpret_cast<char*>(buffer),6);
    uint8_t major_version = *reinterpret_cast<uint8_t*>(buffer+6);
    uint8_t minor_version = *reinterpret_cast<uint8_t*>(buffer+7);
    uint16_t header_str_len = *reinterpret_cast<uint16_t*>(buffer+8);
    std::string header(reinterpret_cast<char*>(buffer+9),header_str_len);

    uint16_t header_total_len = 10 + header_str_len;
    uint32_t data_length = data.size() - header_total_len;
    uint32_t num_pixel = img_dims.at(0) * img_dims.at(1);

    /*
     * Ensure that the data is okay
     */
    if(!(major_version == 1 &&
         minor_version == 0 &&
         npy_magic.find("NUMPY") != std::string::npos)){
      throw std::runtime_error("Npy header data not supported");
    }

    if(!(data_length == (num_pixel * sizeof(float_t)))) {
      throw std::runtime_error("Npy array data shape does not match the image size");
    }

    /*
     * Read and convert the data
     */
    cv::Mat matrix_f32 = cv::Mat(img_dims.at(0), img_dims.at(1),
                                 CV_32F, buffer + header_total_len).clone();

    return matrix_f32;

  });
}


void UnrealCvClient::setCamera(const CameraData & camera)
{
  std::string cam_pose_s = (boost::format("vset /camera/%i/pose %.5f %.5f %.5f %.5f %.5f %.5f") %
                            camera.id %
                            camera.x %
                            camera.y %
                            camera.z %
                            camera.pitch %
                            camera.yaw %
                            camera.roll).str();

  sendCommand(cam_pose_s);
}

void UnrealCvClient::setCameraSize(int width, int height)
{
  VLOG(1) << "Setting the camera size to: " << width << "x" << height;
  std::string req_size = (boost::format("vrun r.setres %dx%d") %
                         width %
                         height).str();
  sendCommand(req_size);
}

void UnrealCvClient::setCameraFOV(float hfov_deg)
{
  VLOG(1) << "Setting the camera horizontal field of view to: " << hfov_deg << " deg";
  const int cam_id = 0;
  std::string req_fov = (boost::format("vset /camera/%i/horizontal_fieldofview %.5f") %
                         cam_id %
                         hfov_deg).str();
  sendCommand(req_fov);
}

void UnrealCvClient::sendCommand(std::string command)
{
  if (!(boost::starts_with(command, "vset") || boost::starts_with(command, "vrun"))) {
    throw std::runtime_error(
          "invalid command: command must start with vget or (vset, vrun)");
  }

  uint32_t tmpc = counter_++;
  VLOG(1) << "SET:" << tmpc << " " << command;
  send(command, tmpc);
  sleep(delay_);

  std::string result_prefix = std::to_string(tmpc) + ":";

  /*
   * is set command: we never expect something else than "ok",
   * we do not use mapf
   */

  std::string result = receive<std::string>(
        [this] (std::istream& stream, uint32_t size) -> std::string {
      return this->istreamToString(stream, size);
});

  if (!boost::starts_with(result, result_prefix + "ok")) {
    throw std::runtime_error("response identifier is incorrect");
  } else {
    VLOG(1) << "GET:" << tmpc << " " << "ok";
  }

  sleep(delay_);

}

template<typename Result>
Result UnrealCvClient::sendCommand(std::string command, std::function<Result(std::istream&, uint32_t)>  mapf)
{
  if (!(boost::starts_with(command, "vget")))
  {
    throw std::runtime_error(
          "invalid command: command must start with vget or (vset, vrun)");
  }

  uint32_t tmpc = counter_++;
  VLOG(1) << "SET:" << tmpc << " " << command;
  send(command, tmpc);
  sleep(delay_);

  std::string result_prefix = std::to_string(tmpc) + ":";

  /*
   * is get command: we expect a response that can
   * be a string or binary data
   */

  Result result = receive<Result>(
        [this, result_prefix, mapf] (std::istream& stream, uint32_t size) -> Result {

    std::string prefix = istreamToString(stream, result_prefix.size());
    size-=result_prefix.size();

    if(!boost::starts_with(prefix, result_prefix)) {
      throw std::runtime_error("response identifier is incorrect");
    }

    return mapf(stream, size);
  });

  sleep(delay_);
  return result;
}

void UnrealCvClient::send(std::string msg, uint32_t counter)
{
  std::string out = std::to_string(counter) + ":" + msg;

  uint32_t magic = 0x9E2B83C1;
  uint32_t size =  out.length();

  boost::asio::streambuf request;
  std::ostream request_stream(&request);
  boost::system::error_code ec;

  request_stream.write((char*) &magic, sizeof(magic));
  request_stream.write((char*) &size, sizeof(size));

  request_stream << out;
  // Send the request.
  boost::asio::write(socket_,
                     request,
                     boost::asio::transfer_exactly(request.size() + sizeof(magic) + sizeof(size)),
                     ec);
}

template<typename Result>
Result UnrealCvClient::receive(std::function<Result(std::istream&, uint32_t)>  parser)
{

  boost::system::error_code ec;
  boost::asio::streambuf response;

  //first read the 8 byte header
  boost::asio::read(socket_, response, boost::asio::transfer_exactly(8), ec);
  handleError(ec);

  uint32_t magic;
  uint32_t size;

  // Check that response is OK.
  std::istream response_stream(&response);
  response_stream.read((char*)&magic, sizeof(magic));
  response_stream.read((char*)&size, sizeof(size));


  boost::asio::read(socket_, response, boost::asio::transfer_exactly(size), ec);
  handleError(ec);

  Result res = parser(response_stream, size);
  return res;
}

void UnrealCvClient::handleError(boost::system::error_code ec)
{
  if (ec == boost::asio::error::eof) {
    throw boost::system::system_error(ec); // Some other error.
  } else if (ec) {
    throw boost::system::system_error(ec); // Some other error.
  }
}

void UnrealCvClient::sleep(uint32_t delay) {
  std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}


std::string UnrealCvClient::istreamToString(
    std::istream& stream, uint32_t size)
{

  char buffer[size];
  stream.read(buffer, size);

  std::stringstream out;
  out << buffer;
  std::string result = out.str();
  return result;
}

// from cnpy: https://github.com/rogersce/cnpy/blob/master/cnpy.cpp
void UnrealCvClient::parse_npy_header(unsigned char* buffer,
                                      size_t& word_size,
                                      std::vector<size_t>& shape,
                                      bool& fortran_order)
{

  //std::string magic_string(buffer,6);
  uint8_t major_version = *reinterpret_cast<uint8_t*>(buffer+6);
  uint8_t minor_version = *reinterpret_cast<uint8_t*>(buffer+7);
  uint16_t header_len = *reinterpret_cast<uint16_t*>(buffer+8);
  std::string header(reinterpret_cast<char*>(buffer+9),header_len);

  size_t loc1, loc2;

  //fortran order
  loc1 = header.find("fortran_order")+16;
  fortran_order = (header.substr(loc1,4) == "True" ? true : false);

  //shape
  loc1 = header.find("(");
  loc2 = header.find(")");

  std::regex num_regex("[0-9][0-9]*");
  std::smatch sm;
  shape.clear();

  std::string str_shape = header.substr(loc1+1,loc2-loc1-1);
  while(std::regex_search(str_shape, sm, num_regex))
  {
    shape.push_back(std::stoi(sm[0].str()));
    str_shape = sm.suffix().str();
  }

  //endian, word size, data type
  //byte order code | stands for not applicable.
  //not sure when this applies except for byte array
  loc1 = header.find("descr")+9;
  bool littleEndian = (header[loc1] == '<' || header[loc1] == '|' ? true : false);
  assert(littleEndian);

  //char type = header[loc1+1];
  //assert(type == map_type(T));

  std::string str_ws = header.substr(loc1+2);
  loc2 = str_ws.find("'");
  word_size = atoi(str_ws.substr(0,loc2).c_str());
}


} // namespace event_camera_simulator
