#pragma once

#include <atomic>
#include <functional>
#include <memory>

#include <esim/common/types.hpp>
#include <ze/common/macros.hpp>
#include <ze/common/noncopyable.hpp>
#include <ze/common/signal_handler.hpp>

// fwd
namespace cv {
class Mat;
}

namespace event_camera_simulator {

using Callback =
  std::function<void (const SimulatorData& sim_data)>;

enum class DataProviderType {
  RendererOnline,
  Folder,
  Rosbag
};

//! A data provider registers to a data source and triggers callbacks when
//! new data is available.
class DataProviderBase : ze::Noncopyable
{
public:
  ZE_POINTER_TYPEDEFS(DataProviderBase);

  DataProviderBase() = delete;
  DataProviderBase(DataProviderType type);
  virtual ~DataProviderBase() = default;

  //! Process all callbacks. Waits until callback is processed.
  void spin();

  //! Read next data field and process callback. Returns false when datatset finished.
  virtual bool spinOnce() = 0;

  //! False if there is no more data to process or there was a shutdown signal.
  virtual bool ok() const = 0;

  //! Pause data provider.
  virtual void pause();

  //! Stop data provider.
  virtual void shutdown();

  //! Register callback function to call when new message is available.
  void registerCallback(const Callback& callback);

  //! Returns the number of cameras in the rig
  virtual size_t numCameras() const = 0;

  //! Returns the camera rig
  ze::CameraRig::Ptr getCameraRig() { return camera_rig_; }

protected:
  DataProviderType type_;
  Callback callback_;
  volatile bool running_ = true;

  ze::CameraRig::Ptr camera_rig_;

private:
  ze::SimpleSigtermHandler signal_handler_; //!< Sets running_ to false when Ctrl-C is pressed.
};

} // namespace event_camera_simulator
