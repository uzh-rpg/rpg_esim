#include <esim/data_provider/data_provider_base.hpp>

namespace event_camera_simulator {

DataProviderBase::DataProviderBase(DataProviderType type)
  : type_(type)
  , signal_handler_(running_)
{}

void DataProviderBase::spin()
{
  while (ok())
  {
    spinOnce();
  }
}

void DataProviderBase::pause()
{
  running_ = false;
}

void DataProviderBase::shutdown()
{
  running_ = false;
}

void DataProviderBase::registerCallback(const Callback& callback)
{
  callback_ = callback;
}

} // namespace event_camera_simulator
