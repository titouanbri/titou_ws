
#include <rokubimini_serial/RokubiminiSerialBusManager.hpp>

namespace rokubimini
{
namespace serial
{
double RokubiminiSerialBusManager::loadTimeStep()
{
  return timeStep_;
}
void RokubiminiSerialBusManager::fetchTimeStep()
{
  if (!nh_->getParam("time_step", timeStep_))
  {
    ROS_INFO("[%s] Could not find the 'time_step' parameter in Parameter Server. Running asynchronously",
             name_.c_str());
    timeStep_ = 0;
  }
}

bool RokubiminiSerialBusManager::createRokubimini(const std::string& rokubiminiName)
{
  auto rokubimini = std::make_shared<RokubiminiSerial>(rokubiminiName, nh_);
  rokubimini->load();
  rokubiminis_.emplace_back(rokubimini);
  if (!addRokubiminiToBus(rokubimini))
  {
    ROS_ERROR("[%s] Could not add rokubimini to bus", name_.c_str());
    return false;
  }
  return true;
}

bool RokubiminiSerialBusManager::loadBusParameters()
{
  std::string port_string = "port";
  if (nh_->hasParam(port_string))
  {
    nh_->getParam(port_string, serialPort_);
  }
  else
  {
    ROS_ERROR("Could not find serial port in Parameter Server: %s", port_string.c_str());
    return false;
  }
  return true;
}

bool RokubiminiSerialBusManager::addRokubiminiToBus(const std::shared_ptr<RokubiminiSerial>& rokubimini) const
{
  auto impl_ptr = std::make_shared<RokubiminiSerialImpl>(rokubimini->getName(), serialPort_);

  rokubimini->setImplPointer(impl_ptr);
  return true;
}

bool RokubiminiSerialBusManager::startupCommunication()
{
  for (auto& rokubimini : rokubiminis_)
  {
    auto rokubimini_serial = std::dynamic_pointer_cast<RokubiminiSerial>(rokubimini);
    fetchTimeStep();
    if (!rokubimini_serial->setPublishMode(timeStep_))
    {
      ROS_ERROR("[%s] Failed to set publish mode (sync vs async) to the serial device",
                rokubimini_serial->getName().c_str());
      return false;
    }
    if (!rokubimini_serial->init())
    {
      ROS_ERROR("[%s] Failed to initialize the serial device", rokubimini_serial->getName().c_str());
      return false;
    }
  }
  return true;
}

void RokubiminiSerialBusManager::setConfigMode()
{
  for (const auto& rokubimini : rokubiminis_)
  {
    auto rokubimini_serial = std::dynamic_pointer_cast<RokubiminiSerial>(rokubimini);
    if (!rokubimini_serial->setConfigMode())
    {
      ROS_ERROR("[%s] The Serial device could not switch to configuration mode", rokubimini_serial->getName().c_str());
    }
  }
}
void RokubiminiSerialBusManager::setRunMode()
{
  for (const auto& rokubimini : rokubiminis_)
  {
    auto rokubimini_serial = std::dynamic_pointer_cast<RokubiminiSerial>(rokubimini);
    if (!rokubimini_serial->setRunMode())
    {
      ROS_ERROR("[%s] The Serial device could not switch to run mode", rokubimini_serial->getName().c_str());
    }
  }
}
}  // namespace serial
}  // namespace rokubimini