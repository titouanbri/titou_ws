
#include <rokubimini_bus_manager/BusManager.hpp>

namespace rokubimini
{
bool RokubiminiBusManager::init()
{
  std::string setup_file;

  name_ = ros::this_node::getName();
  if (!load())
  {
    ROS_ERROR("[%s] Could not load bus configuration", name_.c_str());
    return false;
  }

  if (!startup())
  {
    ROS_WARN("[%s] Bus Manager could not startup", name_.c_str());
    return false;
  }
  createRokubiminiRosPublishers();
  createRokubiminiRosServices();
  std::vector<std::shared_ptr<Rokubimini>> rokubiminis;
  if (getRokubiminis().empty())
  {
    ROS_ERROR("[%s] No rokubimini available", name_.c_str());
    return false;
  }
  bota_worker::WorkerOptions options;
  options.callback_ = std::bind(&RokubiminiBusManager::update, this, std::placeholders::_1);
  options.defaultPriority_ = 10;  // this has high priority
  options.name_ = "RokubiminiBusManager::updateWorker";
  options.timeStep_ = loadTimeStep();
  if (options.timeStep_ != 0)
  {
    ROS_WARN_STREAM("Starting Worker at " << 1 / options.timeStep_ << " Hz, based on force/torque sampling rate.");
    if (!this->addWorker(options))
    {
      ROS_ERROR_STREAM("[RokubiminiBusManager] Worker " << options.name_ << "could not be added!");
      return false;
    }
  }
  return true;
}

bool RokubiminiBusManager::startup()
{
  if (isRunning_)
  {
    ROS_WARN_STREAM("Cannot start up, Rokubimini Bus Manager is already running.");
    return false;
  }
  ROS_DEBUG_STREAM("Starting up Rokubimini Bus Manager ...");
  startupWithoutCommunication();
  if (!startupCommunication())
  {
    shutdownWithoutCommunication();
    return false;
  }
  setConfigMode();
  startupWithCommunication();
  setRunMode();
  isRunning_ = true;
  return true;
}

bool RokubiminiBusManager::update(const bota_worker::WorkerEvent& event)
{
  ROS_DEBUG("[RokubiminiBusManager]: update called");
  readBus();
  updateProcessReading();
  writeToBus();
  publishRosMessages();
  return true;
}

void RokubiminiBusManager::cleanup()
{
  shutdown();
}

void RokubiminiBusManager::shutdown()
{
  if (!isRunning_)
  {
    ROS_WARN_STREAM("Cannot shut down, Rokubimini Bus Manager is not running.");
    return;
  }
  shutdownWithCommunication();
  shutdownBus();
  shutdownWithoutCommunication();
  isRunning_ = false;
}

bool RokubiminiBusManager::addRokubiminiToBus(const std::shared_ptr<Rokubimini>& rokubimini) const
{
  return true;
};
bool RokubiminiBusManager::load()
{
  if (!loadBusParameters())
  {
    ROS_ERROR("[%s] Could not load bus parameters", name_.c_str());
    return false;
  }
  if (!createRokubiminisFromParamServer())
  {
    ROS_ERROR("[%s] Could not create rokubiminis from parameter server", name_.c_str());
    return false;
  }
  return true;
}

bool RokubiminiBusManager::createRokubiminisFromParamServer()
{
  std::vector<std::string> keys;
  std::string rokubimini_name;
  nh_->getParamNames(keys);
  for (const auto& key : keys)
  {
    // append only parameter names with "/bus_name/rokubiminis" and have "/name"
    if (key.find(nh_->getNamespace() + "/rokubiminis") != std::string::npos && key.find("/name") != std::string::npos)
    {
      nh_->getParam(key, rokubimini_name);
      ROS_DEBUG("[%s] Key is: %s and name is: %s", name_.c_str(), key.c_str(), rokubimini_name.c_str());
      if (!createRokubimini(rokubimini_name))
      {
        ROS_ERROR("[%s] Could not create rokubimini", name_.c_str());
        return false;
      };
      ROS_DEBUG("[%s] Successfully created rokubimini with name: %s", name_.c_str(), rokubimini_name.c_str());
    }
  }
  return true;
}
}  // namespace rokubimini