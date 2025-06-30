#include <rokubimini_ethercat/RokubiminiEthercatBusManager.hpp>
#include <thread>
namespace rokubimini
{
namespace ethercat
{
double RokubiminiEthercatBusManager::loadTimeStep()
{
  double time_step;
  if (!nh_->getParam("time_step", time_step))
  {
    ROS_INFO("[%s] Could not find the 'time_step' parameter in Parameter Server. Setting it to minimum sampling rate.",
             name_.c_str());

    int minimumft_sampling_rate = std::numeric_limits<int>::max();
    /* the following function has to be after setting the filter in order Sampling rate Object to be updated.
     * The filters are set inside the startup function above */
    for (const auto& rokubimini : rokubiminis_)
    {
      int ft_sampling_rate;
      rokubimini->getForceTorqueSamplingRate(ft_sampling_rate);
      minimumft_sampling_rate = std::min(minimumft_sampling_rate, ft_sampling_rate);
    }
    time_step = 1.0 / static_cast<float>(minimumft_sampling_rate);
  }
  return time_step;
}
bool RokubiminiEthercatBusManager::createRokubimini(const std::string& rokubiminiName)
{
  auto rokubimini = std::make_shared<RokubiminiEthercat>(rokubiminiName, nh_);
  rokubimini->load();
  rokubiminis_.emplace_back(rokubimini);
  // Create a new bus.
  auto bus = new rokubimini::soem_interface::EthercatBusBase(ethercatBusName_);
  bus_ = std::unique_ptr<rokubimini::soem_interface::EthercatBusBase>(bus);
  if (!addRokubiminiToBus(rokubimini, bus))
  {
    ROS_ERROR("[%s] Could not add rokubimini to bus", name_.c_str());
    return false;
  }
  return true;
}

bool RokubiminiEthercatBusManager::loadBusParameters()
{
  std::string ethercat_bus_string = name_ + "/ethercat_bus";
  if (nh_->hasParam(ethercat_bus_string))
  {
    nh_->getParam(ethercat_bus_string, ethercatBusName_);
  }
  else
  {
    ROS_ERROR("[%s] Could not find ethercat bus in Parameter Server: %s", name_.c_str(), ethercat_bus_string.c_str());
    return false;
  }
  return true;
}

bool RokubiminiEthercatBusManager::addRokubiminiToBus(const std::shared_ptr<RokubiminiEthercat>& rokubimini,
                                                      rokubimini::soem_interface::EthercatBusBase* bus) const
{
  std::string ethercat_address_string = name_ + "/rokubiminis/" + rokubimini->getName() + "/ethercat_address";
  uint32_t ethercat_addr;
  if (nh_->hasParam(ethercat_address_string))
  {
    int ethercat_address;
    nh_->getParam(ethercat_address_string, ethercat_address);
    if (ethercat_address <= 0)
    {
      throw ros::Exception("The EtherCAT address is not a positive number");
    }
    ethercat_addr = static_cast<uint32_t>(ethercat_address);
  }
  else
  {
    ROS_ERROR("Could not find ethercat address in Parameter Server: %s", ethercat_address_string.c_str());
    return false;
  }
  PdoTypeEnum ethercat_pdo_enum = PdoTypeEnum::A;
  auto slave = std::make_shared<RokubiminiEthercatSlave>(rokubimini->getName(), bus, ethercat_addr, ethercat_pdo_enum);
  slave->setProductName(rokubimini->getProductName());
  if (!bus->addSlave(slave))
  {
    return false;
  }

  rokubimini->setSlavePointer(slave);
  return true;
}

void RokubiminiEthercatBusManager::setConfigMode()
{
  setBusPreOperational();
  // Sleep for some time to give the slave time to execute the pre-op cb
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}
void RokubiminiEthercatBusManager::setRunMode()
{
  setBusSafeOperational();
  setBusOperational();
}

bool RokubiminiEthercatBusManager::startupBus()
{
  bool success = startupCommunication();
  setBusOperational();
  return success;
}

void RokubiminiEthercatBusManager::setBusOperational()
{
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  // Only set the state but do not wait for it, since some devices (e.g. junctions) might not be able to reach it.
  bus_->setState(EC_STATE_OPERATIONAL);
}

void RokubiminiEthercatBusManager::setBusPreOperational()
{
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  // Only set the state but do not wait for it, since some devices (e.g. junctions) might not be able to reach it.
  bus_->setState(EC_STATE_PRE_OP);
}

void RokubiminiEthercatBusManager::setBusSafeOperational()
{
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  // Only set the state but do not wait for it, since some devices (e.g. junctions) might not be able to reach it.
  bus_->setState(EC_STATE_SAFE_OP);
}

void RokubiminiEthercatBusManager::waitForState(const uint16_t state, const uint16_t slave,
                                                const unsigned int maxRetries, const double retrySleep)
{
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  bus_->waitForState(state, slave, maxRetries, retrySleep);
}

bool RokubiminiEthercatBusManager::startupCommunication()
{
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  if (!bus_->startup())
  {
    ROS_ERROR("Failed to startup bus %s.", ethercatBusName_.c_str());
    return false;
  }
  return true;
}

void RokubiminiEthercatBusManager::readBus()
{
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  bus_->updateRead();
}

void RokubiminiEthercatBusManager::writeToBus()
{
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  bus_->updateWrite();
}

void RokubiminiEthercatBusManager::shutdownBus()
{
  std::lock_guard<std::recursive_mutex> lock(busMutex_);
  bus_->shutdown();
}

}  // namespace ethercat
}  // namespace rokubimini