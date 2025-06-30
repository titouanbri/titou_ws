#include <rokubimini_ethercat/RokubiminiEthercat.hpp>
#include <rokubimini_msgs/Reading.h>
#include <csignal>
#include <thread>

namespace rokubimini
{
namespace ethercat
{
void RokubiminiEthercat::postSetupConfiguration()
{
}
void RokubiminiEthercat::preSetupConfiguration()
{
  slavePtr_->preSetupConfiguration();
}

void RokubiminiEthercat::updateProcessReading()
{
  {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    slavePtr_->getReading(reading_);

    // Update statusword.
    auto statusword(reading_.getStatusword());
    setStatusword(statusword);
    statuswordRequested_ = false;

    // Handle some errors here

    // Log if activated.
    // if (logger_.logIsActive()) {
    //   logger_.addDataToLog(reading_);
    // }

    // External reading callbacks.
    for (const auto& reading_cb : readingCbs_)
    {
      reading_cb.second(getName(), reading_);
    }
  }

  if (deviceIsMissing())
  {
    Statusword statusword;
    //   statusword.setStateEnum(fsm::StateEnum::DeviceMissing);
    setStatusword(statusword);
  }
  else
  {
    // if (statusword_.isEmpty()) {
    //   requestAndSetStatusword();
    // }
    // if (statusword_.isEmpty()) {
    //   return;
    // }
  }

  // const fsm::StateEnum activeState = statusword_.getStateEnum();
  // if (activeState == fsm::StateEnum::NA) {
  //   ROS_WARN_STREAM("The FSM state is not available.");
  //   return;
  // }
  // stateMachine_.updateActiveState(activeState);
}

void RokubiminiEthercat::shutdownWithCommunication()
{
  slavePtr_->shutdown();
}

bool RokubiminiEthercat::deviceIsMissing() const
{
  return false;
}

bool RokubiminiEthercat::getSerialNumber(unsigned int& serialNumber)
{
  return slavePtr_->getSerialNumber(serialNumber);
}

bool RokubiminiEthercat::getForceTorqueSamplingRate(int& samplingRate)
{
  return slavePtr_->getForceTorqueSamplingRate(samplingRate);
}

bool RokubiminiEthercat::setForceTorqueFilter(const configuration::ForceTorqueFilter& filter)
{
  return slavePtr_->setForceTorqueFilter(filter);
}

bool RokubiminiEthercat::setAccelerationFilter(const unsigned int filter)
{
  return slavePtr_->setAccelerationFilter(filter);
}

bool RokubiminiEthercat::setAngularRateFilter(const unsigned int filter)
{
  return slavePtr_->setAngularRateFilter(filter);
}

bool RokubiminiEthercat::setAccelerationRange(const unsigned int range)
{
  return slavePtr_->setAccelerationRange(range);
}

bool RokubiminiEthercat::setAngularRateRange(const unsigned int range)
{
  return slavePtr_->setAngularRateRange(range);
}

bool RokubiminiEthercat::setForceTorqueOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset)
{
  return slavePtr_->setForceTorqueOffset(forceTorqueOffset);
}

bool RokubiminiEthercat::setSensorConfiguration(const configuration::SensorConfiguration& sensorConfiguration)
{
  if (!slavePtr_->setSensorConfiguration(sensorConfiguration))
  {
    return false;
  }
  getConfiguration().setSensorConfiguration(sensorConfiguration);
  return true;
}

bool RokubiminiEthercat::setSensorCalibration(const calibration::SensorCalibration& sensorCalibration)
{
  if (!slavePtr_->setSensorCalibration(sensorCalibration))
  {
    return false;
  }
  getConfiguration().setSensorCalibration(sensorCalibration);
  return true;
}

bool RokubiminiEthercat::setConfigMode()
{
  return slavePtr_->setConfigMode();
}

bool RokubiminiEthercat::setRunMode()
{
  return slavePtr_->setRunMode();
}

bool RokubiminiEthercat::saveConfigParameter()
{
  return slavePtr_->saveConfigParameter();
}

using RokubiminiReadingRos = rokubimini_msgs::Reading;
using RokubiminiWrenchRos = geometry_msgs::WrenchStamped;
using RokubiminiImuRos = sensor_msgs::Imu;
using RokubiminiTemperatureRos = sensor_msgs::Temperature;
void RokubiminiEthercat::createRosPublishers()
{
  readingPublisher_ = std::make_shared<ros::Publisher>(nh_->advertise<RokubiminiReadingRos>(
      nh_->getNamespace() + "/" + getName() + "/ft_sensor_readings/reading", 10, false));

  wrenchPublisher_ = std::make_shared<ros::Publisher>(nh_->advertise<RokubiminiWrenchRos>(
      nh_->getNamespace() + "/" + getName() + "/ft_sensor_readings/wrench", 10, false));

  imuPublisher_ = std::make_shared<ros::Publisher>(
      nh_->advertise<RokubiminiImuRos>(nh_->getNamespace() + "/" + getName() + "/ft_sensor_readings/imu", 10, false));

  temperaturePublisher_ = std::make_shared<ros::Publisher>(nh_->advertise<RokubiminiTemperatureRos>(
      nh_->getNamespace() + "/" + getName() + "/ft_sensor_readings/temperature", 10, false));
}

void RokubiminiEthercat::publishRosMessages()
{
  auto reading = getReading();
  rokubimini_msgs::Reading reading_msg;
  reading_msg.header.stamp = reading.getWrench().header.stamp;
  // reading_msg.header.frame_id = reading.getWrench().header.frame_id;
  reading_msg.statusword = reading.getStatusword().getData();
  reading_msg.imu = reading.getImu();
  reading_msg.wrench = reading.getWrench();
  reading_msg.externalImu = reading.getExternalImu();
  reading_msg.isForceTorqueSaturated = reading.isForceTorqueSaturated();
  reading_msg.temperature = reading.getTemperature();

  // if reset wrench is triggered take the mean of the measurements
  if (computeMeanWrenchFlag_)
  {
    std::lock_guard<std::recursive_mutex> lock(meanWrenchOffsetMutex_);
    wrenchMessageCount_++;
    meanWrenchOffset_.force.x +=
        ((reading.getWrench().wrench.force.x - meanWrenchOffset_.force.x) / wrenchMessageCount_);
    meanWrenchOffset_.force.y +=
        ((reading.getWrench().wrench.force.y - meanWrenchOffset_.force.y) / wrenchMessageCount_);
    meanWrenchOffset_.force.z +=
        ((reading.getWrench().wrench.force.z - meanWrenchOffset_.force.z) / wrenchMessageCount_);
    meanWrenchOffset_.torque.x +=
        ((reading.getWrench().wrench.torque.x - meanWrenchOffset_.torque.x) / wrenchMessageCount_);
    meanWrenchOffset_.torque.y +=
        ((reading.getWrench().wrench.torque.y - meanWrenchOffset_.torque.y) / wrenchMessageCount_);
    meanWrenchOffset_.torque.z +=
        ((reading.getWrench().wrench.torque.z - meanWrenchOffset_.torque.z) / wrenchMessageCount_);
  }
  readingPublisher_->publish(reading_msg);
  wrenchPublisher_->publish(reading.getWrench());
  imuPublisher_->publish(reading.getImu());
  temperaturePublisher_->publish(reading.getTemperature());
}

void RokubiminiEthercat::signalShutdown()
{
  // wait a small amount of time so that the callback can return the result to the user.
  std::this_thread::sleep_for(std::chrono::microseconds(500));
  kill(getpid(), SIGINT);
}

bool RokubiminiEthercat::firmwareUpdateCallback(rokubimini_msgs::FirmwareUpdateEthercat::Request& request,
                                                rokubimini_msgs::FirmwareUpdateEthercat::Response& response)
{
  response.result = slavePtr_->firmwareUpdate(request.file_path, request.file_name, request.password);
  if (!slavePtr_->isRunning())
  {
    // time to shut down the ROS node.
    std::thread shutdown_thread(&RokubiminiEthercat::signalShutdown, this);
    shutdown_thread.detach();
  }
  return true;
}

bool RokubiminiEthercat::resetWrenchCallback(rokubimini_msgs::ResetWrench::Request& request,
                                             rokubimini_msgs::ResetWrench::Response& response)
{
  ROS_INFO("[%s] Reseting sensor measurements...", name_.c_str());

  // initialize all variables to zero
  meanWrenchOffset_.force.x = 0;
  meanWrenchOffset_.force.y = 0;
  meanWrenchOffset_.force.z = 0;
  meanWrenchOffset_.torque.x = 0;
  meanWrenchOffset_.torque.y = 0;
  meanWrenchOffset_.torque.z = 0;
  wrenchMessageCount_ = 0;
  // enable the computation of the mean wrench
  computeMeanWrenchFlag_ = true;
  // wait for computing the mean of wrench measurements
  while (wrenchMessageCount_ != TOTAL_NUMBER_OF_WRENCH_MESSAGES)
    ;
  // disable the computation of the mean wrench
  computeMeanWrenchFlag_ = false;
  if (!setConfigMode())
  {
    ROS_ERROR("[%s] Device could not switch to config mode", name_.c_str());
    response.success = false;
    return true;
  }
  geometry_msgs::Wrench wrench;
  // lock to get the meanWrenchOffset value
  {
    std::lock_guard<std::recursive_mutex> lock(meanWrenchOffsetMutex_);
    wrench = meanWrenchOffset_;
  }
  geometry_msgs::Wrench desired_wrench = request.desired_wrench;
  auto current_offset = configuration_.getForceTorqueOffset();
  Eigen::Matrix<double, 6, 1> new_offset;
  // new offset = current wrench measurements + current offset - desired offset
  new_offset(0, 0) = desired_wrench.force.x - wrench.force.x + current_offset(0, 0);
  new_offset(1, 0) = desired_wrench.force.y - wrench.force.y + current_offset(1, 0);
  new_offset(2, 0) = desired_wrench.force.z - wrench.force.z + current_offset(2, 0);
  new_offset(3, 0) = desired_wrench.torque.x - wrench.torque.x + current_offset(3, 0);
  new_offset(4, 0) = desired_wrench.torque.y - wrench.torque.y + current_offset(4, 0);
  new_offset(5, 0) = desired_wrench.torque.z - wrench.torque.z + current_offset(5, 0);
  ROS_DEBUG_STREAM("[" << getName() << "] "
                       << "New offset is: " << new_offset);
  if (!setForceTorqueOffset(new_offset))
  {
    ROS_ERROR("[%s] Could not write new offset to device", name_.c_str());
    response.success = false;
    return true;
  }
  if (!setRunMode())
  {
    ROS_ERROR("[%s] Device could not switch to run mode", name_.c_str());
    response.success = false;
    return true;
  }
  response.success = true;
  configuration_.setForceTorqueOffset(new_offset);
  ROS_INFO("[%s] Sensor measurements are reset successfully", name_.c_str());
  return true;
}
void RokubiminiEthercat::createRosServices()
{
  firmwareUpdateService_ = nh_->advertiseService(nh_->getNamespace() + "/" + getName() + "/firmware_update",
                                                 &RokubiminiEthercat::firmwareUpdateCallback, this);
  resetWrenchService_ = nh_->advertiseService(nh_->getNamespace() + "/" + getName() + "/reset_wrench",
                                              &RokubiminiEthercat::resetWrenchCallback, this);
}
bool RokubiminiEthercat::sendSdoReadGeneric(const std::string& indexString, const std::string& subindexString,
                                            const std::string& valueTypeString, std::string& valueString)
{
  return slavePtr_->sendSdoReadGeneric(indexString, subindexString, valueTypeString, valueString);
}

bool RokubiminiEthercat::sendSdoWriteGeneric(const std::string& indexString, const std::string& subindexString,
                                             const std::string& valueTypeString, const std::string& valueString)
{
  return slavePtr_->sendSdoWriteGeneric(indexString, subindexString, valueTypeString, valueString);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int8_t& value)
{
  return slavePtr_->sendSdoReadInt8(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int16_t& value)
{
  return slavePtr_->sendSdoReadInt16(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int32_t& value)
{
  return slavePtr_->sendSdoReadInt32(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int64_t& value)
{
  return slavePtr_->sendSdoReadInt64(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint8_t& value)
{
  return slavePtr_->sendSdoReadUInt8(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint16_t& value)
{
  return slavePtr_->sendSdoReadUInt16(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint32_t& value)
{
  return slavePtr_->sendSdoReadUInt32(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint64_t& value)
{
  return slavePtr_->sendSdoReadUInt64(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     float& value)
{
  return slavePtr_->sendSdoReadFloat(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     double& value)
{
  return slavePtr_->sendSdoReadDouble(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int8_t value)
{
  return slavePtr_->sendSdoWriteInt8(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int16_t value)
{
  return slavePtr_->sendSdoWriteInt16(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int32_t value)
{
  return slavePtr_->sendSdoWriteInt32(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int64_t value)
{
  return slavePtr_->sendSdoWriteInt64(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint8_t value)
{
  return slavePtr_->sendSdoWriteUInt8(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint16_t value)
{
  return slavePtr_->sendSdoWriteUInt16(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint32_t value)
{
  return slavePtr_->sendSdoWriteUInt32(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint64_t value)
{
  return slavePtr_->sendSdoWriteUInt64(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const float value)
{
  return slavePtr_->sendSdoWriteFloat(index, subindex, completeAccess, value);
}

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const double value)
{
  return slavePtr_->sendSdoWriteDouble(index, subindex, completeAccess, value);
}

}  // namespace ethercat
}  // namespace rokubimini