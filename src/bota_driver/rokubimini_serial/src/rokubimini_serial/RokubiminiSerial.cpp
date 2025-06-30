#include <rokubimini_serial/RokubiminiSerial.hpp>
#include <rokubimini_msgs/Reading.h>
#include <csignal>
#include <thread>

namespace rokubimini
{
namespace serial
{
void RokubiminiSerial::postSetupConfiguration()
{
  /*
  ** Print configurations of the sensor
  */
  // configuration_.printConfiguration();
  ROS_DEBUG_STREAM("[" << name_.c_str() << "] Calibration Matrix of the sensor: "
                       << configuration_.getSensorCalibration().getCalibrationMatrix() << std::endl);

  if (implPtr_->runsAsync())
  {
    // start publishing thread
    if (!publishingThread_.joinable())
    {
      ROS_INFO("[%s] Launching publishing thread.", name_.c_str());
      publishingThread_ = std::thread{ &RokubiminiSerial::update, this };
    }
  }
  implPtr_->startup();
}

void RokubiminiSerial::preSetupConfiguration()
{
  parseCommunicationMsgs();
}

void RokubiminiSerial::parseCommunicationMsgs()
{
  if (!implPtr_->parseCommunicationMsgs())
  {
    ROS_ERROR("[%s] Failed to parse communication messages", name_.c_str());
  }
  if (productName_ != implPtr_->getProductName())
  {
    ROS_ERROR("[%s] Invalid product name '%s' given, didn't match the actual product name of the device: '%s'",
              name_.c_str(), productName_.c_str(), implPtr_->getProductName().c_str());
  }
}

bool RokubiminiSerial::setPublishMode(double timeStep)
{
  // if the timeStep variable is 0, we set the flag 'runsAsync' to true. By default the 'runsAsync' flag is false.
  if (timeStep == 0)
  {
    implPtr_->setRunsAsync(true);
  }
  else
  {
    implPtr_->setPollingTimeStep(timeStep);
  }
  return true;
}

bool RokubiminiSerial::init()
{
  return implPtr_->init();
}

void RokubiminiSerial::update()
{
  while (implPtr_->isRunning())
  {
    updateProcessReading();
    publishRosMessages();
  }
}
void RokubiminiSerial::updateProcessReading()
{
  /* ATTENTION
  **
  ** All of this is legacy code
  **
  */
  // if the polling is async and the thread reaching this method is from application, return immediately.
  {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    implPtr_->getReading(reading_);

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
    // statusword.setStateEnum(fsm::StateEnum::DeviceMissing);
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

void RokubiminiSerial::shutdownWithCommunication()
{
  implPtr_->shutdown();
  if (implPtr_->runsAsync())
  {
    // Shutdown the publishing thread if running
    if (publishingThread_.joinable())
    {
      publishingThread_.join();
    }
  }
}

bool RokubiminiSerial::deviceIsMissing() const
{
  return false;
}

bool RokubiminiSerial::getSerialNumber(unsigned int& serialNumber)
{
  return implPtr_->getSerialNumber(serialNumber);
}

bool RokubiminiSerial::getForceTorqueSamplingRate(int& samplingRate)
{
  return implPtr_->getForceTorqueSamplingRate(samplingRate);
}

bool RokubiminiSerial::setForceTorqueFilter(const configuration::ForceTorqueFilter& filter)
{
  return implPtr_->setForceTorqueFilter(filter);
}

bool RokubiminiSerial::setAccelerationFilter(const unsigned int filter)
{
  return implPtr_->setAccelerationFilter(filter);
}

bool RokubiminiSerial::setAngularRateFilter(const unsigned int filter)
{
  return implPtr_->setAngularRateFilter(filter);
}

bool RokubiminiSerial::setAccelerationRange(const unsigned int range)
{
  return implPtr_->setAccelerationRange(range);
}

bool RokubiminiSerial::setAngularRateRange(const unsigned int range)
{
  return implPtr_->setAngularRateRange(range);
}

bool RokubiminiSerial::setForceTorqueOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset)
{
  return implPtr_->setForceTorqueOffset(forceTorqueOffset);
}

bool RokubiminiSerial::setSensorConfiguration(const configuration::SensorConfiguration& sensorConfiguration)
{
  if (!implPtr_->setSensorConfiguration(sensorConfiguration))
  {
    return false;
  }
  getConfiguration().setSensorConfiguration(sensorConfiguration);
  return true;
}

bool RokubiminiSerial::setSensorCalibration(const calibration::SensorCalibration& sensorCalibration)
{
  if (!implPtr_->setSensorCalibration(sensorCalibration))
  {
    return false;
  }
  getConfiguration().setSensorCalibration(sensorCalibration);
  return true;
}

bool RokubiminiSerial::setConfigMode()
{
  return implPtr_->setConfigMode();
}

bool RokubiminiSerial::setRunMode()
{
  return implPtr_->setRunMode();
}

bool RokubiminiSerial::saveConfigParameter()
{
  return implPtr_->saveConfigParameter();
}

bool RokubiminiSerial::loadConfig()
{
  return implPtr_->loadConfig();
}

bool RokubiminiSerial::printUserConfig()
{
  return implPtr_->printUserConfig();
}

bool RokubiminiSerial::setHardwareReset()
{
  return implPtr_->setHardwareReset();
}

bool RokubiminiSerial::setInitMode()
{
  return implPtr_->setInitMode();
}

using RokubiminiReadingRos = rokubimini_msgs::Reading;
using RokubiminiWrenchRos = geometry_msgs::WrenchStamped;
using RokubiminiTemperatureRos = sensor_msgs::Temperature;
void RokubiminiSerial::createRosPublishers()
{
  readingPublisher_ = std::make_shared<ros::Publisher>(nh_->advertise<RokubiminiReadingRos>(
      nh_->getNamespace() + "/" + getName() + "/ft_sensor_readings/reading", 10, false));

  wrenchPublisher_ = std::make_shared<ros::Publisher>(nh_->advertise<RokubiminiWrenchRos>(
      nh_->getNamespace() + "/" + getName() + "/ft_sensor_readings/wrench", 10, false));

  temperaturePublisher_ = std::make_shared<ros::Publisher>(nh_->advertise<RokubiminiTemperatureRos>(
      nh_->getNamespace() + "/" + getName() + "/ft_sensor_readings/temperature", 10, false));
}

void RokubiminiSerial::publishRosMessages()
{
  if (implPtr_->hasFrameSync() && implPtr_->isRunning() && implPtr_->frameTimestampOffsetIsValid())
  {
    auto reading = getReading();
    rokubimini_msgs::Reading reading_msg;
    reading_msg.header.stamp = reading.getWrench().header.stamp;
    reading_msg.header.frame_id = reading.getWrench().header.frame_id;
    reading_msg.statusword = reading.getStatusword().getData();
    reading_msg.wrench = reading.getWrench();
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
    temperaturePublisher_->publish(reading.getTemperature());
    noFrameSyncCounter_ = 0;
  }
  else
  {
    noFrameSyncCounter_++;
  }
  // check if there is no synced frame over 100 times
  if (noFrameSyncCounter_ > 100)
  {
    ROS_ERROR_THROTTLE(3, "[%s] Driver failed to synchronize with the device", name_.c_str());
  }
}

void RokubiminiSerial::signalShutdown()
{
  // wait a small amount of time so that the callback can return the result to the user.
  std::this_thread::sleep_for(std::chrono::microseconds(500));
  kill(getpid(), SIGINT);
}

bool RokubiminiSerial::firmwareUpdateCallback(rokubimini_msgs::FirmwareUpdateSerial::Request& request,
                                              rokubimini_msgs::FirmwareUpdateSerial::Response& response)
{
  response.result = implPtr_->firmwareUpdate(request.file_path);
  if (!implPtr_->isRunning())
  {
    // time to shut down the ROS node.
    std::thread shutdown_thread(&RokubiminiSerial::signalShutdown, this);
    shutdown_thread.detach();
  }
  return true;
}

bool RokubiminiSerial::resetWrenchCallback(rokubimini_msgs::ResetWrench::Request& request,
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
  // new offset = current offset + desired offset - current wrench measurements
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
void RokubiminiSerial::createRosServices()
{
  firmwareUpdateService_ = nh_->advertiseService(nh_->getNamespace() + "/" + getName() + "/firmware_update",
                                                 &RokubiminiSerial::firmwareUpdateCallback, this);
  resetWrenchService_ = nh_->advertiseService(nh_->getNamespace() + "/" + getName() + "/reset_wrench",
                                              &RokubiminiSerial::resetWrenchCallback, this);
}

}  // namespace serial
}  // namespace rokubimini