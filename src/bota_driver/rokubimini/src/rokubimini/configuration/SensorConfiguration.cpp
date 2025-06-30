#include <rokubimini/configuration/SensorConfiguration.hpp>

namespace rokubimini
{
namespace configuration
{
SensorConfiguration::SensorConfiguration(const uint8_t calibrationMatrixActive,
                                         const uint8_t temperatureCompensationActive, const uint8_t imuActive,
                                         const uint8_t coordinateSystemConfigurationActive,
                                         const uint8_t inertiaCompensationActive,
                                         const uint8_t orientationEstimationActive)
  : calibrationMatrixActive_(calibrationMatrixActive)
  , temperatureCompensationActive_(temperatureCompensationActive)
  , imuActive_(imuActive)
  , coordinateSystemConfigurationActive_(coordinateSystemConfigurationActive)
  , inertiaCompensationActive_(inertiaCompensationActive)
  , orientationEstimationActive_(orientationEstimationActive)
{
}

bool SensorConfiguration::load(const std::string& key, const NodeHandlePtr& nh)
{
  bool success = false;
  std::string local_key;
  local_key = key + "/calibration_matrix_active";
  if (nh->hasParam(local_key))
  {
    bool calibration_matrix_active;
    nh->getParam(local_key, calibration_matrix_active);
    calibrationMatrixActive_ = static_cast<uint8_t>(calibration_matrix_active);
    success = true;
  }
  local_key = key + "/temperature_compensation_active";
  if (nh->hasParam(local_key))
  {
    bool temperature_compensation_active;
    nh->getParam(local_key, temperature_compensation_active);
    temperatureCompensationActive_ = static_cast<uint8_t>(temperature_compensation_active);
    success = true;
  }
  local_key = key + "/imu_active";
  if (nh->hasParam(local_key))
  {
    int imu_active;
    nh->getParam(local_key, imu_active);
    imuActive_ = static_cast<uint8_t>(imu_active);
    success = true;
  }
  local_key = key + "/coordinate_system_active";
  if (nh->hasParam(local_key))
  {
    bool coordinate_system_active;
    nh->getParam(local_key, coordinate_system_active);
    coordinateSystemConfigurationActive_ = static_cast<uint8_t>(coordinate_system_active);
    success = true;
  }
  local_key = key + "/inertia_compensation_active";
  if (nh->hasParam(local_key))
  {
    int inertia_compensation_active;
    nh->getParam(local_key, inertia_compensation_active);
    inertiaCompensationActive_ = static_cast<uint8_t>(inertia_compensation_active);
    success = true;
  }
  local_key = key + "/orientation_estimation_active";
  if (nh->hasParam(local_key))
  {
    int orientation_estimation_active;
    nh->getParam(local_key, orientation_estimation_active);
    orientationEstimationActive_ = static_cast<uint8_t>(orientation_estimation_active);
    success = true;
  }
  return success;
}

void SensorConfiguration::print() const
{
  ROS_INFO_STREAM("calibrationMatrixActive_: " << static_cast<unsigned int>(calibrationMatrixActive_));
  ROS_INFO_STREAM("temperatureCompensationActive_: " << static_cast<unsigned int>(temperatureCompensationActive_));
  ROS_INFO_STREAM("imuActive_: " << static_cast<unsigned int>(imuActive_));
  ROS_INFO_STREAM(
      "coordinateSystemConfigurationActive_: " << static_cast<unsigned int>(coordinateSystemConfigurationActive_));
  ROS_INFO_STREAM("inertiaCompensationActive_: " << static_cast<unsigned int>(inertiaCompensationActive_));
  ROS_INFO_STREAM("orientationEstimationActive_: " << static_cast<unsigned int>(orientationEstimationActive_));
}

}  // namespace configuration
}  // namespace rokubimini