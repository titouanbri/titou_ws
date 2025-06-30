#!/usr/bin/env python3


#pragma once

#include <rokubimini/Rokubimini.hpp>
#include <rokubimini_ethercat/RokubiminiEthercatSlave.hpp>
#include <utility>
#include <rokubimini_msgs/FirmwareUpdateEthercat.h>
#include <rokubimini_msgs/ResetWrench.h>

namespace rokubimini
{
namespace ethercat
{
/**
 * @class RokubiminiEthercat
 *
 * @brief The Rokubimini Ethercat class.
 *
 * Inherits from the Rokubimini class. It's the interface
 * in the BRIDGE pattern used. It provides the API to be called by
 * client code and is used for interfacing with the implementation
 * class called RokubiminiEthercatSlave.
 *
 */

class RokubiminiEthercat : public Rokubimini
{
public:
  /**
   * @fn RokubiminiEthercat()
   *
   * @brief Default constructor.
   *
   * The default constructor of the RokubiminiEthercat class.
   */

  RokubiminiEthercat() = default;

  /**
   * @fn RokubiminiEthercat(const std::string name, NodeHandlePtr nh)
   *
   * @brief Constructor with initialization list for the name and nh
   *
   */
  RokubiminiEthercat(const std::string& name, NodeHandlePtr nh) : Rokubimini(name, std::move(nh)){ /* do nothing */ };
  // RokubiminiEthercat() = default;
  ~RokubiminiEthercat() override = default;

  /**
   * @fn virtual void preSetupConfiguration()
   *
   * @brief Pre-setup configuration hook.
   *
   *
   */
  void preSetupConfiguration() override;

  /**
   * @fn void postSetupConfiguration()
   *
   * @brief Post-setup configuration hook.
   *
   *
   */

  void postSetupConfiguration() override;

  /**
   * @fn void updateProcessReading()
   *
   * @brief Updates the \a RokubiminiEthercat object with new measurements.
   *
   * This method updates the internal \a Reading variable of \a
   * RokubiminiEthercat, by getting the new values from its
   * implementation \a RokubiminiEthercatSlave.
   */
  void updateProcessReading() override;

  /**
   * @fn bool deviceIsMissing()
   *
   * @brief Checks if the device is missing.
   *
   */

  bool deviceIsMissing() const override;

  /**
   * @fn void shutdownWithCommunication()
   *
   * @brief Shuts down a Rokubimini Ethercat device before
   * communication has been closed.
   *
   * This method shuts down a Rokubimini Ethercat device before the
   * EthercatBusManager has terminated communication with the device.
   *
   */

  void shutdownWithCommunication() override;

  /**
   * @fn void setSlavePointer(const RokubiminiEthercatSlavePtr &slavePtr)
   *
   * @brief Sets a pointer to the implementation.
   *
   * This method realizes the pimpl paradigm. Through it,
   * the RokubiminiEthercat object holds a pointer to its
   * implementation, a RokubiminiEthercatSlave object.
   *
   * @param slavePtr The pointer to the RokubiminiEthercatSlave
   * implementation.
   *
   */

  void setSlavePointer(const RokubiminiEthercatSlavePtr& slavePtr)
  {
    slavePtr_ = slavePtr;
  }

  /**
   * @fn void setState(const uint16_t state)
   *
   * @brief Sets the desired EtherCAT state machine state of the device in the bus.
   *
   *
   * @param state Desired state.
   *
   */

  void setState(const uint16_t state)
  {
    slavePtr_->setState(state);
  }

  /**
   * @fn bool waitForState(const uint16_t state)
   *
   * @brief Wait for an EtherCAT state machine state to be reached.
   *
   *
   * @param state Desired state.
   * @return True if the state has been reached within the timeout.
   */

  bool waitForState(const uint16_t state)
  {
    return slavePtr_->waitForState(state);
  }

  /**
   * @fn bool getSerialNumber(unsigned int &serialNumber)
   *
   * @brief Gets the serial number of the device.
   *
   *
   * @param serialNumber The serial number to be fetched.
   * @return True if the serial number was successfully fetched.
   *
   */
  // Missing: Methods for calling SDO
  bool getSerialNumber(unsigned int& samplingRate) override;

  /**
   * @fn bool getForceTorqueSamplingRate(int &samplingRate)
   *
   * @brief Gets the force torque sampling rate of the device.
   *
   * @param samplingRate The force torque sampling rate to be
   * fetched.
   * @return True if the force torque sampling rate was
   * successfully fetched.
   *
   */

  bool getForceTorqueSamplingRate(int& samplingRate) override;

  /**
   * @fn bool setForceTorqueFilter(const
   * configuration::ForceTorqueFilter &filter)
   *
   * @brief Sets a force torque filter to the device.
   *
   * @param filter The filter to be set.
   * @return True if the force torque filter was
   * successfully set.
   *
   */

  bool setForceTorqueFilter(const configuration::ForceTorqueFilter& filter) override;

  /**
   * @fn bool setAccelerationFilter(const unsigned int filter)
   *
   * @brief Sets an acceleration filter to the device.
   *
   * @param filter The filter to be set.
   * @return True if the acceleration torque filter was
   * successfully set.
   *
   */

  bool setAccelerationFilter(const unsigned int filter) override;

  /**
   * @fn bool setAngularRateFilter (const unsigned int filter)
   *
   * @brief Sets an angular rate filter to the device.
   *
   * @param filter The filter to be set.
   * @return True if the angular rate filter was
   * successfully set.
   *
   */

  bool setAngularRateFilter(const unsigned int filter) override;

  /**
   * @fn bool setAccelerationRange(const unsigned int range)
   *
   * @brief Sets an acceleration range to the device.
   *
   * @param range The range to be set.
   * @return True if the acceleration range was
   * successfully set.
   *
   */

  bool setAccelerationRange(const unsigned int range) override;

  /**
   * @fn bool setAngularRateRange(const unsigned int range)
   *
   * @brief Sets an angular rate range to the device.
   *
   * @param range The range to be set.
   * @return True if the angular rate range was
   * successfully set.
   *
   */

  bool setAngularRateRange(const unsigned int range) override;

  /**
   * @fn bool setForceTorqueOffset(const Eigen::Matrix<double, 6, 1> &forceTorqueOffset)
   *
   * @brief Sets a force torque offset to the device.
   *
   * @param forceTorqueOffset The offset to be set.
   * @return True if the offset was
   * successfully set.
   *
   */

  bool setForceTorqueOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset) override;

  /**
   * @fn bool setSensorConfiguration(const configuration::SensorConfiguration &sensorConfiguration)
   *
   * @brief Sets a sensor configuration to the device.
   *
   * @param sensorConfiguration The configuration to be set.
   * @return True if the configuration was
   * successfully set.
   *
   */

  bool setSensorConfiguration(const configuration::SensorConfiguration& sensorConfiguration) override;

  /**
   * @fn bool setSensorCalibration(const calibration::SensorCalibration &sensorCalibration)
   *
   * @brief Sets a sensor calibration to the device.
   *
   * @param sensorCalibration The calibration to be set.
   * @return True if the calibration was
   * successfully set.
   *
   */

  bool setSensorCalibration(const calibration::SensorCalibration& sensorCalibration) override;

  /**
   * @fn bool setConfigMode()
   *
   * @brief Sets the device in config mode.
   *
   * @return True if the operation was successful.
   */
  bool setConfigMode();

  /**
   * @fn bool setRunMode()
   *
   * @brief Sets the device in run mode.
   *
   * @return True if the operation was successful.
   */
  bool setRunMode();
  /**
   * @fn bool saveConfigParameter()
   *
   * @brief Saves the current configuration to the device.
   *
   * @return True if the configuration was
   * successfully saved in the device.
   *
   */
  bool saveConfigParameter() override;

  /**
   * @fn void createRosPublishers()
   *
   * @brief Adds ROS publishers related to the device.
   *
   */
  void createRosPublishers() override;

  /**
   * @fn void signalShutdown()
   *
   * @brief Signals shutdown for the ROS node. It's used if a firmware update was successful.
   *
   */
  void signalShutdown();

  /**
   * @fn firmwareUpdateCallback(rokubimini_msgs::FirmwareUpdateEthercat::Request& request,
                              rokubimini_msgs::FirmwareUpdateEthercat::Response& response);
   *
   * @brief The callback for the firmware update ROS service.
   * @param request The request of the ROS service.
   * @param response The response of the ROS service.
   * @return True always.
   *
   */
  bool firmwareUpdateCallback(rokubimini_msgs::FirmwareUpdateEthercat::Request& request,
                              rokubimini_msgs::FirmwareUpdateEthercat::Response& response);

  /**
   * @fn resetWrenchCallback(rokubimini_msgs::ResetWrench::Request& request, rokubimini_msgs::ResetWrench::Response&
   * response)
   *
   * @brief The callback for the reset wrench ROS service.
   * @param request The request of the ROS service.
   * @param response The response of the ROS service.
   * @return True always.
   *
   */
  bool resetWrenchCallback(rokubimini_msgs::ResetWrench::Request& request,
                           rokubimini_msgs::ResetWrench::Response& response);
  /**
   * @fn void createRosServices()
   *
   * @brief Adds ROS services related to the device.
   *
   */
  void createRosServices() override;

  /**
   * @fn void publishRosMessages()
   *
   * @brief Publishes ROS messages with data from the rokubimini device.
   *
   *
   */
  void publishRosMessages() override;

  /**
   * @fn template <typename Value>
  bool sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value &value)
   *
   * @brief Sends a read SDO to the Ethercat device.
   *
   * @param index Index of the SDO.
   * @param subindex  Sub-index of the SDO.
   * @param completeAccess Access all sub-indices at once.
   * @param value Return argument, will contain the value which was read.
   * @return True if the SDO read was sent successfully.
   *
  */

  template <typename Value>
  bool sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value& value);

  /**
   * @fn bool sendSdoReadGeneric(const std::string &indexString, const std::string &subindexString, const std::string
   * &valueTypeString, std::string &valueString)
   *
   * @brief Sends a generic reading SDO to the Ethercat device.
   *
   * @param indexString A string containing the index of the SDO.
   * @param subindexString  A string containing the sub-index of the SDO.
   * @param valueTypeString A string containing the type of the value to read.
   * @param valueString A string containing the value to read.
   * @return True if the SDO read was sent successfully.
   *
   */
  bool sendSdoReadGeneric(const std::string& indexString, const std::string& subindexString,
                          const std::string& valueTypeString, std::string& valueString);

  /**
   * @fn template <typename Value>
  bool sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const Value value)
   *
   * @brief Sends a write SDO to the Ethercat device.
   *
   * @param index Index of the SDO.
   * @param subindex  Sub-index of the SDO.
   * @param completeAccess Access all sub-indices at once.
   * @param value Value to write.
   * @return True if the SDO write was sent successfully.
   *
  */
  template <typename Value>
  bool sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const Value value);

  /**
   * @fn bool sendSdoWriteGeneric(const std::string &indexString, const std::string &subindexString, const std::string
   * &valueTypeString, const std::string &valueString)
   *
   * @brief Sends a generic write SDO to the Ethercat device.
   *
   * @param indexString A string containing the index of the SDO.
   * @param subindexString  A string containing the sub-index of the SDO.
   * @param valueTypeString A string containing the type of the value to write.
   * @param valueString A string containing the value to write.
   * @return True if the SDO write was sent successfully.
   *
   */

  bool sendSdoWriteGeneric(const std::string& indexString, const std::string& subindexString,
                           const std::string& valueTypeString, const std::string& valueString);

protected:
  /**
   * @var RokubiminiEthercatSlavePtr slavePtr_
   *
   * @brief The pointer to implementation.
   *
   */

  RokubiminiEthercatSlavePtr slavePtr_{ nullptr };

  using RosPublisherPtr = std::shared_ptr<ros::Publisher>;

  /**
   * @var RosPublisherPtr readingPublisher_
   *
   * @brief The rokubimini_msgs::Reading publisher.
   *
   */

  RosPublisherPtr readingPublisher_;

  /**
   * @var RosPublisherPtr wrenchPublisher_
   *
   * @brief The sensor_msgs::Wrench publisher.
   *
   */
  RosPublisherPtr wrenchPublisher_;

  /**
   * @var RosPublisherPtr imuPublisher_
   *
   * @brief The sensor_msgs::Imu publisher.
   *
   */
  RosPublisherPtr imuPublisher_;

  /**
   * @var RosPublisherPtr temperaturePublisher_
   *
   * @brief The sensor_msgs::Temperature publisher.
   *
   */
  RosPublisherPtr temperaturePublisher_;

  /**
   * @var ros::ServiceServer firmwareUpdateService_
   *
   * @brief The service for firmware updates.
   *
   */
  ros::ServiceServer firmwareUpdateService_;

  /**
   * @var ros::ServiceServer resetWrenchService_
   *
   * @brief The service for resetting the sensor wrench measurements.
   *
   */
  ros::ServiceServer resetWrenchService_;

  /**
   * @var std::atomic<bool> computeMeanWrenchFlag_
   *
   * @brief The flag for enabling the computation of the mean of the wrench measurements. Used withing the "reset
   * service" callback.
   *
   */
  std::atomic<bool> computeMeanWrenchFlag_{ false };

  /**
   * @var std::atomic<uint32_t> wrenchMessageCount_
   *
   * @brief The counter that is used for the computation of the mean of the wrench measurements. Used for the "reset
   * service" callback.
   *
   */
  std::atomic<uint32_t> wrenchMessageCount_{ 0 };

  /**
   * @var mutable std::recursive_mutex meanWrenchOffsetMutex_
   *
   * @brief The mutex for accessing the mean of the wrench measurements. Used for the "reset service" callback.
   *
   */
  mutable std::recursive_mutex meanWrenchOffsetMutex_;

  /**
   * @var const static uint32_t totalNumberOfWrenchMessages_ = 200;
   *
   * @brief The total number of wrench messages to receive for computing the mean wrench offset. Used for the "reset
   * service" callback.
   *
   */
  const static uint32_t TOTAL_NUMBER_OF_WRENCH_MESSAGES = 200;

  /**
   * @var geometry_msgs::Wrench meanWrenchOffset_
   *
   * @brief The computed mean wrench offset. Used for the "reset service" callback.
   *
   */
  geometry_msgs::Wrench meanWrenchOffset_;
};

template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int8_t& value);
template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int16_t& value);
template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int32_t& value);
template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     int64_t& value);
template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint8_t& value);
template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint16_t& value);
template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint32_t& value);
template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     uint64_t& value);
template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     float& value);
template <>
bool RokubiminiEthercat::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                     double& value);

template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int8_t value);
template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int16_t value);
template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int32_t value);
template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const int64_t value);
template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint8_t value);
template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint16_t value);
template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint32_t value);
template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const uint64_t value);
template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const float value);
template <>
bool RokubiminiEthercat::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                      const double value);

}  // namespace ethercat
}  // namespace rokubimini


if __name__ == "__main__":

    sensor = BotaForceTorqueSensorComm()

    frequence=500
    dt=1/frequence
    rospy.init_node("force_sensor_eth_publisher")
    pub = rospy.Publisher("/force_sensor_eth", WrenchStamped, queue_size=10)
    rate = rospy.Rate(frequence)  # Hz

    rospy.loginfo("Force sensor node started.")
    
    zero_force_buffer=[]
    nb_pour_moyenne_zero_force=20
    axes = ['x', 'y', 'z', 'rx', 'ry', 'rz']



    # plt.ion()
    # fig, ax = plt.subplots()
    # lines = [ax.plot([], [], label=axis)[0] for axis in axes]
    # ax.set_xlim(0, nb_pour_moyenne_zero_force)
    # ax.set_ylim(-10, 10)  # Ajuste selon ton capteur
    # ax.legend()
    # force_history = [[] for _ in range(6)]
    # x_data = []

    rospy.loginfo("Collecte des %d premières mesures pour calcul du offset...", nb_pour_moyenne_zero_force)
    while len(zero_force_buffer) < nb_pour_moyenne_zero_force:
        forces = sensor.read_frame()
        if forces is not None:
            zero_force_buffer.append(forces)


            #tracer
            # for i in range(6):
            #     force_history[i].append(forces[i])
            # x_data.append(len(zero_force_buffer))

            # for i, line in enumerate(lines):
            #     line.set_data(x_data, force_history[i])

            # ax.relim()
            # ax.autoscale_view()
            # plt.pause(0.001)

        rospy.sleep(0.05)  # éviter de saturer le port série

   
        
            
    zero_force = [
        np.mean([f[axis] for f in zero_force_buffer])
        for axis in range(6)
    ]
    rospy.logwarn("Offset calculé sur beaucoup de mesures : %s", zero_force)
    

    try:
        while not rospy.is_shutdown():
            forces = sensor.read_frame()


            if forces:
                msg = WrenchStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "fake_gripper"  # sans doute inutile 
                msg.wrench.force.x = forces[0]-zero_force[0]
                msg.wrench.force.y = forces[1]-zero_force[1]
                msg.wrench.force.z = forces[2]-zero_force[2]
                msg.wrench.torque.x = forces[3]-zero_force[3]
                msg.wrench.torque.y = forces[4]-zero_force[4]
                msg.wrench.torque.z = forces[5]-zero_force[5]
                # rospy.loginfo(f"Forces: {round(forces[0],1)}, {round(forces[1],1)}, {round(forces[2],1)}")
                pub.publish(msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        sensor.close()
        rospy.loginfo("Force sensor node stopped.")