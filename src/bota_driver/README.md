# Rokubi mini 2.0 - Force-Torque Sensor - README

## Overview

This software package will provide a driver and a ROS interface for the ethercat and serial version of the rokubi mini force-torque sensor.
This is at the moment just a skeleton to connect to rokubi mini devices and support the driver development of its firmware.

**Authors(s):** Ilias Patsiaouras, Mike Karamousadakis

## Building

[![pipeline status](https://gitlab.com/botasys/bota_driver/badges/master/pipeline.svg)](https://gitlab.com/botasys/bota_driver/-/commits/master)

## Installation

### Building from Source

In order to use the `bota_driver` package, you need to download first the following dependencies:

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [soem](https://github.com/mgruhler/soem)
- [ethercat_grant](https://github.com/shadow-robot/ethercat_grant)

Before building, you need to make sure that all the binary dependencies are installed. To do so, run in a terminal:

```bash
cd catkin_workspace/ && rosdep update && rosdep install --from-path src --ignore-src -y -r
```

#### Building

To build the bota_driver from source, clone the latest version from this repository into your catkin workspace and compile the package using:

1. The `catkin_tools` package:

	```bash
	cd catkin_workspace/src
	git clone https://gitlab.com/botasys/bota_driver.git
	cd ../
	catkin build bota_driver
	```

2. If you don't have the `catkin_tools` package, you can still build from source with `catkin_make`:

	```bash
	cd catkin_workspace/src
	git clone https://gitlab.com/botasys/bota_driver.git
	cd ../
	catkin_make --only-pkg-with-deps bota_driver
	# Don't forget to switch back to building all packages when you are done:
	# catkin_make -DCATKIN_WHITELIST_PACKAGES=""
	```

3. Finally, you can build from source with `catkin_make_isolated`. The only difference is that each package will be processed sequentially:

	```bash
	cd catkin_workspace/src
	git clone https://gitlab.com/botasys/bota_driver.git
	cd ../
	catkin_make_isolated --pkg bota_driver
	```

### Unit Tests

No unit tests so far.


## Packages

#### rokubimini

The core C++ library to interface one or multiple rokubimini devices. Contains abstract interfaces to start the communication.

#### rokubimini_ethercat

The ethercat implementation of rokubimini.

#### rokubimini_serial

The serial implementation of rokubimini.

#### rokubimini_bus_manager

An abstract class for managing a bus.

#### bota_node

ROS node wrapper with some convenience functions using *bota_worker*.

#### bota_worker

High resolution and threaded version of the ROS rate class.

#### bota_signal_handler

Contains a static signal handling helper class.

#### rokubimini_msgs

Contains the definitions of the ROS messages and services used for communication over ROS.

## Usage

### Launching

#### Serial

If no EtherCAT device is present, the `roslaunch` commands with only serial devices works without `root` privileges provided that the user is in the `dialout` group. This can be done with:

```bash
sudo usermod -a -G dialout username
```
Please log off and log in again for the changes to take effect!

To run the sensor you can use the following command:
```bash
  roslaunch rokubimini_serial rokubimini_serial.launch
```

#### EtherCAT
The EtherCAT device driver is using SOEM which requires access to certain network capabilities as it is using raw sockets, and as such any executable linking
against SOEM needs to be run with certain privileges.
Typically, you would run any SOEM executables with `sudo` or as `root`.
This is impractical for any ROS system, and as such there exists a tool called
[`ethercat_grant`](https://github.com/shadow-robot/ethercat_grant) that helps with that.

If you followed the installation instruction above `ethercat_grant` is already installed. Alternatively, you can install it with
```bash
sudo apt install ros-<DISTRO>-ethercat-grant
```
and add the following `launch prefix` to the `node` tag of the `rokubimini_ethercat_bus_manager_node` in your launchfile
```xml
launch-prefix="ethercat_grant
```
**Note:** This launch prefix is already added to the default launch file and you can run the driver of an EtherCAT device with:
```bash
  roslaunch rokubimini_ethercat rokubimini_ethercat.launch
```

##### (Alternative:) EtherCAT without launch prefix
Instead of using the `ethercat_grant` launch prefix, the above command to run the EtherCAT device driver can also be combined with escalated privileges (e.g. `su root` or `sudo su`, if you don't have set a `root` password).

### Extra `launch` Arguments

#### Niceness
If you want to change the niceness of the node, consider adding the following XML element as `launch_prefix`:
```xml
	<arg name="launch_prefix" default="nice -n -10" />
	...
	<node name="bus0" pkg="rokubimini_serial" type="rokubimini_serial_bus_manager_node"  output="screen" launch-prefix="$(arg launch_prefix)" required="true">
```
**Note:** If you are running the launch file as a **normal** user, you need to append to your `/etc/security/limits.conf` file the following line (change `username` to your username):
```bash
username - nice -20
```
Please log off and log in again for the changes to take effect!

#### GDB
For using a `launch` file with `gdb`, you need the following XML element:
```xml
  <arg name="launch_prefix_gdb" default="gdb -ex run --args" />
```
### Configuration

In order to run your existing setup, you should modify accordingly the launch file that matches your case (`serial` or `ethercat`  - found in `rokubimini_[serial|ethercat]/launch` directory). In these `launch` files, there are two kinds of parameters; parameters for each rokubimini and parameters for the bus.

The parameters for each rokubimini are the following:
- name: The name of the rokubimini devices. Needed to handle multiple devices on the master side,
- configuration_file: The relative path to the configuration file of the sensor
- product_name: The product name of each device.
- ethercat_address (**EtherCAT only**): The address of the EtherCAT device. EtherCAT addresses are distributed to each slave incrementally. The slave closest to the master has address 1, the second one 2 etc.

The parameters for each bus are the following:
- For the `ethercat` bus:
  - ethercat_bus: The ethercat bus containing the sensor. Is the ethernet adapter name on which the sensor is connected to, as indicated in the `ifconfig` command output.
- For the `serial` bus:
  - port: The serial port to connect to communicate with the serial sensor.
  - baud_rate: The baud rate used for the serial communication.

For configuring the sensor parameters, the following parameters can be set in a `rokubimini_sensor.yaml` file (and set in the `configuration_file` parameter of the `launch` file):

- set_reading_to_nan_on_disconnect: Sets the reading to nan when the sensor disconnects
- imu_acceleration_range: 0 = ±2g, 1 = ±4g, 2 = ±8g, 3 = ±16g
- imu_angular_rate_range: 0 = ±250°/s, 1 = ±500°/s, 2 = ±1000°/s, 3 = ±2000°/s
- imu_acceleration_filter: (cut-off Freq) 1 = 460Hz, 2 = 184Hz, 3 = 92Hz, 4 = 41Hz, 5 = 21Hz, 6 = 10Hz, 7 = 5Hz
- imu_angular_rate_filter: (cut-off Freq) 3 = 184Hz, 4 = 92Hz, 5 = 41Hz, 6 = 21Hz, 7 = 10Hz, 8 = 5Hz
- sinc_filter_size: (cut-off Freq high/low@sampling Freq) 51 = 1674/252Hz@1000Hz, 64 = 1255/189Hz@800Hz, 128 = 628/94.5hz@400Hz, 205 = 393/59.5Hz@250Hz 256 = 314/47.5Hz@200Hz, 512 = 157/23.5@100Hz
- fir_disable: false = low cut-off frequency, true = high cut-off frequency from the above result e.g. for sinc filter_size: 48 and fir_disable: 1 you get cut-off freq 1674Hz@1000Hz sample rate
- chop_enable: should be always false
- fast_enable: (only applies if fir_disable is false) True = will result in low cut-off frequency but would be still able to catch step impulses of high cut-off frequency
- calibration_matrix_active: Use the calibration matrix to compute sensor output
- temperature_compensation_active: Compensate drift due to temperature !not supported yet!
- imu_active: Chooses which IMU type is active: 0 = no imu active, 1 = internal IMU active, 2 = external IMU active (if available), 3 = both IMUs active
- coordinate_system_active: Set a user defined coordinate system
- inertia_compensation_active: Enables compensation due to inertia effect
- orientation_estimation_active: Enables orientation estimation and outputs a quaternion

### Subscribed Topics

None

### Published Topics

* **`/<bus_name>/<rokubimini_name>/ft_sensor_readings/reading`** (`rokubimini_msgs/Reading`)
* **`/<bus_name>/<rokubimini_name>/ft_sensor_readings/imu`** ([`sensor_msgs/Imu`](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html))
* **`/<bus_name>/<rokubimini_name>/ft_sensor_readings/wrench`** ([`geometry_msgs/WrenchStamped`](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/WrenchStamped.html))
* **`/<bus_name>/<rokubimini_name>/ft_sensor_readings/temperature`** ([`sensor_msgs/Temperautre`](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Temperature.html))

### Services

- For Serial devices:
  * **`/<bus_name>/<rokubimini_name>/firmware_update`** (`rokubimini_msgs/FirmwareUpdateSerial`). Arguments:
    * `file_path`: The path of the firmware update file.
  * **`/<bus_name>/<rokubimini_name>/reset_wrench`** (`rokubimini_msgs/ResetWrench`). Arguments:
    * `desired_wrench`: The desired wrench to set.
- For EtherCAT devices:
  * **`/<bus_name>/<rokubimini_name>/firmware_update`** (`rokubimini_msgs/FirmwareUpdateEthercat`). Arguments:
    * `file_name`: The name of the firmware file.
    * `file_path`: The path of the firmware update file.
	* `password`: The password for authorization.
  * **`/<bus_name>/<rokubimini_name>/reset_wrench`** (`rokubimini_msgs/ResetWrench`). Arguments:
    * `desired_wrench`: The desired wrench to set.

**Notes:**
- The **`<bus_name>`** is the `name` attribute in the `node` element, that is set in the launch file.
- The **`<rokubimini_name>`** is the configuration parameter `ft_sensor_x_name` that is set in the launch file.
## Support

For any queries or problems found with the software provided, please contact us at `sw-support@botasys.com`
