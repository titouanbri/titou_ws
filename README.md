# UR30-admittance

## Connect the robot

* In settings/network/wired/IPv4/manual: set the "address" to the same IP as the robot, changing only the last digit, e.g., 192.168.3.20X with X different from 0, and set the subnet mask to 255.255.255.0

## Steps to configure the PC to work with the UR30 and run the admittance

* follow this tuto and install the full version of ros : https://www.youtube.com/watch?v=Qk4vLFhvfbI&list=PLLSegLrePWgIbIrA4iehUQ-impvIXdd9Q
```
sudo apt update
```

```
rosdep init
rosdep update
cd titou_ws/ #or the name you gave to the ws
rosdep install --from-paths src --ignore-src -r -y
```
```
sudo apt install -y \
  ros-noetic-trac-ik-kinematics-plugin \
  ros-noetic-realtime-tools \
  ros-noetic-controller-manager \
  ros-noetic-robot-state-publisher \
  ros-noetic-tf-conversions \
  ros-noetic-joint-trajectory-controller \
  ros-noetic-soem \
  ros-noetic-ur-client-library \
  ros-noetic-ethercat-grant \
  ros-noetic-soem \
  ros-noetic-kdl-parser-py \
  python3-pip \
  ros-noetic-industrial-msgs \
  ros-noetic-xacro \
  ros-noetic-moveit \
<<<<<<< HEAD
  ros-noetic-joint-state-publisher
=======
  ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-gazebo-plugins
>>>>>>> 917a03b4c5aa4b3a071230e44b697b7771c40ba5
```

```
pip install --upgrade scipy
```
* Replace “titouan” with username using vscode (in ws), ctrl shift f
## Steps for the EtherCAT sensor

* Installation: [https://gitlab.com/botasys/bota\_driver.git](https://gitlab.com/botasys/bota_driver.git) bota\_driver if it doesn’t work otherwise
```
echo "/opt/ros/noetic/lib" | sudo tee /etc/ld.so.conf.d/ros-noetic.conf
```
```
sudo ldconfig
```

In bota\_driver/rokubi\_ethercat/rokubi\_ethercat.launch:

* Change the IP of the sensor if the driver asks for it
* ChatGPT for errors

### Final command to start acquisition of the ethercat sensor (runs with setup.launch)
```
roslaunch rokubimini\_ethercat rokubimini\_ethercat.launch
```
## Launch

* Command to start UR driver + EtherCAT driver (modify with the correct robot info): 
```
roslaunch carnicero setup.launch
```
In both node there is an offset problem due to the default sensor offset, the force sensor publisher make a "fake" calibration, so the robot could drift. If the robot drifts, you have to restart the node. 
* Command to start admittance: (make sure to wait until both sensors are marked “OK” in the terminal) (slower with 300/400 Hz)
```
rosrun carnicero admittance.py 
```

* there is also the cpp version : 
```
rosrun carnicero admittance_control_node   (faster with more than 3000 Hz, but the UR robot driver can handle 500 Hz maximum)
```
## Path to scripts/launch used:

\~/catkin\_ws/src/universal\_robot/carnicero

There are publisher for the rokubi serial and ethercat, and the admittance code. there are also codes for data acquisition and for the Onrobot ethercat sensor. "force_sensor_eth_publisher.py" is the only one to use the rokubi ethercat sensor.


# Important 
Because of several hardware problems, There might be changes in the script depending on which is the last configuration used, if there are problems don't hesitate to contact the autors.

## Email of the GOAT, “UR30 pin crusher”, “UR30 thunderstriker”:

[titouan.briancon@sigma-clermont.fr](mailto:titouan.briancon@sigma-clermont.fr)

## Email of the “demagnetizer”, “Bota crumpler”:

[matteo.proverbio@sigma-clermont.fr](mailto:matteo.proverbio@sigma-clermont.fr)
