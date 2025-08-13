# UR30-admittance

## Connect the robot

* In settings/network/wired/IPv4/manual: set the "address" to the same IP as the robot, changing only the last digit, e.g., 192.168.3.20X with X different from 0, and set the subnet mask to 255.255.255.0

## Steps to configure the PC to work with the UR30 and run the admittance

* Replace “titouan” with username using vscode (in ws), ctrl shift f
* sudo apt update
* sudo apt install ros-noetic-desktop-full (follow a tutorial to install ROS)
* sudo apt install ros-noetic-soem
* pip install --upgrade scipy
* sudo apt install ros-noetic-kdl-parser-py

## Steps for the EtherCAT sensor

* Installation: [https://gitlab.com/botasys/bota\_driver.git](https://gitlab.com/botasys/bota_driver.git) bota\_driver if it doesn’t work otherwise

* sudo apt install ros-noetic-ethercat-grant

* echo "/opt/ros/noetic/lib" | sudo tee /etc/ld.so.conf.d/ros-noetic.conf

* sudo ldconfig

In bota\_driver/rokubi\_ethercat/rokubi\_ethercat.launch:

* Change the IP of the sensor if the driver asks for it
* ChatGPT for errors

### Final command to start acquisition (runs with setup.launch)

* roslaunch rokubimini\_ethercat rokubimini\_ethercat.launch

## Launch

* Command to start UR driver + EtherCAT driver (modify with the correct robot info): roslaunch carnicero setup.launch
* Command to start admittance: rosrun carnicero admittance.py (make sure to wait until both sensors are marked “OK” in the terminal)

## Path to scripts/launch used:

\~/catkin\_ws/src/universal\_robot/carnicero

# Important 

### because of several hardware problems, it could be modification in the scripts depending on which is the last configuration used, if there are problems don't hesitate to contact the autors.

## Email of the GOAT, “UR30 pin crusher”, “UR30 thunderstriker”:

[titouan.briancon@sigma-clermont.fr](mailto:titouan.briancon@sigma-clermont.fr)

## Email of the “demagnetizer”, “Bota crumpler”:

[matteo.proverbio@sigma-clermont.fr](mailto:matteo.proverbio@sigma-clermont.fr)
