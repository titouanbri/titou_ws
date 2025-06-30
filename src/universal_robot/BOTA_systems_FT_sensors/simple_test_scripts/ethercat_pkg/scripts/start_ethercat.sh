#!/bin/bash


source /opt/ros/noetic/setup.bash
source /home/titouan/catkin_ws/devel/setup.bash


rosrun ethercat_pkg publisher_ethercat.py _interface:=enxd037453fd6d2
