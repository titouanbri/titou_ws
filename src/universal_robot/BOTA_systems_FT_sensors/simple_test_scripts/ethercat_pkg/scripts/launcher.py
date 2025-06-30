#!/usr/bin/env python3
import subprocess
import sys
import rospy

if __name__ == "__main__":
    rospy.init_node("force_sensor_wrapper", anonymous=True)
    interface = rospy.get_param("~interface", "enxd037453fd6d2")
    cmd = ["bash", "/home/titouan/catkin_ws/src/ethercat_pkg/scripts/start_ethercat.sh", "_interface:=" + interface]
    subprocess.call(cmd)

