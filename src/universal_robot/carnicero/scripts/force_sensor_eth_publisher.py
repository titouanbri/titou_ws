#!/usr/bin/env python3

#publisher for the rokubi ethercat sensor 
#to works, the sensor driver have to be launch before, with : roslaunch rokubi_ethercat rokubi_ethercat.launch
#this publisher take the data from the driver, and publishes them in a new topic after removing the offset

import rospy
from geometry_msgs.msg import WrenchStamped
import numpy as np



class ForceSensorProcessor:
    def __init__(self):
        rospy.init_node("force_sensor_eth_publisher")

        self.nb_pour_moyenne_zero_force = 20
        self.zero_force_buffer = []
        self.zero_force = None

        self.pub = rospy.Publisher("/force_sensor_eth", WrenchStamped, queue_size=10)
        self.sub = rospy.Subscriber("/bus0/ft_sensor0/ft_sensor_readings/wrench", WrenchStamped, self.callback)

        rospy.loginfo("ForceSensorProcessor node started. Waiting for %d initial readings to compute offset...", self.nb_pour_moyenne_zero_force)

    def callback(self, msg: WrenchStamped):
        # Convert message to list of forces/torques
        raw_data = [
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z
        ]

        if self.zero_force is None:
            self.zero_force_buffer.append(raw_data)

            if len(self.zero_force_buffer) >= self.nb_pour_moyenne_zero_force:
                self.zero_force = np.mean(self.zero_force_buffer, axis=0)
                rospy.logwarn("ethercat sensor OK")
            return  

        corrected_data = [raw_data[i] - self.zero_force[i] for i in range(6)]

        corrected_msg = WrenchStamped()
        corrected_msg.header.stamp = rospy.Time.now()
        corrected_msg.header.frame_id = msg.header.frame_id  
        corrected_msg.wrench.force.x = corrected_data[0]
        corrected_msg.wrench.force.y = corrected_data[1]
        corrected_msg.wrench.force.z = corrected_data[2]
        corrected_msg.wrench.torque.x = corrected_data[3]
        corrected_msg.wrench.torque.y = corrected_data[4]
        corrected_msg.wrench.torque.z = corrected_data[5]

        self.pub.publish(corrected_msg)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
  try:
      processor = ForceSensorProcessor()
      processor.spin()
  except rospy.ROSInterruptException:
      pass