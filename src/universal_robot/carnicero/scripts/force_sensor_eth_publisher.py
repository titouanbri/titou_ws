#!/usr/bin/env python3


#pragma once

# include <rokubimini/Rokubimini.hpp>
# include <rokubimini_ethercat/RokubiminiEthercatSlave.hpp>
# include <utility>
# include <rokubimini_msgs/FirmwareUpdateEthercat.h>
# include <rokubimini_msgs/ResetWrench.h>
import rospy
from geometry_msgs.msg import WrenchStamped
import numpy as np


# if __name__ == "__main__":

#     sensor = BotaForceTorqueSensorComm()

#     frequence=500
#     dt=1/frequence
#     rospy.init_node("force_sensor_eth_publisher")
#     pub = rospy.Publisher("/force_sensor_eth", WrenchStamped, queue_size=10)
#     rate = rospy.Rate(frequence)  # Hz

#     rospy.loginfo("Force sensor node started.")
    
#     zero_force_buffer=[]
#     nb_pour_moyenne_zero_force=20
#     axes = ['x', 'y', 'z', 'rx', 'ry', 'rz']



#     # plt.ion()
#     # fig, ax = plt.subplots()
#     # lines = [ax.plot([], [], label=axis)[0] for axis in axes]
#     # ax.set_xlim(0, nb_pour_moyenne_zero_force)
#     # ax.set_ylim(-10, 10)  # Ajuste selon ton capteur
#     # ax.legend()
#     # force_history = [[] for _ in range(6)]
#     # x_data = []

#     rospy.loginfo("Collecte des %d premières mesures pour calcul du offset...", nb_pour_moyenne_zero_force)
#     while len(zero_force_buffer) < nb_pour_moyenne_zero_force:
#         forces = sensor.read_frame()
#         if forces is not None:
#             zero_force_buffer.append(forces)


#             #tracer
#             # for i in range(6):
#             #     force_history[i].append(forces[i])
#             # x_data.append(len(zero_force_buffer))

#             # for i, line in enumerate(lines):
#             #     line.set_data(x_data, force_history[i])

#             # ax.relim()
#             # ax.autoscale_view()
#             # plt.pause(0.001)

#         rospy.sleep(0.05)  # éviter de saturer le port série

   
        
            
#     zero_force = [
#         np.mean([f[axis] for f in zero_force_buffer])
#         for axis in range(6)
#     ]
#     rospy.logwarn("Offset calculé sur beaucoup de mesures : %s", zero_force)
    

#     try:
#         while not rospy.is_shutdown():
#             forces = sensor.read_frame()


#             if forces:
#                 msg = WrenchStamped()
#                 msg.header.stamp = rospy.Time.now()
#                 msg.header.frame_id = "fake_gripper"  # sans doute inutile 
#                 msg.wrench.force.x = forces[0]-zero_force[0]
#                 msg.wrench.force.y = forces[1]-zero_force[1]
#                 msg.wrench.force.z = forces[2]-zero_force[2]
#                 msg.wrench.torque.x = forces[3]-zero_force[3]
#                 msg.wrench.torque.y = forces[4]-zero_force[4]
#                 msg.wrench.torque.z = forces[5]-zero_force[5]
#                 # rospy.loginfo(f"Forces: {round(forces[0],1)}, {round(forces[1],1)}, {round(forces[2],1)}")
#                 pub.publish(msg)
#             rate.sleep()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         sensor.close()
#         rospy.loginfo("Force sensor node stopped.")

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
            return  # Ne publie rien avant le calcul du zéro

        # Soustraction de l'offset
        corrected_data = [raw_data[i] - self.zero_force[i] for i in range(6)]

        # Création et publication du message corrigé
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