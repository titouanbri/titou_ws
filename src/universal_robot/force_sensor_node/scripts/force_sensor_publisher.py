#!/usr/bin/env python3


#publishing of the sensor's data (serial connection)

import rospy
from geometry_msgs.msg import WrenchStamped
import serial
import struct
import time
import numpy as np
import matplotlib.pyplot as plt


HEADER = 0xAA
FRAME_SIZE = 1 + 2 + 6*4 + 4 + 4 + 2

class BotaForceTorqueSensorComm:
    def __init__(self, port="/dev/ttyUSB0", baudrate=460800):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.crc_error_count = 0


    def crc16_x25(self, data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0x8408
                else:
                    crc >>= 1
        return crc ^ 0xFFFF

    def serial_read_bytes(self, n):
        return self.ser.read(n)

    def sync_and_read_frame(self):
        while True:
            b = self.serial_read_bytes(1)
            if not b:
                return None
            if b[0] == HEADER:
                rest = self.serial_read_bytes(FRAME_SIZE - 1)
                if len(rest) == FRAME_SIZE - 1:
                    return b + rest

    def read_frame(self):
        raw = self.sync_and_read_frame()

        if raw is None:
            return None
                
        payload = raw[1:-2]
        crc_received = struct.unpack('<H', raw[-2:])[0]
        crc_calculated = self.crc16_x25(payload)



        if crc_received != crc_calculated:
            return None

        status, = struct.unpack_from('<H', payload, 0)
        forces = struct.unpack_from('<6f', payload, 2)
        return forces if status == 0 else None

    def close(self):
        self.ser.close()


    def send_command(self, cmd: str):
        self.ser.write((cmd + '\n').encode('ascii'))
        self.ser.flush()
        time.sleep(0.05)
        response = self.ser.readline().decode(errors='ignore').strip()
        print(f"[SENSOR] Sent: {cmd} | Response: {response}")


if __name__ == "__main__":

    sensor = BotaForceTorqueSensorComm()



    frequence=500
    dt=1/frequence
    rospy.init_node("force_sensor_publisher")
    pub = rospy.Publisher("/force_sensor", WrenchStamped, queue_size=10)
    rate = rospy.Rate(frequence)  # Hz

    rospy.loginfo("Force sensor node started.")
    
    zero_force_buffer=[]
    nb_pour_moyenne_zero_force=20
    axes = ['x', 'y', 'z', 'rx', 'ry', 'rz']




    rospy.loginfo("Collecte des %d premières mesures pour calcul du offset...", nb_pour_moyenne_zero_force)
    while len(zero_force_buffer) < nb_pour_moyenne_zero_force:
        forces = sensor.read_frame()
        rospy.loginfo(forces)

        if forces is not None:
            zero_force_buffer.append(forces)



        rospy.sleep(0.05)  # éviter de saturer le port série

   
        
            
    zero_force = [
        np.mean([f[axis] for f in zero_force_buffer])
        for axis in range(6)
    ]
    rospy.logwarn("serial sensor OK")
    

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