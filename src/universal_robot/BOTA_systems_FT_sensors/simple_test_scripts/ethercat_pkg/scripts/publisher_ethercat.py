#!/usr/bin/env python3

import rospy
import struct
import ctypes
import collections
import pysoem
from geometry_msgs.msg import WrenchStamped

class ForceSensorNode:

    BOTA_VENDOR_ID = 0xB07A
    BOTA_PRODUCT_CODE = 0x00000001
    SINC_LENGTH = 256

    def __init__(self, ifname):
        self._ifname = ifname
        self._master = pysoem.Master()
        SlaveSet = collections.namedtuple('SlaveSet', 'slave_name product_code config_func')
        self._expected_slave_mapping = {
            0: SlaveSet('BFT-MEDS-ECAT-M8', self.BOTA_PRODUCT_CODE, self.bota_sensor_setup)
        }
        self.time_step = 1.0
        self.pub = rospy.Publisher("/force_sensor_eth", WrenchStamped, queue_size=10)

    def bota_sensor_setup(self, slave_pos):
        rospy.loginfo("Configuring Bota sensor...")
        slave = self._master.slaves[slave_pos]
        slave.sdo_write(0x8010, 1, bytes(ctypes.c_uint8(1)))
        slave.sdo_write(0x8010, 2, bytes(ctypes.c_uint8(0)))
        slave.sdo_write(0x8010, 3, bytes(ctypes.c_uint8(1)))
        slave.sdo_write(0x8006, 2, bytes(ctypes.c_uint8(1)))
        slave.sdo_write(0x8006, 3, bytes(ctypes.c_uint8(0)))
        slave.sdo_write(0x8006, 4, bytes(ctypes.c_uint8(0)))
        slave.sdo_write(0x8006, 1, bytes(ctypes.c_uint16(self.SINC_LENGTH)))

        sampling_rate = struct.unpack('h', slave.sdo_read(0x8011, 0))[0]
        if sampling_rate > 0:
            self.time_step = 1.0 / float(sampling_rate)
        rospy.loginfo(f"Sensor configured. Sampling rate: {sampling_rate} Hz, Time step: {self.time_step}s")

    def run(self):
        self._master.open(self._ifname)

        if self._master.config_init() > 0:
            rospy.loginfo(f"{len(self._master.slaves)} slaves found and configured.")
            for i, slave in enumerate(self._master.slaves):
                assert slave.man == self.BOTA_VENDOR_ID
                assert slave.id == self._expected_slave_mapping[i].product_code
                slave.config_func = self._expected_slave_mapping[i].config_func

            self._master.config_map()
            self._master.state_check(pysoem.SAFEOP_STATE, 50000)
            self._master.state = pysoem.OP_STATE
            self._master.write_state()
            self._master.state_check(pysoem.OP_STATE, 50000)

            if self._master.state != pysoem.OP_STATE:
                raise Exception("Slaves did not reach OP state")

            rospy.loginfo("Starting EtherCAT read loop...")
            rate = rospy.Rate(1.0 / self.time_step)
            while not rospy.is_shutdown():
                self._master.send_processdata()
                self._master.receive_processdata(2000)

                sensor_input = self._master.slaves[0].input

                Fx = struct.unpack_from('f', sensor_input, 5)[0]
                Fy = struct.unpack_from('f', sensor_input, 9)[0]
                Fz = struct.unpack_from('f', sensor_input, 13)[0]
                Mx = struct.unpack_from('f', sensor_input, 17)[0]
                My = struct.unpack_from('f', sensor_input, 21)[0]
                Mz = struct.unpack_from('f', sensor_input, 25)[0]

                msg = WrenchStamped()
                msg.header.stamp = rospy.Time.now()
                msg.wrench.force.x = Fx
                msg.wrench.force.y = Fy
                msg.wrench.force.z = Fz
                msg.wrench.torque.x = Mx
                msg.wrench.torque.y = My
                msg.wrench.torque.z = Mz

                self.pub.publish(msg)
                rate.sleep()

        else:
            rospy.logerr("No EtherCAT slaves found.")
            self._master.close()

        self._master.state = pysoem.INIT_STATE
        self._master.write_state()
        self._master.close()


if __name__ == '__main__':
    rospy.init_node("force_sensor_node")
    ifname = rospy.get_param("~interface", "enxd037453fd6d2")  # Peut être override avec _interface:=ethX
    try:
        node = ForceSensorNode(ifname)
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Exception: {e}")
