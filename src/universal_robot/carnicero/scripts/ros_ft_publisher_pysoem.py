#!/usr/bin/env python
"""
ROS1 Node: Read EtherCAT sensor data and publish to a ROS topic.

This script uses the pysoem library to interface with an EtherCAT bus, reads input data from a specified slave and maps it to a ROS message.
Includes debug logs to inspect slave mapping and raw PDO bytes.

Usage:
  Place in your package's scripts/, make executable, then:
    rosrun <your_package> sensor_to_ros.py __name:=sensor_publisher

Parameters (rosparam or remapped via command line):
  interface     : EtherCAT network interface (string, default: "eth0")
  slave_position: Position/index of EtherCAT slave in the chain (int, default: 0)
  byte_offset   : Starting byte offset in the input PDO (int, default: 0)
  num_values    : Number of data values to read (int, default: 6)
  data_type     : Data format, either "float32" or "int16" (string, default: "int16")
  scale         : Scale factor to apply to raw values (float, default: 1.0)
  topic_name    : ROS topic to publish (string, default: "/sensor_data")
  publish_rate  : Rate in Hz for reading and publishing (float, default: 10.0)

Topic Type:
  std_msgs/Float32MultiArray
"""
import rospy
from std_msgs.msg import Float32MultiArray
import pysoem
import struct


def main():
    # Initialize ROS node
    rospy.init_node('sensor_publisher', anonymous=False)

    # ROS parameters
    iface         = rospy.get_param('~interface', 'eth0')
    slave_pos     = rospy.get_param('~slave_position', 0)
    byte_offset   = rospy.get_param('~byte_offset', 0)
    num_values    = rospy.get_param('~num_values', 6)
    data_type     = rospy.get_param('~data_type', 'int16')
    scale         = rospy.get_param('~scale', 1.0)
    topic_name    = rospy.get_param('~topic_name', '/sensor_data')
    rate_hz       = rospy.get_param('~publish_rate', 10.0)

    # Publisher
    pub = rospy.Publisher(topic_name, Float32MultiArray, queue_size=10)
    rate = rospy.Rate(rate_hz)

    # EtherCAT master setup
    master = pysoem.Master()
    rospy.loginfo("Opening EtherCAT interface '%s'", iface)
    try:
        master.open(iface)
    except Exception as e:
        rospy.logerr("Failed to open interface %s: %s", iface, e)
        return

    # Discover and configure slaves
    count = master.config_init()
    if count <= 0:
        rospy.logerr("No EtherCAT slaves found!")
        return
    rospy.loginfo("%d EtherCAT slave(s) found", count)

    # Map PDOs
    master.config_map()

    master.activate()
    # puis un cycle pour laisser le noyau prendre en compte la config
    master.send_processdata()
    master.receive_processdata(timeout=2e6)


    # Debug: list slave mapping after config_map
    for idx, slave in enumerate(master.slaves):
        rospy.loginfo("Slave %d: name='%s', input PDO length=%d bytes, output PDO length=%d bytes",
                      idx, getattr(slave, 'name', 'N/A'), len(slave.input), len(slave.output))

    # Set slaves to OPERATIONAL
    master.state = pysoem.OP_STATE
    master.write_state()
    master.send_processdata()
    master.receive_processdata(timeout=2e6)
    if not master.state_check(pysoem.OP_STATE, timeout=2e6):
        rospy.logerr("Failed to set slaves to OPERATIONAL")
        return
    rospy.loginfo("Slaves in OPERATIONAL state, starting data loop...")

    # — send the “Start F/T Data Output” command (0x0B) once before reading
    slave = master.slaves[slave_pos]
    slave.output = b'\x0B\x00\x00\x00'
    master.send_processdata()
    master.receive_processdata(timeout=2e6)
    # rospy.loginfo("Sent Start‐Output command to sensor")


    prev_raw = None
    # Main loop: read and publish
    while not rospy.is_shutdown():

            # ————— on redonne à chaque cycle la commande “Start‐FT” —————
        slave = master.slaves[slave_pos]
        slave.output = b'\x0B\x00\x00\x00'
        master.send_processdata()
        master.receive_processdata(timeout=2e6)

        try:
            slave = master.slaves[slave_pos]
            
            data = slave.input  # bytearray of input PDO

            # Debug: raw data hex and change detection
            raw_hex = data.hex()
            if prev_raw is None or raw_hex != prev_raw:
                rospy.loginfo("Raw PDO data (hex): %s", raw_hex)
                prev_raw = raw_hex
            else:
                rospy.logdebug("Raw PDO data unchanged")

            # Unpack values
            values = []
            if data_type == 'float32':
                step, fmt = 4, '<f'
            elif data_type == 'int16':
                step, fmt = 2, '<h'
            else:
                rospy.logerr("Unsupported data_type '%s'", data_type)
                break

            for i in range(byte_offset, byte_offset + num_values*step, step):
                if i + step <= len(data):
                    raw = struct.unpack(fmt, bytes(data[i:i+step]))[0]
                    values.append(raw * scale)
                else:
                    rospy.logwarn("Index %d out of range for PDO length %d", i, len(data))
                    break

            # Publish data
            msg = Float32MultiArray()
            msg.data = values
            pub.publish(msg)
        except IndexError:
            rospy.logwarn("Slave position %d out of range", slave_pos)
        except Exception as e:
            rospy.logwarn("Error during read/publish: %s", e)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
