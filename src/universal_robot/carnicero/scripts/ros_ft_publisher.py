#!/usr/bin/env python
"""
ROS1 node to publish EtherCAT force/torque sensor data on a ROS topic.

This node reads the 6-axis sensor values via Etherlab CLI (ethercat upload)
and publishes them as geometry_msgs/WrenchStamped on the 'ft_sensor' topic.

Requirements:
- ROS1 (rospy, geometry_msgs)
- Etherlab CLI installed and configured
- Node must be run with sufficient privileges to call 'ethercat upload'
  (e.g., run with sudo or grant ethercat CLI permission via sudoers)
"""

import rospy
from geometry_msgs.msg import WrenchStamped
import subprocess

# Configuration
SLAVE_POS = 0
PUBLISH_RATE_HZ = 10  # publishing frequency in Hz
USE_REAL = False      # True to read REAL32 (0x6000), False for Int+Divider (0x6010)

# EtherCAT object indexes
IDX_REAL = 0x6000
IDX_INT  = 0x6010
REAL_SUB = range(1, 7)  # subindexes 1..6 for Fx, Fy, Fz, Mx, My, Mz
INT_SUB  = range(1, 7)

def ethercat_upload(index, subindex, data_type):
    """Invoke ethercat upload and return parsed value."""
    cmd = [
        "ethercat", "upload",
        "-p", str(SLAVE_POS),
        "--type", data_type,
        hex(index), hex(subindex)
    ]
    result = subprocess.run(cmd,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE,
                            text=True)
    if result.returncode != 0:
        rospy.logerr(f"EtherCAT upload error: {result.stderr.strip()}")
        raise RuntimeError(result.stderr.strip())
    # Output may be e.g. "0xFFFF... -27700" or just "-2.86"
    parts = result.stdout.strip().split()
    return parts[-1]

def read_divider():
    """Read uint16 divider from subindex 7 of 0x6010."""
    val = ethercat_upload(IDX_INT, 7, "uint16")
    return int(val, 0)

def read_ft():
    """Return tuple (Fx, Fy, Fz, Mx, My, Mz) as floats."""
    if USE_REAL:
        # Direct float readings
        vals = [float(ethercat_upload(IDX_REAL, si, "float")) for si in REAL_SUB]
    else:
        # Integer readings + divider
        divider = read_divider()
        raw = [int(ethercat_upload(IDX_INT, si, "int32"), 0) for si in INT_SUB]
        vals = [r / float(divider) for r in raw]
    return vals

def main():
    rospy.init_node('ft_sensor_publisher')
    pub = rospy.Publisher('ft_sensor', WrenchStamped, queue_size=10)
    rate = rospy.Rate(PUBLISH_RATE_HZ)
    rospy.loginfo("FT sensor publisher started (USE_REAL=%s)", USE_REAL)

    while not rospy.is_shutdown():
        try:
            Fx, Fy, Fz, Mx, My, Mz = read_ft()
        except Exception as e:
            rospy.logwarn("Failed to read FT data: %s", e)
            rate.sleep()
            continue

        msg = WrenchStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "ft_sensor_frame"
        msg.wrench.force.x  = Fx
        msg.wrench.force.y  = Fy
        msg.wrench.force.z  = Fz
        msg.wrench.torque.x = Mx
        msg.wrench.torque.y = My
        msg.wrench.torque.z = Mz

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()
