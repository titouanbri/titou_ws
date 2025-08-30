#!/usr/bin/env python
"""Publishes constant elbow and wrist positions for testing.

This node simulates the topics normally provided by another PC by
publishing `geometry_msgs/PointStamped` messages with fixed coordinates
on `/right_arm/elbow` and `/right_arm/wrist`.
"""
import rospy
from geometry_msgs.msg import PointStamped


def main():
    rospy.init_node("right_arm_constant_publisher")
    elbow_pub = rospy.Publisher("/right_arm/elbow", PointStamped, queue_size=1)
    wrist_pub = rospy.Publisher("/right_arm/wrist", PointStamped, queue_size=1)

    rate = rospy.Rate(50)  # Hz

    elbow_msg = PointStamped()
    wrist_msg = PointStamped()
    elbow_msg.header.frame_id = "base_link"
    wrist_msg.header.frame_id = "base_link"

    # Constant positions expressed in base_link
    elbow_msg.point.x = 0.4
    elbow_msg.point.y = 0.0
    elbow_msg.point.z = 0.3

    wrist_msg.point.x = 0.6
    wrist_msg.point.y = 0.0
    wrist_msg.point.z = 0.2

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        elbow_msg.header.stamp = now
        wrist_msg.header.stamp = now
        elbow_pub.publish(elbow_msg)
        wrist_pub.publish(wrist_msg)
        rate.sleep()


if __name__ == "__main__":
    main()
