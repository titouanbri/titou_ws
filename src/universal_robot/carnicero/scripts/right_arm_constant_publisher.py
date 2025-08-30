#!/usr/bin/env python
"""Publishes sinusoidal elbow and wrist positions for testing.

This node simulates the topics normally provided by another PC by
publishing `geometry_msgs/PointStamped` messages with coordinates that vary
sinusoidally on `/right_arm/elbow` and `/right_arm/wrist`.
"""
import math
import rospy
from geometry_msgs.msg import Point, PointStamped


def main():
    rospy.init_node("right_arm_constant_publisher")
    elbow_pub = rospy.Publisher("/right_arm/elbow", PointStamped, queue_size=1)
    wrist_pub = rospy.Publisher("/right_arm/wrist", PointStamped, queue_size=1)

    rate = rospy.Rate(50)  # Hz

    elbow_msg = PointStamped()
    wrist_msg = PointStamped()
    elbow_msg.header.frame_id = "base_link"
    wrist_msg.header.frame_id = "base_link"

    # Parameters for sinusoidal motion
    elbow_base = Point(0.4, 0.0, 0.3)
    wrist_base = Point(0.6, 0.0, 0.2)

    amplitude = 0.1
    frequency = 0.5  # Hz

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        t = now.to_sec()

        elbow_msg.header.stamp = now
        wrist_msg.header.stamp = now

        # Update elbow and wrist positions with sinusoidal variations
        elbow_msg.point.x = elbow_base.x + amplitude * math.sin(2 * math.pi * frequency * t)
        elbow_msg.point.y = elbow_base.y
        elbow_msg.point.z = elbow_base.z + amplitude * math.cos(2 * math.pi * frequency * t)

        wrist_msg.point.x = wrist_base.x + amplitude * math.sin(2 * math.pi * frequency * t)
        wrist_msg.point.y = wrist_base.y
        wrist_msg.point.z = wrist_base.z + amplitude * math.cos(2 * math.pi * frequency * t)

        elbow_pub.publish(elbow_msg)
        wrist_pub.publish(wrist_msg)
        rate.sleep()


if __name__ == "__main__":
    main()
