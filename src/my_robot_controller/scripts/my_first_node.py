#!/usr/bin/env python3
import rospy

if __name__=='__main__':
    rospy.init_node("test_node")
    rospy.loginfo("test node kingkong")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo("hello")
        rate.sleep()
