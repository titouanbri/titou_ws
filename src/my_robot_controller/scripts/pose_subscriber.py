#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
def pose_callback(msg: Pose):
    rospy.loginfo("(" + str(msg.x) + ")")

if __name__=='__main__':
    rospy.init_node("turtle_node_subscriber")
    sub=rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
    rospy.loginfo("node ouioui")
    rospy.spin()