#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import WrenchStamped

class ForceController:
    def __init__(self):
        self.force_sub = rospy.Subscriber("/wrench", WrenchStamped, self.force_cb)
        self.twist_pub = rospy.Publisher("/servo_server/delta_twist_cmds", TwistStamped, queue_size=1)
        self.zero_force = 0.0
        self.threshold = 0.5
        self.gain = 0.002

    def force_cb(self, msg):
        force_y = msg.wrench.force.y
        error = force_y - self.zero_force

        if abs(error) > self.threshold:
            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            twist.twist.linear.x = self.gain * error
            self.twist_pub.publish(twist)

if __name__ == "__main__":
    rospy.init_node("force_based_controller")
    fc = ForceController()
    rospy.spin()

