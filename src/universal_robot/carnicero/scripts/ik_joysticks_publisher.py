#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy


class ContinuousJoyRemapper:
    def __init__(self):
        rospy.init_node('IK_joysticks_publisher', anonymous=True)

        self.joy_publisher = rospy.Publisher('/IK_joy_publisher', Joy, queue_size=10)
        rospy.Subscriber('/joy1', Joy, self.joy_callback)

        self.joy_axes = [0.0] * 8
        self.joy_buttons = [0] * 11

        rospy.loginfo("Remapper continu a démarré, s'abonne à /joy1 et publie sur /IK_joy_publisher.")

    def joy_callback(self, joy_msg):
        self.joy_axes = joy_msg.axes
        self.joy_buttons = joy_msg.buttons

    def run(self):
        rate = rospy.Rate(250)
        while not rospy.is_shutdown():
            mapped_joy_msg = Joy()
            mapped_joy_msg.header.stamp = rospy.Time.now()

            velocity_factor = 0.75

            mapped_axes = [0.0] * 6
            mapped_buttons = [0] * 6

            mapped_axes[0] = self.joy_axes[0] * velocity_factor
            mapped_axes[1] = (self.joy_axes[1] - 0.56) * velocity_factor

            mapped_axes[3] = self.joy_axes[4] * velocity_factor
            mapped_axes[4] = self.joy_axes[5] * velocity_factor

            if self.joy_buttons[0] == 1:
                mapped_axes[2] = velocity_factor
            if self.joy_buttons[1] == 1:
                mapped_axes[2] = -velocity_factor

            if self.joy_buttons[2] == 1:
                mapped_axes[5] = velocity_factor
            if self.joy_buttons[3] == 1:
                mapped_axes[5] = -velocity_factor

            if abs(mapped_axes[1]) < 0.2 * velocity_factor:
                mapped_axes[1] = 0.0

            mapped_joy_msg.axes = mapped_axes
            mapped_joy_msg.buttons = mapped_buttons

            self.joy_publisher.publish(mapped_joy_msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        remapper = ContinuousJoyRemapper()
        remapper.run()
    except rospy.ROSInterruptException:
        pass

