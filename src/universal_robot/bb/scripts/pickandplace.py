#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
from ur_msgs.srv import SetIO  # <-- Add this line

class UR5ePickPlace:
    """UR5ePickPlace handles controlling the UR5e robot and the RG2 gripper."""

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pickandplace', anonymous=True)

        # Initialize MoveGroupCommander for UR5e robot
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("ur5e_arm")

        # Set planning parameters
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.05)
        self.move_group.set_planning_time(10.0)
        self.move_group.limit_max_cartesian_link_speed(0.04, link_name="flange")

    def ready_position(self):
        """Joint positions for the ready position.in radian"""
        return [0.349, -1.518, -1.780, -1.378, 1.553, 1.797]

    def start_position(self):
        """Joint positions for the start position."""
        return [0.034, -1.553, -1.745, -1.361, 1.553, 1.483]

    def pick_position(self):
        """Joint positions for the pick position."""
        return [-0.017, -1.727, -2.059, -0.802, 1.518, 1.378]

    def end_position(self):
        """Joint positions for the end position."""
        return [0.541, -1.570, -1.745, -1.361, 1.553, 1.989]

    def place_position(self):
        """Joint positions for the place position."""
        return [0.541, -1.658, -1.937, -1.082, 1.553, 1.989]

    def move_to_position(self, joint_goal):
        """Move to the specified joint position."""
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()  # Ensure there's no residual movement

    def open_gripper(self):
        """Open the RG2 gripper using digital output."""
        rospy.loginfo("Opening the gripper...")
        rospy.wait_for_service('/ur_hardware_interface/set_io')
        try:
            set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
            response = set_io(1, 1, 1)  # Set digital output 1 to 0 (open gripper)
            rospy.loginfo(f"Gripper opened: {response}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to open gripper: {e}")

    def close_gripper(self):
        """Close the RG2 gripper using digital output."""
        rospy.loginfo("Closing the gripper...")
        rospy.wait_for_service('/ur_hardware_interface/set_io')
        try:
            set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
            response = set_io(1, 1, 0)  # Set digital output 1 to 1 (close gripper)
            rospy.loginfo(f"Gripper closed: {response}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to close gripper: {e}")

    def pick_and_place(self):
        """Perform the pick-and-place operation."""

        # Move to the ready position
        rospy.loginfo("Moving to the ready position...")
        self.move_to_position(self.ready_position())
        rospy.sleep(2) 

        # Move to the start position
        rospy.loginfo("Moving to the start position...")
        self.move_to_position(self.start_position())
        rospy.sleep(2)

        # Open the gripper
        self.open_gripper()
        rospy.sleep(2)

        # Move to the pick position
        rospy.loginfo("Moving to the pick position...")
        self.move_to_position(self.pick_position())
        rospy.sleep(2)

        # Close the gripper (pick object)
        rospy.loginfo("Closing the gripper...")
        self.close_gripper()
        rospy.sleep(5)
        
        # Move back to the start position
        rospy.loginfo("Moving to the start position...")
        self.move_to_position(self.start_position())
        rospy.sleep(2)

        # Move to the end position
        rospy.loginfo("Moving to the end position...")
        self.move_to_position(self.end_position())
        rospy.sleep(2)

        # Move to the place position
        rospy.loginfo("Moving to the place position...")
        self.move_to_position(self.place_position())
        rospy.sleep(2)

        # Open the gripper (place object)
        rospy.loginfo("Opening the gripper...")
        self.open_gripper()
        rospy.sleep(2)

        # Move back to the end position
        rospy.loginfo("Moving back to the end position...")
        self.move_to_position(self.end_position())
        rospy.sleep(2)


if __name__ == '__main__':
    pickandplace = UR5ePickPlace()

    try:
        pickandplace.pick_and_place()
    except rospy.ROSInterruptException:
        pass

    moveit_commander.roscpp_shutdown()

