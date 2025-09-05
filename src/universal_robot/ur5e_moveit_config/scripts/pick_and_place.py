#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from ur_msgs.srv import SetIO  # <-- Add this line

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("pick_and_place", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "manipulator".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        tool0 = move_group.get_end_effector_link()
        print("============ End effector link: %s" % tool0)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.tool0 = tool0
        self.group_names = group_names

    def go_to_joint_state(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau/4
        joint_goal[2] = tau/4
        joint_goal[3] = tau/4
        joint_goal[4] = tau/4
        joint_goal[5] = 0
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self,a,b,c):
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 1.0
        pose_goal.position.x = a
        pose_goal.position.y = b
        pose_goal.position.z = c
        move_group.set_pose_target(pose_goal)
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self,a,b,c):
        move_group = self.move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x += a
        wpose.position.y += b
        wpose.position.z += c
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01  # waypoints to follow  # eef_step
        )
        return plan, fraction

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        box_name = self.box_name
        scene = self.scene
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = box_name in scene.get_known_object_names()
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

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

# -*- coding: utf-8 -*-
import math
import rospy
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

def _dict_to_pose(d, current_orientation=None):
    p = Pose()
    p.position.x = d["x"]; p.position.y = d["y"]; p.position.z = d["z"]
    if {"qx","qy","qz","qw"}.issubset(d.keys()):
        p.orientation.x = d["qx"]; p.orientation.y = d["qy"]
        p.orientation.z = d["qz"]; p.orientation.w = d["qw"]
    elif {"roll","pitch","yaw"}.issubset(d.keys()):
        qx, qy, qz, qw = quaternion_from_euler(d["roll"], d["pitch"], d["yaw"])
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    elif current_orientation is not None:
        p.orientation = current_orientation
    else:
        raise ValueError("Orientation absente et pose courante inconnue.")
    return p

def follow_cartesian_points(move_group, robot, points,
                            eef_step=0.01,            # m
                            jump_threshold=0.0,       # 0 = désactivé
                            velocity_scale=0.2,       # 0..1
                            accel_scale=0.2,          # 0..1
                            avoid_collisions=True,
                            min_fraction=0.98,
                            max_attempts=4,
                            execute=True):
    """
    Fait suivre au robot un chemin cartésien défini par une liste de waypoints.
    - move_group : moveit_commander.MoveGroupCommander
    - robot      : moveit_commander.RobotCommander
    - points     : list[geometry_msgs.msg.Pose] ou list[dict] (x,y,z + (qx,qy,qz,qw) ou (roll,pitch,yaw))
    Retourne (success: bool, fraction: float, plan)
    """
    current_pose = move_group.get_current_pose().pose
    current_orientation = current_pose.orientation

    waypoints = []
    for pt in points:
        if isinstance(pt, Pose):
            waypoints.append(pt)
        elif isinstance(pt, dict):
            waypoints.append(_dict_to_pose(pt, current_orientation))
        else:
            raise TypeError("Chaque point doit être un Pose ou un dict.")

    attempt = 0
    fraction = 0.0
    plan = None
    _eef_step = float(eef_step)

    while attempt < max_attempts:
        # ---- CORRECTIF MINIMAL ICI : arguments positionnels + compatibilité sans avoid_collisions
        try:
            traj, fraction = move_group.compute_cartesian_path(
                waypoints, _eef_step, jump_threshold, avoid_collisions
            )
        except TypeError:
            traj, fraction = move_group.compute_cartesian_path(
                waypoints, _eef_step, jump_threshold
            )
        # ---- FIN CORRECTIF

        # Retiming (respect des vitesses/accélérations)
        traj = move_group.retime_trajectory(robot.get_current_state(), traj, velocity_scale, accel_scale)

        if fraction >= min_fraction:
            plan = traj
            break
        _eef_step = max(_eef_step * 0.5, 0.0025)
        attempt += 1

    success = plan is not None and fraction >= min_fraction
    if success and execute:
        move_group.execute(plan, wait=True)

    return success, fraction, plan

def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")

        tutorial = MoveGroupPythonInterfaceTutorial()

        input("test")
        points = [
            {"x": 0.40, "y": 0.00, "z": 0.30, "roll": 0.0, "pitch": math.pi/2, "yaw": 0.0},
            {"x": 0.45, "y": 0.10, "z": 0.28, "roll": 0.0, "pitch": math.pi/2, "yaw": 0.0},
            {"x": 0.50, "y": 0.00, "z": 0.26, "roll": 0.0, "pitch": math.pi/2, "yaw": 0.0},
        ]

        ok, frac, plan = follow_cartesian_points(
            tutorial.move_group, tutorial.robot, points, velocity_scale=0.3, accel_scale=0.3
        )

        input("============ Press `Enter` to execute a movement using a pose goal ...")
        tutorial.go_to_pose_goal(0.3,0.3,0.4)

        input("============ Press `Enter` to tout faire Cartesian path ...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path(0,0,-0.2)
        tutorial.display_trajectory(cartesian_plan)
        tutorial.execute_plan(cartesian_plan)

        input("============ Press `Enter` to tout faire Cartesian path ...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path(0,0,0.18)
        tutorial.display_trajectory(cartesian_plan)
        tutorial.execute_plan(cartesian_plan)

        input("============ Press `Enter` to execute a movement using a pose goal ...")
        tutorial.go_to_pose_goal(0.22,0.22,0.45)

        input("============ Press `Enter` to tout faire Cartesian path ...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path(0,0,-0.18)
        tutorial.display_trajectory(cartesian_plan)
        tutorial.execute_plan(cartesian_plan)

        input("============ Press `Enter` to tout faire Cartesian path ...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path(0,0,0.1)
        tutorial.display_trajectory(cartesian_plan)
        tutorial.execute_plan(cartesian_plan)

        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _rospy:
##    http://docs.ros.org/noetic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL
