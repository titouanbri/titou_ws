#!/usr/bin/env python
from __future__ import print_function
from six.moves import input
import numpy as np
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from ur_msgs.srv import SetIO  # <-- Add this line
from geometry_msgs.msg import WrenchStamped  # exemple, à changer selon ton message
from std_msgs.msg import Float64MultiArray
from collections import deque
import tf.transformations as tf_trans
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_multiply, quaternion_inverse
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R
from scipy.signal import butter, lfilter_zi, lfilter, filtfilt
from geometry_msgs.msg import WrenchStamped, Wrench, Vector3
from controller_manager_msgs.srv import SwitchController

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



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


        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("sensor_test", anonymous=True)

        
        robot = moveit_commander.RobotCommander()

        
        scene = moveit_commander.PlanningSceneInterface()

        
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        
        planning_frame = move_group.get_planning_frame()
        # print("============ Planning frame: %s" % planning_frame)

        tool0 = move_group.get_end_effector_link()
        # print("============ End effector link: %s" % tool0)

        group_names = robot.get_group_names()
        # print("============ Available Planning Groups:", robot.get_group_names())

        
        # print("============ Printing robot state")
        # print(robot.get_current_state())
        # print("")
        
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.tool0 = tool0
        self.group_names = group_names
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.force_data = None  # pour stocker la dernière donnée
        rospy.Subscriber('/force_sensor', WrenchStamped, self.force_callback)



        self.last_R_mat=None
        self.last_Rot=None


    def force_callback(self, msg):
        self.force_data = msg

    # def go_to_joint_state(self):
    #     move_group = self.move_group

        
    #     joint_goal = move_group.get_current_joint_values()
    #     joint_goal[0] = 0
    #     joint_goal[1] = -tau/4
    #     joint_goal[2] = tau/4
    #     joint_goal[3] = tau/4
    #     joint_goal[4] = tau/4
    #     joint_goal[5] = 0
        
    #     move_group.go(joint_goal, wait=True)

    #     move_group.stop()

    #     current_joints = move_group.get_current_joint_values()
    #     return all_close(joint_goal, current_joints, 0.01)

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

    # def plan_cartesian_path(self,a,b,c):
        
    #     move_group = self.move_group

    #     waypoints = []
    #     wpose = move_group.get_current_pose().pose
    #     wpose.position.x += a  
    #     wpose.position.y += b  
    #     wpose.position.z += c

    #     waypoints.append(copy.deepcopy(wpose))


        
    #     (plan, fraction) = move_group.compute_cartesian_path(
    #         waypoints, 0.01  # waypoints to follow 0.01 # eef_step
    #     )

    #     # Note: We are just planning, not asking move_group to actually move the robot yet:
    #     return plan, fraction
    # def display_trajectory(self, plan):
        
    #     robot = self.robot
    #     display_trajectory_publisher = self.display_trajectory_publisher

        
    #     display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    #     display_trajectory.trajectory_start = robot.get_current_state()
    #     display_trajectory.trajectory.append(plan)
    #     display_trajectory_publisher.publish(display_trajectory)
    # def execute_plan(self, plan):
        
    #     move_group = self.move_group

    #     move_group.execute(plan, wait=True)

    # def wait_for_state_update(
    #     self, box_is_known=False, box_is_attached=False, timeout=4
    # ):
        
    #     box_name = self.box_name
    #     scene = self.scene

    #     start = rospy.get_time()
    #     seconds = rospy.get_time()
    #     while (seconds - start < timeout) and not rospy.is_shutdown():
    #         # Test if the box is in attached objects
    #         attached_objects = scene.get_attached_objects([box_name])
    #         is_attached = len(attached_objects.keys()) > 0

    #         # Test if the box is in the scene.
    #         # Note that attaching the box will remove it from known_objects
    #         is_known = box_name in scene.get_known_object_names()

    #         # Test if we are in the expected state
    #         if (box_is_attached == is_attached) and (box_is_known == is_known):
    #             return True

    #         # Sleep so that we give other threads time on the processor
    #         rospy.sleep(0.1)
    #         seconds = rospy.get_time()

    #     # If we exited the while loop without returning then we timed out
    #     return False
    #     ## END_SUB_TUTORIAL
        
    def switch_controllers(self,start_list, stop_list):
        rospy.wait_for_service('/controller_manager/switch_controller')
        try:
            switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            resp = switch_controller(start_controllers=start_list, stop_controllers=stop_list, strictness=1)
            return resp.ok
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def start_controller(self,controller_name):
        rospy.wait_for_service('/controller_manager/switch_controller')
        try:
            switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            resp = switch_controller(start_controllers=[controller_name], stop_controllers=[], strictness=1)
            if resp.ok:
                rospy.loginfo(f"Controller '{controller_name}' started.")
                return True
            else:
                rospy.logwarn(f"Failed to start controller '{controller_name}'.")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

def main():
    try:
        print("t'as pas le temps de lire de toute façon")
      
        tutorial = MoveGroupPythonInterfaceTutorial()

        input("============ Press `Enter` to the initial pose ...")
        # tutorial.switch_controllers(['scaled_pos_joint_traj_controller'], ['joint_group_vel_controller'])
        tutorial.go_to_pose_goal(0.2,0.2,0.2)


  

        # input("============ Press `Enter` to tout faire Cartesian path ...")
        # cartesian_plan, fraction = tutorial.plan_cartesian_path(0,0,-0.2)     
        # tutorial.display_trajectory(cartesian_plan)
        # tutorial.execute_plan(cartesian_plan)
        
        # input("============ Press `Enter` to close la mano ...")
        # tutorial.close_gripper()



        print("finito")
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

