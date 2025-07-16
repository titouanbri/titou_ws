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
from geometry_msgs.msg import Quaternion,Pose
from tf.transformations import quaternion_multiply, quaternion_inverse
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R
from scipy.signal import butter, lfilter_zi, lfilter, filtfilt
from geometry_msgs.msg import WrenchStamped, Wrench, Vector3
from controller_manager_msgs.srv import SwitchController
from trajectory_msgs.msg import JointTrajectoryPoint
import math
from typing import Tuple



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



        self.last_R_mat=None
        self.last_Rot=None



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

    def euler_to_quaternion(self,roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
        """
        Convertit des angles d'Euler (roll, pitch, yaw) en quaternion (w, x, y, z).

        :param roll: rotation autour de l'axe X, en radians
        :param pitch: rotation autour de l'axe Y, en radians
        :param yaw: rotation autour de l'axe Z, en radians
        :return: tuple (w, x, y, z) représentant le quaternion
        """
        # demi-angles
        hr = roll  / 2.0
        hp = pitch / 2.0
        hy = yaw   / 2.0

        cr = math.cos(hr)
        sr = math.sin(hr)
        cp = math.cos(hp)
        sp = math.sin(hp)
        cy = math.cos(hy)
        sy = math.sin(hy)

        # calcul du quaternion
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        return x/norm, y/norm, z/norm, w/norm



    def quaternion_to_euler(self,q):
        """
        Convertit un quaternion en angles d'Euler.

        Paramètres
        ----------
        q : list ou tuple de quatre floats
            Quaternion sous la forme [i, j, k, w] (x, y, z, w).

        Retour
        ------
        list of float
            Angles d'Euler [roll, pitch, yaw] en radians.
        """
        i, j, k, w = q

        # Roll (rotation autour de X)
        t0 = 2.0 * (w * i + j * k)
        t1 = 1.0 - 2.0 * (i * i + j * j)
        roll = math.atan2(t0, t1)

        # Pitch (rotation autour de Y)
        t2 = 2.0 * (w * j - k * i)
        t2 = max(-1.0, min(1.0, t2))  # clamp
        pitch = math.asin(t2)

        # Yaw (rotation autour de Z)
        t3 = 2.0 * (w * k + i * j)
        t4 = 1.0 - 2.0 * (j * j + k * k)
        yaw = math.atan2(t3, t4)

        return [roll, pitch, yaw]
   

    def go_to_pose_goal(self,x,y,z,roll, pitch, yaw):


        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        i,j,k,w=self.euler_to_quaternion(roll,pitch,yaw)
        pose_goal.orientation.x = i
        pose_goal.orientation.y = j
        pose_goal.orientation.z = k
        pose_goal.orientation.w = w

        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

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
            waypoints, 0.01  # waypoints to follow 0.01 # eef_step
        )

        # Note: We are just planning, not asking move_group to actually move the robot yet:
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
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL
        
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

    def traj1(self,pose_0,angle,gripper_L,nb_point):
        waypoints=[]
        pose_0=np.array(pose_0)
        pose_safe=pose_0-np.array([0.05,0.05,0.05,0,0,0])
        self.go_to_pose_goal(*pose_safe)

        for n in range (nb_point):
            pose=Pose()
            teta=2*np.pi*n/nb_point
            roll=pose_0[3] + np.sin(teta)*angle
            pitch=pose_0[4] + np.cos(teta)*angle
            yaw=pose_0[5]

            x=pose_0[0]
            y=pose_0[1]+gripper_L*np.sin(angle)*np.sin(teta)
            z=pose_0[2]+gripper_L*np.sin(angle)*np.cos(teta)

            qx,qy,qz,qw=self.euler_to_quaternion(roll,pitch,yaw)
            pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            pose.position.x,pose.position.y,pose.position.z = x,y,z
            waypoints.append(pose)
            # self.go_to_pose_goal(x,y,z,roll,pitch,yaw)

        move_group = self.move_group

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,0.01  # waypoints to follow 0.005 # eef_step
        )
        rospy.loginfo(f"Trajectoire planifiée à {fraction*100:.1f}% des waypoints")
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        
        if fraction > 0.99:
            # affichage rviz
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = robot.get_current_state()
            display_trajectory.trajectory.append(plan)
            display_trajectory_publisher.publish(display_trajectory)
            self.move_group.execute(plan, wait=True)
        else:
            rospy.logwarn("Planification incomplète, ajuste eef_step / rayon_angle / nb_point")      

    def traj(self, C: Tuple[float, float, float], axis: Tuple[float, float, float], angle: float, height: float, nb_point: int):
        """
        balayge conique

        C : point fixe durant le mouvement (pointe du cône)
        axis : axe du cône
        angle : demi angle du cône
        height : hauteur du cône
        nb_point : nombre de points de la trajectoire
        """
        # Numérisation des paramètres
        C = np.array(C, dtype=float)
        v0 = np.array(axis, dtype=float)
        norm_v0 = np.linalg.norm(v0)
        if norm_v0 < 1e-6:
            rospy.logerr("L'axe du cône ne peut pas être nul !")
            return
        v0 /= norm_v0

        # Calcul de la génératrice L du cône (distance sommet -> cercle) en fonction de la hauteur et de l'ouverture
        # L * cos(angle) = height  =>  L = height / cos(angle)
        try:
            L = height / math.cos(angle)
        except ZeroDivisionError:
            rospy.logerr("Angle de cône invalide, cos(angle)=0")
            return

        # Position de départ en retrait le long de -v0 pour éviter collision
        safe_offset = height +0.05
        C_safe = C + safe_offset * v0

        # Calcul de l'orientation initiale pour pointer vers C
        z_axis0 = (C - C_safe)
        z_axis0 /= np.linalg.norm(z_axis0)
        x_temp0 = np.cross([0, 1, 0], z_axis0)
        if np.linalg.norm(x_temp0) < 1e-3:
            x_temp0 = np.cross([1, 0, 0], z_axis0)
        x_axis0 = x_temp0 / np.linalg.norm(x_temp0)
        y_axis0 = np.cross(z_axis0, x_axis0)
        R0 = np.vstack((x_axis0, y_axis0, z_axis0)).T
        M0 = np.eye(4)
        M0[:3, :3] = R0
        quat0 = tf_trans.quaternion_from_matrix(M0)
        orientation=self.quaternion_to_euler(quat0)
        # Orientation initiale arbitraire (pointant en Z global)
        self.go_to_pose_goal(*C_safe, *orientation)

        # Construction d'une base orthonormée perpendiculaire à v0
        arbitrary = np.array([1.0, 0.0, 0.0]) if abs(v0[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        e1 = np.cross(v0, arbitrary)
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(v0, e1)
        e2 /= np.linalg.norm(e2)

        waypoints = []
        for i in range(nb_point):
            phi = 2 * math.pi * i / nb_point
            # Direction conique unitaire
            direction = math.cos(angle) * v0 + math.sin(angle) * (math.cos(phi) * e1 + math.sin(phi) * e2)
            # Position du waypoint
            P = C + L * direction

            # Orientation : l'outil pointe vers C
            z_axis = (C - P)
            z_axis /= np.linalg.norm(z_axis)
            x_temp = np.cross([0, 1, 0], z_axis)
            if np.linalg.norm(x_temp) < 1e-3:
                x_temp = np.cross([1, 0, 0], z_axis)
            x_axis = x_temp / np.linalg.norm(x_temp)
            y_axis = np.cross(z_axis, x_axis)

            # Conversion en quaternion
            R_mat = np.vstack((x_axis, y_axis, z_axis)).T
            M = np.eye(4)
            M[:3, :3] = R_mat
            quat = tf_trans.quaternion_from_matrix(M)

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = P
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
            waypoints.append(pose)

        # Planification et exécution
        plan, fraction = self.move_group.compute_cartesian_path(waypoints, eef_step=0.01)
        rospy.loginfo(f"Planification conique achevée à {fraction*100:.1f}%")
        if fraction > 0.99:
            # traj = moveit_msgs.msg.DisplayTrajectory()
            # traj.trajectory_start = self.robot.get_current_state()
            # traj.trajectory.append(plan)
            # self.display_trajectory_publisher.publish(traj)
            self.move_group.execute(plan, wait=True)
        else:
            rospy.logwarn("Planification incomplète : augmentez nb_point ou ajustez angle/height.")


def main():
    try:
        print("t'as pas le temps de lire de toute façon")
      
        tutorial = MoveGroupPythonInterfaceTutorial()
        # tutorial.switch_controllers(['scaled_pos_joint_traj_controller'], ['joint_group_vel_controller'])
        input("============ Press `Enter` open the gripper ...")
        # tutorial.open_gripper()
        
        input("============ Press `Enter` to go to the head ...")
        tutorial.go_to_pose_goal(0.4,0.4,0.45,0,np.pi,0)
        tutorial.go_to_pose_goal(0.4,0.4,0.35,0,np.pi,0)

        input("============ Press `Enter` close the gripper ...")
        # tutorial.close_gripper()


        input("============ Press `Enter` to start the movement ...")
        tutorial.go_to_pose_goal(0.4,0.4,0.45,0,np.pi,0)
        tutorial.traj([0.50,0.40,0.27],[-1,0,0],0.25,0.35,20)
      
        print("finito")
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

