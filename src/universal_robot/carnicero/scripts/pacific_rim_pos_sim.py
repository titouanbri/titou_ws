#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Adaptation du contrôleur PR_control_pos pour la simulation Gazebo.
Utilise /eff_joint_traj_controller pour commander le robot UR dans ur_gazebo
et prend en charge l'initialisation de la pose ainsi que le fallback TF.
"""

import os
import sys
import importlib
import rospy
import PyKDL
import tf2_ros
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from geometry_msgs.msg import TransformStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController

# ---- Début bloc à coller (ROS 1) -------------------------------------------

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
POS_INIT = [0.0, -1.2, 1.6, -0.4, 1.4, 0.0]   # radians
DUR_INIT = 3.0  # secondes

def send_initial_pose_ros1(topic="/eff_joint_traj_controller/command"):
    pub = rospy.Publisher(topic, JointTrajectory, queue_size=1)
    start = rospy.Time.now()
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        if (rospy.Time.now() - start).to_sec() > 3.0:
            break
        rospy.sleep(0.05)

    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
    traj.joint_names = JOINTS
    pt = JointTrajectoryPoint()
    pt.positions = POS_INIT
    pt.velocities = [0.0] * len(JOINTS)
    pt.time_from_start = rospy.Duration(DUR_INIT)
    traj.points = [pt]

    rospy.loginfo("Envoi de la pose initiale (ROS1)…")
    pub.publish(traj)
    rospy.sleep(DUR_INIT + 0.3)
# ---- Fin bloc à coller (ROS 1) ---------------------------------------------

# --- import du contrôleur d'PR position ---
try:  # pragma: no cover - seulement en runtime ROS
    from pacific_rim_pos import PR_control_pos
except Exception:
    script_path = os.path.join(os.path.dirname(__file__), "pacific_rim_pos.py")
    if sys.version_info[0] >= 3:
        spec = importlib.util.spec_from_file_location("pacific_rim_pos", script_path)
        pacific_rim_pos = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(pacific_rim_pos)  # type: ignore[attr-defined]
    else:  # Python 2
        import imp  # type: ignore[deprecated]
        pacific_rim_pos = imp.load_source("pacific_rim_pos", script_path)
    PR_control_pos = pacific_rim_pos.PR_control_pos


class PR_control_pos_sim(PR_control_pos):
    """Contrôleur PR position adapté à Gazebo."""

    def __init__(self):
        # Initialisation de la classe de base
        super(PR_control_pos_sim, self).__init__()

        # En simulation on contrôle la frame "tool0"
        self.tool_frame = rospy.get_param("~tool_frame", "tool0")

        # Recalcule la chaîne KDL pour base_link -> tool0
        robot_description = rospy.get_param("/robot_description")
        robot = URDF.from_xml_string(robot_description)
        ok, tree = treeFromUrdfModel(robot)
        if not ok:
            raise rospy.ROSException("URDF -> KDL Tree failed")
        self.kdl_chain = tree.getChain(self.base_frame, self.tool_frame)
        self.kdl_jnt_to_jac_solver = PyKDL.ChainJntToJacSolver(self.kdl_chain)
        self.kdl_fk_solver = PyKDL.ChainFkSolverPos_recursive(self.kdl_chain)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Namespace du contrôleur (optionnel)
        self.controller_ns = rospy.get_param("~controller_ns", "").rstrip("/")

        eff_topic = (
            self.controller_ns + "/eff_joint_traj_controller/command"
            if self.controller_ns
            else "/eff_joint_traj_controller/command"
        )
        self.traj_pub = rospy.Publisher(eff_topic, JointTrajectory, queue_size=10)

    # ------- contrôleurs -------
    def switch_controllers(self, start_list, stop_list):
        ns = self.controller_ns
        service = (ns + "/controller_manager/switch_controller") if ns else "/controller_manager/switch_controller"
        rospy.wait_for_service(service)
        try:
            switch_controller = rospy.ServiceProxy(service, SwitchController)
            resp = switch_controller(start_controllers=start_list, stop_controllers=stop_list, strictness=1)
            return resp.ok
        except Exception as e:  # pragma: no cover
            rospy.logwarn("Switch controller fail: %s", e)
            return False

    # ------- initialisation TF + latch -------
    def init_offset_and_latch(self):
        if not self.tf_buffer.can_transform(
            self.base_frame, self.tool_frame, rospy.Time(0), rospy.Duration(5.0)
        ):
            joint_positions = PyKDL.JntArray(self.kdl_chain.getNrOfJoints())
            frame = PyKDL.Frame()
            self.kdl_fk_solver.JntToCart(joint_positions, frame)

            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self.base_frame
            t.child_frame_id = self.tool_frame
            t.transform.translation.x = frame.p[0]
            t.transform.translation.y = frame.p[1]
            t.transform.translation.z = frame.p[2]
            qx, qy, qz, qw = frame.M.GetQuaternion()
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.static_tf_broadcaster.sendTransform(t)
            rospy.sleep(0.1)

            if not self.tf_buffer.can_transform(
                self.base_frame, self.tool_frame, rospy.Time(0), rospy.Duration(5.0)
            ):
                raise rospy.ROSException(
                    "Transform %s -> %s not available" % (self.base_frame, self.tool_frame)
                )

        super(PR_control_pos_sim, self).init_offset_and_latch()


def main():
    try:
        ctrl = PR_control_pos_sim()
        start_ctrls = rospy.get_param("~start_controllers", ["eff_joint_traj_controller"])
        stop_ctrls = rospy.get_param("~stop_controllers", [])
        if start_ctrls or stop_ctrls:
            ok = ctrl.switch_controllers(start_ctrls, stop_ctrls)
            if not ok:
                rospy.logwarn("Impossible de (re)configurer les contrôleurs, on continue quand même.")
        send_initial_pose_ros1()
        rospy.loginfo("Init offset + latch…")
        ctrl.init_offset_and_latch()
        rospy.loginfo("Go.")
        ctrl.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        return


if __name__ == "__main__":
    main()
