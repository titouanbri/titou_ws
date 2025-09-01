#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
PR controller pour Gazebo (UR30) avec /eff_joint_traj_controller.

Cette version s’appuie sur le contrôleur "position seule" (wrist uniquement)
et crée un "pont" qui transforme les vitesses de joint (Float64MultiArray)
émises par la classe de base en petites cibles de position (JointTrajectory)
envoyées vers /eff_joint_traj_controller/command, qui est le contrôleur dispo
dans ur_gazebo.
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
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController
from threading import Lock



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
    # S’assurer que ROS est initialisé (si ton main ne l’a pas déjà fait)

    # Attendre la présence du contrôleur
    start = rospy.Time.now()
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        if (rospy.Time.now() - start).to_sec() > 3.0:
            break
        rospy.sleep(0.05)

    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now() + rospy.Duration(0.1)  # éviter "passé"
    traj.joint_names = JOINTS

    pt = JointTrajectoryPoint()
    pt.positions  = POS_INIT
    pt.velocities = [0.0]*len(JOINTS)
    pt.time_from_start = rospy.Duration(DUR_INIT)
    traj.points = [pt]

    rospy.loginfo("Envoi de la pose initiale (ROS1)…")
    pub.publish(traj)

    # Laisser le contrôleur exécuter la consigne avant de continuer
    rospy.sleep(DUR_INIT + 0.3)
# ---- Fin bloc à coller (ROS 1) ---------------------------------------------


# --- import du contrôleur d'PR "position seule" ---
try:  # pragma: no cover - seulement en runtime ROS
    from pacific_rim import PR_control  # <- ta classe simplifiée (wrist-only)
except Exception:
    script_path = os.path.join(os.path.dirname(__file__), "pacific_rim.py")
    if sys.version_info[0] >= 3:
        spec = importlib.util.spec_from_file_location("pacific_rim", script_path)
        pacific_rim = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(pacific_rim)  # type: ignore[attr-defined]
    else:  # Python 2
        import imp  # type: ignore[deprecated]
        pacific_rim = imp.load_source("pacific_rim", script_path)
    PR_control = pacific_rim.PR_control


class VelToTrajBridge(object):
    """
    Convertit des vitesses en petites cibles de position pour /eff_joint_traj_controller/command.
    Utilise un horizon futur (lookahead) pour éviter que le contrôleur rejette les points.
    """

    def __init__(self, traj_pub, joint_names, get_current_pos, dt):
        self.traj_pub = traj_pub
        self.joint_names = list(joint_names)
        self.get_current_pos = get_current_pos

        # On ne se sert plus de dt comme horizon ; on garde dt pour compat
        self.dt = float(dt)
        # Horizon futur du point (secs) et limite d’incrément (rad)
        self.lookahead = rospy.get_param("~lookahead", 0.5)           # 0.5 s
        self.max_increment = rospy.get_param("~max_increment", 0.20)  # rad (≈11.5°)

    def publish(self, msg):
        # msg est un Float64MultiArray (vitesses qdot) venant de la classe de base
        try:
            vels = list(msg.data)
        except Exception:
            return

        q_curr = self.get_current_pos()
        n = min(len(self.joint_names), len(q_curr), len(vels))
        if n == 0:
            return

        # Intégration simple + clamp
        q_next = []
        for i in range(n):
            dq = vels[i] * self.lookahead
            dq = max(min(dq, self.max_increment), -self.max_increment)
            q_next.append(q_curr[i] + dq)

        # IMPORTANT : estampiller DANS LE FUTUR
        now = rospy.Time.now()
        traj = JointTrajectory()
        traj.header.stamp = now + rospy.Duration.from_sec(self.lookahead)
        traj.joint_names = self.joint_names[:n]

        pt = JointTrajectoryPoint()
        pt.positions = q_next
        pt.time_from_start = rospy.Duration.from_sec(self.lookahead)
        traj.points = [pt]

        self.traj_pub.publish(traj)


class PR_control_sim(PR_control):
    """
    Adapte le contrôleur d'PR (position seule) pour Gazebo en utilisant
    /eff_joint_traj_controller (position) au lieu d'un publisher de vitesses.
    """

    def __init__(self):
        # Initialise d’abord la classe de base (wrist-only, publie des vitesses)
        super(PR_control_sim, self).__init__()

        # En simulation on utilise "tool0" comme frame outil
        self.tool_frame = rospy.get_param("~tool_frame", "tool0")

        # Reconstruire la chaîne KDL pour base_link -> tool0
        robot_description = rospy.get_param("/robot_description")
        robot = URDF.from_xml_string(robot_description)
        ok, tree = treeFromUrdfModel(robot)
        if not ok:
            raise rospy.ROSException("URDF -> KDL Tree failed")
        self.kdl_chain = tree.getChain(self.base_frame, self.tool_frame)
        self.kdl_jnt_to_jac_solver = PyKDL.ChainJntToJacSolver(self.kdl_chain)
        self.kdl_fk_solver = PyKDL.ChainFkSolverPos_recursive(self.kdl_chain)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Namespace contrôleur (vide par défaut)
        self.controller_ns = rospy.get_param("~controller_ns", "").rstrip("/")

        # Topic du contrôleur trajectoire (présent dans ur_gazebo)
        eff_topic = (
            self.controller_ns + "/eff_joint_traj_controller/command"
            if self.controller_ns
            else "/eff_joint_traj_controller/command"
        )
        self.traj_pub = rospy.Publisher(eff_topic, JointTrajectory, queue_size=10)

        # Noms de joints UR (UR30)
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # Abonnement à /joint_states pour connaître la position actuelle
        self._lock = Lock()
        self._q = [0.0] * len(self.joint_names)
        self._js_sub = rospy.Subscriber("/joint_states", JointState, self._js_cb)

        # Pas d’envoi (horizon) pour la conversion vit->pos (en s)
        self.bridge_dt = rospy.get_param("~bridge_dt", 0.10)  # 100 ms

        # Remplace l’éditeur de vitesses par le "bridge" vers JointTrajectory
        self.joint_vel_pub = VelToTrajBridge(
            self.traj_pub, self.joint_names, self._get_current_pos, self.bridge_dt
        )

    # ------- utils -------

    def _js_cb(self, msg):
        # Met à jour self._q dans l'ordre de self.joint_names
        name_to_pos = dict(zip(msg.name, msg.position))
        with self._lock:
            self._q = [name_to_pos.get(j, q_old) for j, q_old in zip(self.joint_names, self._q)]

    def _get_current_pos(self):
        with self._lock:
            return list(self._q)

    # ------- contrôleurs -------

    def switch_controllers(self, start_list, stop_list):
        """
        Démarre eff_joint_traj_controller (position). Pas de joint_group_vel_controller en simu.
        """
        ns = self.controller_ns
        service = (ns + "/controller_manager/switch_controller") if ns else "/controller_manager/switch_controller"
        rospy.wait_for_service(service)
        try:
            switch_controller = rospy.ServiceProxy(service, SwitchController)
            resp = switch_controller(
                start_controllers=start_list, stop_controllers=stop_list, strictness=1
            )
            return resp.ok
        except Exception as e:  # pragma: no cover
            rospy.logwarn("Switch controller fail: %s", e)
            return False

    def init_offset_and_latch(self):
        """
        Assure la TF base_link -> tool0 (fallback URDF si absente), puis init de la classe de base.
        """
        if not self.tf_buffer.can_transform(
            self.base_frame, self.tool_frame, rospy.Time(0), rospy.Duration(5.0)
        ):
            # Fallback: TF statique depuis la FK à zéro
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

        # Lancement de l'init offset + latch de la classe de base (position seule)
        super(PR_control_sim, self).init_offset_and_latch()


def main():
    try:
        ctrl = PR_control_sim()
        # Démarrer le contrôleur présent en simu
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
