#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions de code source doivent conserver l'avis de copyright
#    ci-dessus, cette liste de conditions et l'avis de non-responsabilité.
#  * Les redistributions sous forme binaire doivent reproduire l'avis de copyright
#    ci-dessus, cette liste de conditions et l'avis de non-responsabilité dans la
#    documentation et/ou autres matériaux fournis avec la distribution.
#  * Ni le nom de SRI International ni les noms de ses contributeurs ne peuvent
#    être utilisés pour approuver ou promouvoir des produits dérivés de ce logiciel
#    sans autorisation écrite préalable spécifique.
#
# CE LOGICIEL EST FOURNI PAR LES DÉTENTEURS DU COPYRIGHT ET LES CONTRIBUTEURS "EN L'ÉTAT"
# ET TOUTES GARANTIES EXPRESSES OU IMPLICITES, Y COMPRIS, MAIS SANS S'Y LIMITER,
# LES GARANTIES IMPLICITES DE QUALITÉ MARCHANDE ET D'ADÉQUATION À UN USAGE PARTICULIER
# SONT DÉCLINÉES. EN AUCUN CAS LE PROPRIÉTAIRE DU COPYRIGHT OU LES CONTRIBUTEURS
# NE SERONT RESPONSABLES DE DOMMAGES DIRECTS, INDIRECTS, ACCESSOIRES, SPÉCIAUX,
# EXEMPLAIRES OU CONSÉCUTIFS (Y COMPRIS, MAIS SANS S'Y LIMITER, L'ACQUISITION DE BIENS
# OU SERVICES DE SUBSTITUTION, LA PERTE D'UTILISATION, DE DONNÉES OU DE PROFITS,
# OU L'INTERRUPTION D'ACTIVITÉ) QUELLE QU'EN SOIT LA CAUSE ET SUR TOUTE THÉORIE
# DE RESPONSABILITÉ, CONTRACTUELLE, STRICTE OU DÉLICTUELLE (Y COMPRIS LA NÉGLIGENCE
# OU AUTRE) DÉCOULANT DE L'UTILISATION DE CE LOGICIEL, MÊME SI AVISÉ DE LA POSSIBILITÉ
# DE TELS DOMMAGES.

from __future__ import print_function
from six.moves import input

import sys
import copy
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from ur_msgs.srv import SetIO

try:
    from math import pi, tau, dist, fabs, cos, sqrt
except Exception:
    from math import pi, fabs, cos, sqrt
    tau = 2.0 * pi
    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from moveit_commander.conversions import pose_to_list


# ---------------------------------------------------------------------------
# Helpers MoveIt
# ---------------------------------------------------------------------------

def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False
    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)
    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        d = dist((x1, y1, z1), (x0, y0, z0))
        cos_phi_half = fabs(qx0*qx1 + qy0*qy1 + qz0*qz1 + qw0*qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance/2.0)
    return True


class MoveGroupPythonInterfaceTutorial(object):
    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("pick_and_place", anonymous=True)

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
        print("============ Planning frame: %s" % planning_frame)
        tool0 = move_group.get_end_effector_link()
        print("============ End effector link: %s" % tool0)
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

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

    def go_to_pose_goal(self, a, b, c):
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        # Quaternion par défaut (ici x=1 => 180° autour de X) — adapte si besoin
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

    def plan_cartesian_path(self, a, b, c):
        move_group = self.move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x += a
        wpose.position.y += b
        wpose.position.z += c
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01
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

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
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
        rospy.loginfo("Opening the gripper...")
        rospy.wait_for_service('/ur_hardware_interface/set_io')
        try:
            set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
            response = set_io(1, 1, 1)
            rospy.loginfo("Gripper opened: %s", response)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to open gripper: %s", e)

    def close_gripper(self):
        rospy.loginfo("Closing the gripper...")
        rospy.wait_for_service('/ur_hardware_interface/set_io')
        try:
            set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
            response = set_io(1, 1, 0)
            rospy.loginfo("Gripper closed: %s", response)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to close gripper: %s", e)


# ---------------------------------------------------------------------------
# Génération de waypoints & suivi cartésien
# ---------------------------------------------------------------------------

def _dict_to_pose(d, current_orientation=None):
    p = Pose()
    p.position.x = d["x"]; p.position.y = d["y"]; p.position.z = d["z"]

    if {"qx", "qy", "qz", "qw"}.issubset(d.keys()):
        p.orientation.x = d["qx"]; p.orientation.y = d["qy"]
        p.orientation.z = d["qz"]; p.orientation.w = d["qw"]
    elif {"roll", "pitch", "yaw"}.issubset(d.keys()):
        qx, qy, qz, qw = quaternion_from_euler(d["roll"], d["pitch"], d["yaw"])
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    elif current_orientation is not None:
        p.orientation = current_orientation
    else:
        raise ValueError("Orientation absente et pose courante inconnue.")
    return p


def follow_cartesian_points(move_group, robot, points,
                            eef_step=0.01,
                            jump_threshold=0.0,
                            velocity_scale=0.2,
                            accel_scale=0.2,
                            avoid_collisions=True,  # conservé pour compat
                            min_fraction=0.98,
                            max_attempts=4,
                            execute=True):
    """
    Fait suivre au robot un chemin cartésien défini par une liste de waypoints.
    - move_group : moveit_commander.MoveGroupCommander
    - robot      : moveit_commander.RobotCommander
    - points     : list[Pose] ou list[dict] (x,y,z + quaternion ou RPY)
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
        # --- Compatibilité multi-versions de MoveIt ---
        try:
            # Nouvelle API: (waypoints, eef_step, avoid_collisions)
            traj, fraction = move_group.compute_cartesian_path(
                waypoints, _eef_step, bool(avoid_collisions)
            )
        except TypeError:
            try:
                # Ancienne API (4 args): (waypoints, eef_step, jump_threshold, avoid_collisions)
                traj, fraction = move_group.compute_cartesian_path(
                    waypoints, _eef_step, float(jump_threshold), bool(avoid_collisions)
                )
            except TypeError:
                # Ancienne API (3 args): (waypoints, eef_step, jump_threshold)
                traj, fraction = move_group.compute_cartesian_path(
                    waypoints, _eef_step, float(jump_threshold)
                )
        # -------------------------------------------------

        # Retiming (respect vitesses/accélérations)
        traj = move_group.retime_trajectory(
            robot.get_current_state(), traj, velocity_scale, accel_scale
        )

        if fraction >= min_fraction:
            plan = traj
            break

        _eef_step = max(_eef_step * 0.5, 0.0025)
        attempt += 1

    success = plan is not None and fraction >= min_fraction
    if success and execute:
        move_group.execute(plan, wait=True)

    return success, fraction, plan


def make_circle_points(center, radius,
                       normal='z',
                       start_angle=0.0, end_angle=2*math.pi, num=160,
                       roll=0.0, pitch=math.pi/2, yaw=0.0,
                       follow_tangent=False):
    """
    Génère une LISTE DE POINTS (dict) décrivant un cercle cartésien pour MoveIt.

    Paramètres
    ----------
    center : tuple (cx, cy, cz)
        Centre du cercle (m).
    radius : float
        Rayon du cercle (m).
    normal : str
        'z' -> cercle horizontal dans le plan XY (z constant),
        'x' -> cercle vertical dans le plan YZ (x constant),
        'y' -> cercle vertical dans le plan XZ (y constant).
    start_angle, end_angle : float
        Angles en radians (0 → 2π pour un tour).
    num : int
        Nombre de waypoints (plus grand = plus lisse).
    roll, pitch, yaw : float
        Orientation RPY de l’outil (si follow_tangent=False).
    follow_tangent : bool
        Si True, adapte le yaw pour suivre la tangente du mouvement
        (utile pour des opérations de dessin/grattage).

    Retour
    ------
    list[dict] : chaque dict contient x, y, z, roll, pitch, yaw
    """
    cx, cy, cz = center
    pts = []
    for i in range(num + 1):
        t = start_angle + (end_angle - start_angle) * i / num

        if normal == 'z':      # cercle dans XY, z constant
            x = cx + radius * math.cos(t)
            y = cy + radius * math.sin(t)
            z = cz
            yaw_i = (yaw + t) if follow_tangent else yaw
        elif normal == 'x':    # cercle dans YZ, x constant
            x = cx
            y = cy + radius * math.cos(t)
            z = cz + radius * math.sin(t)
            yaw_i = (yaw + t) if follow_tangent else yaw
        elif normal == 'y':    # cercle dans XZ, y constant
            x = cx + radius * math.cos(t)
            y = cy
            z = cz + radius * math.sin(t)
            yaw_i = (yaw + t) if follow_tangent else yaw
        else:
            raise ValueError("normal doit être 'x', 'y' ou 'z'")

        pts.append({
            "x": x, "y": y, "z": z,
            "roll": roll, "pitch": pitch, "yaw": yaw_i
        })
    return pts


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------

def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Demo: Trajectoire Circulaire avec MoveIt (waypoints cartésiens)")
        print("----------------------------------------------------------")
        print("Ctrl-C pour quitter à tout moment")
        print("")

        tutorial = MoveGroupPythonInterfaceTutorial()

        # === 1) Définis TON cercle ici ============================
        center = (0.45, 0.00, 0.26)   # Centre du cercle (m)
        radius = 0.08                 # Rayon (m)
        normal = 'z'                  # 'z' -> plan XY ; 'x' ou 'y' pour vertical
        num_points = 160              # Nombre de waypoints
        # Orientation outil: ici "pointe vers le bas" (UR classique)
        roll = 0.0
        pitch = math.pi/2
        yaw = 0.0
        follow_tangent = False        # True si tu veux suivre la tangente
        # ==========================================================

        # Optionnel: amener l'outil au centre avant de commencer
        tutorial.go_to_pose_goal(center[0], center[1], center[2])

        input("Appuie sur Entrée pour générer les points du cercle...")

        # 2) Génère LA LISTE DE POINTS
        circle_points = make_circle_points(
            center=center,
            radius=radius,
            normal=normal,
            start_angle=0.0,
            end_angle=2*math.pi,
            num=num_points,
            roll=roll, pitch=pitch, yaw=yaw,
            follow_tangent=follow_tangent
        )

        print("Nombre de waypoints générés:", len(circle_points))

        input("Appuie sur Entrée pour planifier & exécuter la trajectoire...")

        # 3) Passe simplement cette liste à UNE fonction
        ok, frac, plan = follow_cartesian_points(
            tutorial.move_group,
            tutorial.robot,
            circle_points,
            eef_step=0.008,       # résolution discrète (m)
            jump_threshold=0.0,
            velocity_scale=0.25,
            accel_scale=0.25,
            min_fraction=0.95,
            execute=True          # exécuter directement si plan OK
        )
        print("Résultat: success=%s, fraction=%.3f" % (ok, frac))

        print("============ Trajectoire circulaire terminée !")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
