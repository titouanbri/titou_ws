#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Séquence UR30 (Gazebo) :
- 0–T_hold : coordonnées fixes
- T_hold–T_hold+T_lift : le end-effector (wrist) monte de lift_distance (par défaut 0.50 m)
- ensuite : cercle autour de la position levée (plan XY)

Publie des PointStamped sur:
  - right_arm/elbow
  - right_arm/wrist
"""

import math
import rospy
from geometry_msgs.msg import Point, PointStamped

def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))

def clamp_point(p, xlim, ylim, zlim):
    return Point(
        clamp(p.x, xlim[0], xlim[1]),
        clamp(p.y, ylim[0], ylim[1]),
        clamp(p.z, zlim[0], zlim[1]),
    )

def unit(vec):
    nx = math.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)
    if nx < 1e-9:
        return (1.0, 0.0, 0.0)
    return (vec[0]/nx, vec[1]/nx, vec[2]/nx)

def max_step_before_limits(origin, direction, xlim, ylim, zlim):
    """Distance max >0 le long de 'direction' depuis 'origin' avant de toucher les limites."""
    ux, uy, uz = direction
    s_candidates = []
    if abs(ux) > 1e-9:
        s = (xlim[1] - origin.x)/ux if ux > 0 else (xlim[0] - origin.x)/ux
        if s > 0: s_candidates.append(s)
    if abs(uy) > 1e-9:
        s = (ylim[1] - origin.y)/uy if uy > 0 else (ylim[0] - origin.y)/uy
        if s > 0: s_candidates.append(s)
    if abs(uz) > 1e-9:
        s = (zlim[1] - origin.z)/uz if uz > 0 else (zlim[0] - origin.z)/uz
        if s > 0: s_candidates.append(s)
    return min(s_candidates) if s_candidates else float('inf')

def safe_circle_radius(center, xlim, ylim, margin=0.01):
    """Rayon max pour rester dans la boîte XY autour de 'center'."""
    r = min(center.x - xlim[0], xlim[1] - center.x,
            center.y - ylim[0], ylim[1] - center.y) - margin
    return max(0.02, r)  # au moins 2 cm

def main():
    rospy.init_node("right_arm_sequence_publisher")
    elbow_pub = rospy.Publisher("right_arm/elbow", PointStamped, queue_size=1)
    wrist_pub = rospy.Publisher("right_arm/wrist", PointStamped, queue_size=1)

    rate = rospy.Rate(rospy.get_param("~rate", 50))  # Hz

    # Limites de travail "soft"
    xlim = (rospy.get_param("~x_min", 0.20), rospy.get_param("~x_max", 0.90))
    ylim = (rospy.get_param("~y_min", -0.40), rospy.get_param("~y_max", 0.40))
    zlim = (rospy.get_param("~z_min", 0.05), rospy.get_param("~z_max", 0.90))

    frame_id = rospy.get_param("~frame_id", "base_link")

    # Base du wrist (TCP) — point de départ / maintien initial
    wrist_base = Point(
        clamp(rospy.get_param("~wrist_base_x", 0.60), xlim[0], xlim[1]),
        clamp(rospy.get_param("~wrist_base_y", 0.00), ylim[0], ylim[1]),
        clamp(rospy.get_param("~wrist_base_z", 0.25), zlim[0], zlim[1]),
    )

    # Distance coude→poignet (avant-bras)
    ew_distance = float(rospy.get_param("~ew_distance", 0.32))  # m

    # Séquence temporelle
    hold_duration = float(rospy.get_param("~hold_duration", 3.0))     # s (phase 1)
    lift_distance = float(rospy.get_param("~lift_distance", 0.50))    # m (phase 2)
    lift_duration = float(rospy.get_param("~lift_duration", 2.0))     # s (phase 2)

    # Cercle (phase 3)
    circle_radius = float(rospy.get_param("~circle_radius", 0.10))    # m
    circle_frequency = float(rospy.get_param("~circle_frequency", 0.25))  # Hz

    # Pré-calculs
    t0 = rospy.Time.now().to_sec()
    margin = 1e-3

    # Applique la limite Z sur la montée
    max_lift = max(0.0, zlim[1] - wrist_base.z - margin)
    lift_applied = min(lift_distance, max_lift)
    circle_center = Point(wrist_base.x, wrist_base.y, wrist_base.z + lift_applied)

    # Rayon sûr
    circle_radius_eff = min(circle_radius, safe_circle_radius(circle_center, xlim, ylim))

    wrist_msg = PointStamped()
    elbow_msg = PointStamped()
    wrist_msg.header.frame_id = frame_id
    elbow_msg.header.frame_id = frame_id

    two_pi = 2.0 * math.pi

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        t = now.to_sec() - t0

        # ---- Phase 1: maintien ----
        if t < hold_duration:
            wrist = wrist_base

        # ---- Phase 2: montée lissée (min-jerk) ----
        elif t < hold_duration + lift_duration:
            tau = (t - hold_duration) / max(1e-6, lift_duration)
            s = tau*tau*(3.0 - 2.0*tau)  # 3-2-1 polynomial (C1)
            wrist = Point(wrist_base.x, wrist_base.y, wrist_base.z + s*lift_applied)

        # ---- Phase 3: cercle XY autour du point levé ----
        else:
            t_circ = t - (hold_duration + lift_duration)
            theta = two_pi * circle_frequency * t_circ
            wrist = Point(
                circle_center.x + circle_radius_eff * math.cos(theta),
                circle_center.y + circle_radius_eff * math.sin(theta),
                circle_center.z
            )

        # Clamp sécurité
        wrist = clamp_point(wrist, xlim, ylim, zlim)

        # Coude placé derrière le wrist le long de -X (distance ~ ew_distance)
        u_back = unit((-1.0, 0.0, 0.0))  # vecteur vers l'arrière
        s_max = max_step_before_limits(wrist, u_back, xlim, ylim, zlim)
        L = min(ew_distance, max(0.0, s_max - margin))
        elbow = Point(
            wrist.x + L * u_back[0],
            wrist.y + L * u_back[1],
            wrist.z + L * u_back[2],
        )

        # Publier
        wrist_msg.header.stamp = now
        elbow_msg.header.stamp = now
        wrist_msg.point = wrist
        elbow_msg.point = elbow
        wrist_pub.publish(wrist_msg)
        elbow_pub.publish(elbow_msg)

        rate.sleep()

if __name__ == "__main__":
    main()
