#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
UR30 (Gazebo) — Séquence verticale :
- Phase 1 : maintien à la position de base pendant ~hold_duration
- Phase 2 : montée verticale (profil lissé) de lift_distance en ~lift_duration
- Phase 3 : pause en haut (dwell_top)
- Phase 4 : descente verticale (profil lissé) en ~lift_duration
- Phase 5 : pause en bas (dwell_bottom)
→ Boucle sur les phases 2..5 tant que le nœud tourne.

Publie des PointStamped sur:
  - right_arm/elbow
  - right_arm/wrist
"""

import math
import rospy
from geometry_msgs.msg import Point, PointStamped

def clamp(v, vmin, vmax): return max(vmin, min(vmax, v))

def clamp_point(p, xlim, ylim, zlim):
    return Point(clamp(p.x, xlim[0], xlim[1]),
                 clamp(p.y, ylim[0], ylim[1]),
                 clamp(p.z, zlim[0], zlim[1]))

def unit(vec):
    n = math.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)
    return (1.0, 0.0, 0.0) if n < 1e-9 else (vec[0]/n, vec[1]/n, vec[2]/n)

def max_step_before_limits(origin, direction, xlim, ylim, zlim):
    ux, uy, uz = direction
    cand = []
    if abs(ux) > 1e-9:
        s = (xlim[1]-origin.x)/ux if ux>0 else (xlim[0]-origin.x)/ux
        if s>0: cand.append(s)
    if abs(uy) > 1e-9:
        s = (ylim[1]-origin.y)/uy if uy>0 else (ylim[0]-origin.y)/uy
        if s>0: cand.append(s)
    if abs(uz) > 1e-9:
        s = (zlim[1]-origin.z)/uz if uz>0 else (zlim[0]-origin.z)/uz
        if s>0: cand.append(s)
    return min(cand) if cand else float('inf')

def smoothstep01(tau):
    """Profil C1 (3-2-1) : accélère puis décélère en douceur, tau∈[0,1]."""
    tau = clamp(tau, 0.0, 1.0)
    return tau*tau*(3.0 - 2.0*tau)

def main():
    rospy.init_node("right_arm_updown_publisher")
    elbow_pub = rospy.Publisher("right_arm/elbow", PointStamped, queue_size=1)
    wrist_pub = rospy.Publisher("right_arm/wrist", PointStamped, queue_size=1)

    rate = rospy.Rate(rospy.get_param("~rate", 50))  # Hz

    # Boîte de travail "soft"
    xlim = (rospy.get_param("~x_min", 0.20), rospy.get_param("~x_max", 0.90))
    ylim = (rospy.get_param("~y_min", -0.40), rospy.get_param("~y_max", 0.40))
    zlim = (rospy.get_param("~z_min", 0.05), rospy.get_param("~z_max", 0.90))

    frame_id = rospy.get_param("~frame_id", "base_link")

    # Base du wrist (TCP)
    wrist_base = Point(
        clamp(rospy.get_param("~wrist_base_x", 0.60), xlim[0], xlim[1]),
        clamp(rospy.get_param("~wrist_base_y", 0.00), ylim[0], ylim[1]),
        clamp(rospy.get_param("~wrist_base_z", 0.25), zlim[0], zlim[1]),
    )

    # Distance coude→poignet
    ew_distance = float(rospy.get_param("~ew_distance", 0.32))  # m

    # Durées / amplitudes
    hold_duration  = float(rospy.get_param("~hold_duration", 3.0))   # s
    lift_distance  = float(rospy.get_param("~lift_distance", 0.50))  # m
    lift_duration  = float(rospy.get_param("~lift_duration", 2.0))   # s
    dwell_top      = float(rospy.get_param("~dwell_top", 0.5))       # s
    dwell_bottom   = float(rospy.get_param("~dwell_bottom", 0.5))    # s

    t0 = rospy.Time.now().to_sec()
    margin = 1e-3

    # Respect du plafond Z
    max_lift = max(0.0, zlim[1] - wrist_base.z - margin)
    lift_applied = min(lift_distance, max_lift)

    wrist_msg = PointStamped(); elbow_msg = PointStamped()
    wrist_msg.header.frame_id = frame_id; elbow_msg.header.frame_id = frame_id

    # Temps d’un cycle complet (haut+pause+bas+pause)
    cycle_T = max(1e-6, lift_duration + dwell_top + lift_duration + dwell_bottom)

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        t = now.to_sec() - t0

        # Phase 1 : maintien initial
        if t < hold_duration:
            z = wrist_base.z

        else:
            tc = t - hold_duration
            # Boucle sur le cycle
            tau = tc % cycle_T

            if tau < lift_duration:
                # Montée lissée
                s = smoothstep01(tau / lift_duration)
                z = wrist_base.z + s * lift_applied
            elif tau < lift_duration + dwell_top:
                # Palier en haut
                z = wrist_base.z + lift_applied
            elif tau < lift_duration + dwell_top + lift_duration:
                # Descente lissée
                tau_d = (tau - lift_duration - dwell_top) / lift_duration
                s = smoothstep01(tau_d)
                z = wrist_base.z + (1.0 - s) * lift_applied
            else:
                # Palier en bas
                z = wrist_base.z

        # TCP (X,Y fixes), Z selon la phase
        wrist = Point(wrist_base.x, z ,-wrist_base.y)
        wrist = clamp_point(wrist, xlim, ylim, zlim)

        # Coude placé derrière le wrist le long de -X
        u_back = unit((-1.0, 0.0, 0.0))
        s_max = max_step_before_limits(wrist, u_back, xlim, ylim, zlim)
        L = min(ew_distance, max(0.0, s_max - margin))
        elbow = Point(wrist.x + L*u_back[0], wrist.y + L*u_back[1], wrist.z + L*u_back[2])

        # Publier
        wrist_msg.header.stamp = now; wrist_msg.point = wrist
        elbow_msg.header.stamp = now; elbow_msg.point = elbow
        wrist_pub.publish(wrist_msg); elbow_pub.publish(elbow_msg)

        rate.sleep()

if __name__ == "__main__":
    main()
