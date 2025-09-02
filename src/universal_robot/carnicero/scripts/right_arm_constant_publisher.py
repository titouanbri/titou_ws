#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Publie des positions coude/poignet allant de gauche à droite avec un arrêt de 2 s à chaque extrémité.

Le mouvement est uniquement sur l'axe Y (REP-103 : +Y = gauche, -Y = droite).
"""

import rospy
from geometry_msgs.msg import Point, PointStamped


def main():
    rospy.init_node("right_arm_left_right_publisher")

    elbow_pub = rospy.Publisher("/right_arm/elbow", PointStamped, queue_size=1)
    wrist_pub = rospy.Publisher("/right_arm/wrist", PointStamped, queue_size=1)

    rate = rospy.Rate(50)  # Hz

    elbow_msg = PointStamped()
    wrist_msg = PointStamped()
    elbow_msg.header.frame_id = "base_link"
    wrist_msg.header.frame_id = "base_link"

    # Positions de base (x, y, z)
    elbow_base = Point(0.4, 0.0, 0.3)
    wrist_base = Point(0.6, 0.0, 0.2)

    # Paramètres (peuvent être surchargés via rosparam)
    amplitude = rospy.get_param("~amplitude", 0.20)     # déplacement latéral +/- en mètres
    dwell = rospy.get_param("~dwell_time", 10.0)         # temps d'arrêt à chaque extrémité (s)
    travel = rospy.get_param("~travel_time", 3.0)       # temps pour aller d'une extrémité à l'autre (s)

    # Pré-calculs
    y_left = +amplitude
    y_right = -amplitude
    cycle = 2 * dwell + 2 * travel
    t0 = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        t = now.to_sec() - t0
        phase = t % cycle

        # Offset Y avec plateaux de 2 s aux extrémités (rampe linéaire entre les deux)
        if phase < dwell:
            y_off = y_left
        elif phase < dwell + travel:
            tau = (phase - dwell) / travel
            y_off = y_left + (y_right - y_left) * tau
        elif phase < dwell + travel + dwell:
            y_off = y_right
        else:
            tau = (phase - (dwell + travel + dwell)) / travel
            y_off = y_right + (y_left - y_right) * tau

        elbow_msg.header.stamp = now
        wrist_msg.header.stamp = now

        # Mouvement gauche-droite uniquement sur Y (X et Z constants)
        elbow_msg.point.x = elbow_base.x
        elbow_msg.point.y = elbow_base.y + y_off
        elbow_msg.point.z = elbow_base.z

        wrist_msg.point.x = wrist_base.x
        wrist_msg.point.y = wrist_base.y + y_off
        wrist_msg.point.z = wrist_base.z

        elbow_pub.publish(elbow_msg)
        wrist_pub.publish(wrist_msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
