#!/usr/bin/env python
import rospy
from moveit_commander import PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    rospy.init_node("add_floor_collision")
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    rospy.sleep(2)  # laisser le temps au PlanningScene de se connecter

    # Définition de la pose de la boîte (sol)
    floor_pose = PoseStamped()
    floor_pose.header.frame_id = robot.get_planning_frame()  # souvent "world" ou "base_link"
    floor_pose.pose.position.x = 0.0
    floor_pose.pose.position.y = 0.0
    floor_pose.pose.position.z = -0.005    # demi-épaiseur sous le plan z=0
    floor_pose.pose.orientation.w = 1.0

    # Taille : 10 m × 10 m × 0.01 m (épaisseur très fine)
    scene.add_box("floor", floor_pose, size=(10.0, 10.0, 0.01))

    rospy.loginfo("Sol ajouté au Planning Scene")

# --- Ajout d'un mur ---
    wall_pose = PoseStamped()
    wall_pose.header.frame_id = robot.get_planning_frame()
    # Positionnez le mur à y = +5 m (bord nord du sol), centré en x et à mi-hauteur
    wall_pose.pose.position.x = -0.5
    wall_pose.pose.position.y = 0
    wall_pose.pose.position.z = 1.5   # mi-hauteur pour un mur de 3 m

    # Orientation neutre (pas de rotation) : mur perpendiculaire à l'axe x
    wall_pose.pose.orientation.w = 1.0

    # Taille du mur : 10 m large (x), 0.01 m épais (y), 3 m haut (z)
    scene.add_box("wall", wall_pose, size=(0.01, 5, 3.0))
    rospy.loginfo("Mur ajouté au Planning Scene")    



    # --- Ajout d'un mur2 ---
    wall_pose = PoseStamped()
    wall_pose.header.frame_id = robot.get_planning_frame()
    # Positionnez le mur à y = +5 m (bord nord du sol), centré en x et à mi-hauteur
    wall_pose.pose.position.x = 0.59
    wall_pose.pose.position.y = 0.41
    wall_pose.pose.position.z = 0.16   # mi-hauteur pour un mur de 3 m

    # Orientation neutre (pas de rotation) : mur perpendiculaire à l'axe x
    wall_pose.pose.orientation.w = 1.0

    # Taille du mur : 10 m large (x), 0.01 m épais (y), 3 m haut (z)
    scene.add_box("wall2", wall_pose, size=(0.03, 0.03,0.32))
    rospy.loginfo("Mur2 ajouté au Planning Scene")    

    rospy.spin()
