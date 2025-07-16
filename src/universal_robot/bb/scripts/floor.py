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
    rospy.spin()
