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


    def transform_wrench_to_frame_alamain(self,wrench_msg):


        seuil_norme_q=0.1
        correction = R.from_euler('z', -np.pi / 2)

        tool_frame="fake_gripper"
        base_frame="base_link"
        rel_force_pub = rospy.Publisher('/rel_force', WrenchStamped, queue_size=1)

        try:
            transform = self.tf_buffer.lookup_transform(
                base_frame,
                tool_frame,
                wrench_msg.header.stamp,
                rospy.Duration(0.1)
            )

            # Récupère la rotation
            q = transform.transform.rotation
            # rospy.loginfo("Quaternion TF : [%.4f %.4f %.4f %.4f]", q.x, q.y, q.z, q.w)

            # à tester si il est pas cassé
            Rot_test = R.from_quat([q.x, q.y, q.z, q.w])

            # check pour la matrice 
            R_mat = Rot_test.as_dcm()

            if self.last_R_mat is not None:
                diff = np.linalg.norm(R_mat - self.last_R_mat)
                
                if diff <= seuil_norme_q:  # ← seuil pour les normes à tej
                    Rot=Rot_test
                    Rot = Rot*correction 
                    self.last_Rot=Rot
                    self.last_R_mat = R_mat
                     
                else : 
                    Rot=self.last_Rot

                    # rospy.logwarn(" Changement brutal de la matrice de rotation détecté (diff = %.4f)", diff)
            else :
                Rot=Rot_test*correction
                self.last_Rot=Rot
                self.last_R_mat=R_mat

    

            # Force et torque dans le repère capteur
            force = np.array([
                wrench_msg.wrench.force.x,
                wrench_msg.wrench.force.y,
                wrench_msg.wrench.force.z
            ])
            torque = np.array([
                wrench_msg.wrench.torque.x,
                wrench_msg.wrench.torque.y,
                wrench_msg.wrench.torque.z
            ])

            #déplacement capteur ----> manche
            d=np.array([0,0.05,0.07])
            torque_manche=torque + np.cross(d,force)


            # Transformation (rotation uniquement)
            force_trans = Rot.apply(force)
            torque_trans = Rot.apply(torque_manche)



            # Crée un nouveau message WrenchStamped
            new_wrench = WrenchStamped()
            new_wrench.header.stamp = wrench_msg.header.stamp
            new_wrench.header.frame_id = tool_frame
            new_wrench.wrench.force.x = force_trans[0]
            new_wrench.wrench.force.y = force_trans[1]
            new_wrench.wrench.force.z = force_trans[2]
            new_wrench.wrench.torque.x =torque_trans[0]
            new_wrench.wrench.torque.y =torque_trans[1]
            new_wrench.wrench.torque.z =torque_trans[2]

            # publication force réelle pour plot

            norme_F=np.linalg.norm(np.array([force_trans[0],force_trans[1],force_trans[2],torque_trans[0],torque_trans[1],torque_trans[2]]))
            msg_rel = WrenchStamped()
            msg_rel.header.stamp = rospy.Time.now()
            msg_rel.header.frame_id = "base_link"  # ou le frame que tu utilises
            msg_rel.wrench.force = Vector3(norme_F, force_trans[1], force_trans[2])
            msg_rel.wrench.torque = Vector3(torque_trans[0], torque_trans[1], torque_trans[2])
            rel_force_pub.publish(msg_rel)

            


            return new_wrench
        

        except Exception as e:
            rospy.logwarn("Erreur transformation explicite : %s", str(e))
            return None

    



    def admittance_openloop(self) :

        rospy.loginfo("Starting full-body admittance control (6D: position + orientation via torque)...")

        # Admittance parameters
        c=15
        
        M = 4; B = c*M; K = 0        # translation
        M_rot = 0.07; B_rot = c*M_rot ; K_rot = 0  # rotation
        # dt = 0.01  # 1/   Hz
        max_jerk_cart = 0.3      #limiter l'acceleration
        max_jerk_rot = 0.2
        force_dead_zone_cart = 0.3  # eviter de publier pour rien
        force_dead_zone_rot = 0.005
        F_alpha = 0.05  # filtre passe-bas exponentiel (proche de 0 = réponse lente mais beaucoup filtré)
        last_time=rospy.Time.now()
        joint_velocity_smoothed = None  # pour initialiser
        V_alpha = 0.12  # coefficient de lissage (plus petit = plus lisse)

    
        axes = ['x', 'y', 'z', 'rx', 'ry', 'rz']
        filtered_force = {axis: 0.0 for axis in axes}
        v_k = {axis: 0.0 for axis in axes}
        x_k = {axis: 0.0 for axis in axes}


        smoothed_rel = {axis: 0.0 for axis in axes}

        zero_force = None
        zero_force_buffer=[]

        

        joint_vel_pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)


        rospy.sleep(1.0)
        # rate = rospy.Rate(1.0 / dt)
        


        while not rospy.is_shutdown():
            current_time=rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            last_time = current_time


           

            if self.force_data is None:
                # rate.sleep()
                continue


            # Transform force into base_link
            wrench_global = self.transform_wrench_to_frame_alamain(self.force_data)
            if wrench_global is None:
                # rate.sleep()
                continue

            fx = wrench_global.wrench.force.x
            fy = wrench_global.wrench.force.y
            fz = wrench_global.wrench.force.z
            tx = wrench_global.wrench.torque.x
            ty = wrench_global.wrench.torque.y
            tz = wrench_global.wrench.torque.z

            current_force = {
                'x': fx, 'y': fy, 'z': fz,
                'rx': tx, 'ry': ty, 'rz': tz
            }

            # if zero_force is None:
            #     zero_force_buffer.append(current_force)
            #     if len(zero_force_buffer) >= nb_pour_moyenne_zero_force:
                    

            #         zero_force = {
            #             axis: np.median([f[axis] for f in zero_force_buffer])
            #             for axis in axes
            #         }
            #         rospy.loginfo("Offset calculé sur beaucoup de mesures : %s", zero_force)
            #         continue
            #     else:
            #         # rate.sleep()
            #         continue


            # Relative force + filtering
            for axis in axes:
                rel = current_force[axis] #- zero_force[axis]  #force relative

                #filtre passe bas exponentiel
                smoothed_rel[axis] = (1 - F_alpha) * smoothed_rel[axis] + F_alpha * rel



                filtered_force[axis] = smoothed_rel[axis]


            # Admittance dynamics

            # test nouvelle zone morte
            # if np.linalg.norm(np.array([filtered_force['x'], filtered_force['y'], filtered_force['z']])) < force_dead_zone_cart :
            #         filtered_force['x'] = 0.0
            #         filtered_force['y'] = 0.0
            #         filtered_force['z'] = 0.0
            # if np.linalg.norm(np.array([filtered_force['rx'], filtered_force['ry'], filtered_force['rz']])) < force_dead_zone_rot :
            #         filtered_force['rx'] = 0.0
            #         filtered_force['ry'] = 0.0
            #         filtered_force['rz'] = 0.0
            
            
                




            for axis in axes:

                
                if axis in ['x', 'y', 'z']:
                    M_, B_, K_ = M, B, K
                else:
                    M_, B_, K_ = M_rot, B_rot, K_rot

                a_k = (filtered_force[axis] - B_ * v_k[axis] - K_ * x_k[axis]) / M_
                # a_k[axis] = max(min(a_k[axis], max_jerk), -max_jerk)
                v_k[axis] += a_k * dt
                x_k[axis] += v_k[axis] * dt
                # v_k[axis] = max(min(v_k[axis], max_vel), -max_vel)
            

            

            try:
                joint_values = self.move_group.get_current_joint_values()
                jacobian = self.move_group.get_jacobian_matrix(joint_values)

                # Détection du repère de la Jacobienne
                jacobian_frame = self.move_group.get_pose_reference_frame()


                # Génère le vecteur vitesse dans base_link
                v_lin_base = np.array([v_k['x'], v_k['y'], v_k['z']])
                v_rot_base = np.array([v_k['rx'], v_k['ry'], v_k['rz']])

                # Transforme les vitesses si nécessaire
                if jacobian_frame != "base_link":
                    transform = self.tf_buffer.lookup_transform(jacobian_frame, "base_link", rospy.Time(0), rospy.Duration(0.1))
                    q = transform.transform.rotation
                    rotation = R.from_quat([q.x, q.y, q.z, q.w])
                    v_lin = rotation.apply(v_lin_base)
                    v_rot = rotation.apply(v_rot_base)
                else:
                    v_lin = v_lin_base
                    v_rot = v_rot_base


                ee_vel = np.concatenate([v_lin, v_rot])
                joint_velocities = np.linalg.pinv(jacobian).dot(ee_vel)

                
                

                # publication pour tracer pas mal de truc finalement 
                # norme_F=np.linalg.norm(np.array([filtered_force['x'],filtered_force['y'],filtered_force['z'],filtered_force['rx'], filtered_force['ry'], filtered_force['rz']]))
                # norme_F0=np.linalg.norm(np.array([zero_force['x'],zero_force['y'],zero_force['z'],zero_force['rx'], zero_force['ry'], zero_force['rz']]))

                # msg_filt = WrenchStamped()
                # msg_filt.header.stamp = rospy.Time.now()
                # msg_filt.header.frame_id = "base_link"  # ou le frame que tu utilises
                # msg_filt.wrench.force = Vector3(1/smoothed_dt, 0, filtered_force['z'])
                # msg_filt.wrench.torque = Vector3(filtered_force['rx'], filtered_force['ry'], filtered_force['rz'])
                # filtered_force_pub.publish(msg_filt)

                


                if joint_velocity_smoothed is None:
                    joint_velocity_smoothed = joint_velocities
                else:
                    joint_velocity_smoothed = V_alpha * joint_velocities + (1 - V_alpha) * joint_velocity_smoothed


                

                
                vel_msg= Float64MultiArray()
                vel_msg.data= joint_velocity_smoothed.tolist()
                joint_vel_pub.publish(vel_msg)
                
               



            except Exception as e:
                rospy.logwarn("Erreur dans le calcul ou la publication : %s", str(e))

            # rate.sleep()
          

    

def main():
    try:
        print("t'as pas le temps de lire de toute façon")
      
        tutorial = MoveGroupPythonInterfaceTutorial()

        # input("============ Press `Enter` to the initial pose ...")

        tutorial.switch_controllers(['scaled_pos_joint_traj_controller'], ['joint_group_vel_controller'])
        tutorial.go_to_pose_goal(-1,0.55,0.3)


        input("============ Press `Enter` to start the admittance ...")

        tutorial.switch_controllers(['joint_group_vel_controller'], ['scaled_pos_joint_traj_controller'])

        tutorial.admittance_openloop()


  

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

