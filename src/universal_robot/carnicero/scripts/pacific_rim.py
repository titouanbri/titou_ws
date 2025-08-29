#!/usr/bin/env python
from __future__ import print_function
from six.moves import input
import numpy as np
import sys
import copy
import rospy
import geometry_msgs.msg
from ur_msgs.srv import SetIO  
from geometry_msgs.msg import WrenchStamped  
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
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
import PyKDL
from kdl_parser_py.urdf import treeFromUrdfModel
import subprocess
from std_msgs.msg import String
import os
import serial
import time
from math import sqrt
from geometry_msgs.msg import PointStamped



class admittance_control(object):
    def __init__(self):
        rospy.init_node("admittance", anonymous=True)

        group_name = "manipulator"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.force_data = None 
        self.joint_state = None


        rospy.Subscriber('/right_arm/elbow', PointStamped, self.elbow_real_callback) # sensor used to acquire forces 
        rospy.Subscriber('/right_arm/wrist', PointStamped, self.wrist_real_callback) # sensor used to acquire forces 
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)



        self.last_R_mat_test=None
        self.last_Rot=None


        self.tool_frame="tool0"
        self.base_frame="base_link"

        robot_description = rospy.get_param("/robot_description")
        robot = URDF.from_xml_string(robot_description)
        ok, tree = treeFromUrdfModel(robot)
        chain = tree.getChain("base_link", self.tool_frame)  

        self.kdl_chain = chain
        self.kdl_jnt_to_jac_solver = PyKDL.ChainJntToJacSolver(chain)


        
        self.kdl_fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)

        self.correction = R.from_euler('z', -np.pi / 2)  # constant correction for the sensor frame (only for the rokubi serial and ethercat)

        
        self.joint_frames = [     # joint frame names (not used but useful to know)
        'base_link',
        'shoulder_link',
        'forearm_link',
        'wrist_1_link',
        'wrist_2_link',
        'wrist_3_link',
         ]
        
        self.button=True

        self.v_k = np.zeros(6)
        self.x_k = np.zeros(6)
        self.a_k = np.zeros(6)
        self.last_wrist_real=np.zeros(7)
        self.last_elbow_real=np.zeros(7)
        self.wrist_real=None
        self.elbow_real=None
        self.offset=np.zeros(3)
        self.buffer_offset=[]

    def wrist_real_callback(self, msg):
        self.wrist_real = np.array([msg.point.x,-msg.point.z,msg.point.y])


    def elbow_real_callback(self, msg):
        self.elbow_real = np.array([msg.point.x,-msg.point.z,msg.point.y])

    def joint_state_callback(self, msg):
        self.joint_state = msg



    def init_offset(self):
        i=0

        while i<5:
            if self.wrist_real is None:
                continue
            self.buffer_offset.append(-self.wrist_real+self.get_joint_positions("wrist_3_link")[:3])
            i+=1

        P=np.vstack(self.buffer_offset)
        self.offset=P.mean(axis=0)
        print(self.offset)


    #function to be sure that the correct controller is launched 
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


    def get_joint_positions(self,joint_frame):
        
        
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.base_frame,    
                joint_frame,              
                rospy.Time(0),
                rospy.Duration(0.1)
            )
            t = tf_stamped.transform.translation
            r = tf_stamped.transform.rotation
            positions = np.array([t.x, t.y, t.z, r.x, r.y, r.z, r.w])
        except Exception as e:
            rospy.logwarn(f"Impossible d'obtenir {joint_frame} : {e}")
        return positions





    def quaternion_orientation_BA(self,A, B, axe_reference="z"):
        """
        Retourne le quaternion unitaire (w, x, y, z) qui aligne l'axe choisi (+X/+Y/+Z)
        sur le vecteur BA (de B vers A). Rotation minimale (roll non contraint).
        
        A, B : itérables (x, y, z)
        axe_reference : 'x', 'y' ou 'z'
        """
        from math import sqrt

        # -- Utilitaires locaux
        def norm(v): return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
        def dot(a,b): return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
        def cross(a,b): 
            return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
        def normalize(v):
            n = norm(v)
            if n < 1e-12:
                raise ValueError("Vecteur nul : les points A et B sont confondus.")
            return (v[0]/n, v[1]/n, v[2]/n)

        # -- Préparation des données
        A = tuple(float(x) for x in A)
        B = tuple(float(x) for x in B)
        BA = (A[0]-B[0], A[1]-B[1], A[2]-B[2])
        b = normalize(BA)

        if axe_reference == "x":
            a = (1.0, 0.0, 0.0)
        elif axe_reference == "y":
            a = (0.0, 1.0, 0.0)
        elif axe_reference == "z":
            a = (0.0, 0.0, 1.0)
        else:
            raise ValueError("axe_reference doit être 'x', 'y' ou 'z'.")

        # -- Cas numériques
        eps = 1e-8
        c = max(-1.0, min(1.0, dot(a, b)))  # clamp du cosinus

        # Vecteurs presque identiques -> pas de rotation
        if c > 1.0 - eps:
            return (1.0, 0.0, 0.0, 0.0)

        # Vecteurs presque opposés -> rotation de 180° autour d'un axe orthogonal
        if c < -1.0 + eps:
            # Choisir un vecteur non-colinéaire pour construire un axe
            pick = (1.0, 0.0, 0.0) if abs(a[0]) < 0.9 else (0.0, 1.0, 0.0)
            axis = cross(a, pick)
            n = norm(axis)
            if n < eps:  # secours au cas très dégénéré
                pick = (0.0, 0.0, 1.0)
                axis = cross(a, pick)
                n = norm(axis)
            axis = (axis[0]/n, axis[1]/n, axis[2]/n)
            return (0.0, axis[0], axis[1], axis[2])

        # -- Cas général : quaternion via demi-angle
        axis = cross(a, b)
        w = 1.0 + c
        q0, q1, q2, q3 = w, axis[0], axis[1], axis[2]

        # Normalisation du quaternion
        qn = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        return np.array([q0/qn, q1/qn, q2/qn, q3/qn])
    

    def point_quat_to_euler(self,point, seq='xyz', degrees=False, quat_format='xyzw'):
        """
        Convertit un point (x, y, z, q) -> (x, y, z, euler).
        
        Parameters
        ----------
        point : np.ndarray shape (7,)
            [x, y, z, q1, q2, q3, q4]
        seq : str, default 'xyz'
            Séquence d'Euler (ex. 'xyz', 'zyx', ...). 
            Requiert SciPy sauf pour le repli 'xyz' (roll-pitch-yaw).
        degrees : bool, default False
            True -> angles en degrés, False -> radians.
        quat_format : {'xyzw','wxyz'}, default 'xyzw'
            Ordre des composantes du quaternion passé dans `point`.
        Returns
        -------
        np.ndarray shape (6,)
            [x, y, z, a1, a2, a3] dans l'ordre de `seq`.
        """
        p = np.asarray(point, dtype=float)
        if p.shape != (7,):
            raise ValueError("`point` doit être un np.array de taille 7 (x,y,z + quaternion).")

        x, y, z = p[:3]
        q = p[3:]

        if quat_format == 'xyzw':
            qx, qy, qz, qw = q
        elif quat_format == 'wxyz':
            qw, qx, qy, qz = q
        else:
            raise ValueError("quat_format doit être 'xyzw' ou 'wxyz'.")

        # Normalisation (sécurité numérique)
        norm = np.linalg.norm([qw, qx, qy, qz])
        if not np.isfinite(norm) or norm <= 1e-12:
            raise ValueError("Quaternion de norme quasi nulle ou invalide.")
        qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm

        # Tentative avec SciPy si dispo (support complet des séquences)
        try:
            from scipy.spatial.transform import Rotation as R
            r = R.from_quat([qx, qy, qz, qw])  # SciPy attend (x, y, z, w)
            angles = r.as_euler(seq, degrees=degrees)
        except Exception:
            # Repli sans SciPy : roll-pitch-yaw (X-Y-Z) uniquement, équivalent à une décomposition ZYX (yaw-pitch-roll)
            if seq.lower() != 'xyz':
                raise ImportError(
                    "SciPy non disponible : le repli ne supporte que seq='xyz' (roll-pitch-yaw). "
                    "Installe scipy pour d'autres séquences."
                )
            # Formules standard (radians)
            sinr_cosp = 2 * (qw * qx + qy * qz)
            cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
            roll = np.arctan2(sinr_cosp, cosr_cosp)

            sinp = 2 * (qw * qy - qz * qx)
            if abs(sinp) >= 1:
                pitch = np.pi / 2 * np.sign(sinp)  # verrouillage à ±90°
            else:
                pitch = np.arcsin(sinp)

            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            yaw = np.arctan2(siny_cosp, cosy_cosp)

            angles = np.array([roll, pitch, yaw])
            if degrees:
                angles = np.degrees(angles)

        return np.array([x, y, z, *angles])





    def admittance_openloop(self) :

        rospy.loginfo("Starting full-body admittance control (6D: position + orientation via torque)...")

        # Admittance parameters
        frequence=350 #Hz   be sure that this frequency is stable on your computer, use "rostopic hz /topic_name"
        dt =1/frequence
        joint_velocity_smoothed = None  
        V_alpha = 0.01  # velocity exponential low-pass filter (close to 0 = slow response but strong filtering)

    
        filtered_force = np.zeros(6)
        smoothed_rel = np.zeros(6)

        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        vel_msg= Float64MultiArray()
        vel_msg.data=[0,0,0,0,0,0]
        lambda_dls = 0.015   # damping of the close singularity accelerations : Higher value = more damping, less speed
        Kp = 2     # 1/s  (XYZ | RPY)
        Ki = 0.1   # 1/s^2
        Kd =0.6   # -
        eps=0.1
        last_x_real=np.zeros(7)
        



        

        joint_vel_pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)


        rospy.sleep(1.0)
        rate = rospy.Rate(frequence)
        


        while not rospy.is_shutdown() : #and self.safety():


            if self.joint_state is None:
                rate.sleep()
                continue  

            #joint positions
            joint_values = [self.joint_state.position[self.joint_state.name.index(jn)] for jn in joint_names]  #arange l'ordre des joint
            joint_values_np = np.array(joint_values)

            
            # computing the jacobian
            jnt_array = PyKDL.JntArray(len(joint_values))
            for i, pos in enumerate(joint_values):
                jnt_array[i] = pos

            fk_frame = PyKDL.Frame()
            self.kdl_fk_solver.JntToCart(jnt_array, fk_frame)
            jacobian = PyKDL.Jacobian(len(joint_values))
            self.kdl_jnt_to_jac_solver.JntToJac(jnt_array, jacobian)
            jacobian = np.array([[jacobian[i, j] for j in range(jacobian.columns())] for i in range(6)])



            wrist_r=self.quaternion_orientation_BA(self.wrist_real,self.elbow_real)

            #position mesuré / cible
            x_real=self.get_joint_positions("wrist_3_link")
          
            x_ref=np.concatenate((self.wrist_real+self.offset,wrist_r))


            e=x_ref-x_real
            

            v=(x_real-last_x_real)/dt

            v_cmd=Kp*e+Ki*e*dt-Kd*v

            last_x_real=x_real

            ee_vel = self.point_quat_to_euler(v_cmd,quat_format='wxyz')
            print(ee_vel)

            #damping of singularity accelerations 

            U, S, Vt = np.linalg.svd(jacobian, full_matrices=False)
            S_damped = S / (S**2 + lambda_dls**2)
            J_pinv_dls = Vt.T.dot(np.diag(S_damped)).dot(U.T)
            joint_velocities = J_pinv_dls.dot(ee_vel)



            #Exponential filter on the velocity to be published
            if joint_velocity_smoothed is None:
                joint_velocity_smoothed = joint_velocities
            else:
                joint_velocity_smoothed = V_alpha * joint_velocities + (1 - V_alpha) * joint_velocity_smoothed


            

            
            vel_msg.data= joint_velocity_smoothed.tolist()
            joint_vel_pub.publish(vel_msg)

            rospy.loginfo_throttle(0.5, "x_robot: %s | wrist_real: %s", x_real[:3], x_ref[:3])


                
               



            rate.sleep()
        #clean stop with the button
        values=np.array(vel_msg.data)
        for _ in range (100) :
            values= values/1.03               
            vel_msg.data=values.tolist()
            joint_vel_pub.publish(vel_msg)
            rospy.sleep(0.0025)
            

        vel_msg.data=[0,0,0,0,0,0]
        joint_vel_pub.publish(vel_msg)
        
        
def main():
    try:
        print("node initialisation")

        admittance=admittance_control()
        
        #controller switch (as a precaution)
        admittance.switch_controllers(['joint_group_vel_controller'], ['scaled_pos_joint_traj_controller'])
        print("init de l'offset")
        admittance.init_offset()
        print("c'est bon")

        #launching the admittance
        admittance.admittance_openloop()

        print("end of the admittance")
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()


          

    
