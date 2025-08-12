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


#teensy initialisation (if using the teensy)

# PORT = '/dev/ttyACM0'   
# BAUD = 9600
# try:
#     ser = serial.Serial(PORT, BAUD, timeout=0)
#     print(f"Connecté à {PORT} à {BAUD} bauds.")
#     time.sleep(2)  
# except serial.SerialException as e:
#     print(f"Erreur d'ouverture du port série : {e}")
    



class admittance_control(object):
    def __init__(self):
        rospy.init_node("sensor_test", anonymous=True)

        group_name = "manipulator"

        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.force_data = None  # pour stocker la dernière donnée
        self.joint_state = None

        rospy.Subscriber('/force_sensor_eth', WrenchStamped, self.force_callback) # sensor used to acquire forces 
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)



        self.last_R_mat_test=None
        self.last_Rot=None

        # for time logging

        #variable pour changer de robot
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


    def force_callback(self, msg):
        self.force_data = msg

    def joint_state_callback(self, msg):
        self.joint_state = msg




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
                self.base_frame,    # repère absolu
                joint_frame,              # repère de l’articulation
                rospy.Time(0),
                rospy.Duration(0.1)
            )
            t = tf_stamped.transform.translation
            positions = np.array([t.x, t.y, t.z])
        except Exception as e:
            rospy.logwarn(f"Impossible d'obtenir {joint_frame} : {e}")
        return positions

    # function for spacial safeties, you can adapt it with your environment values. Disabled by default, to activate it uncomment the good lign in the loop 
    def safety(self):
        limite_basse_wrist=0.20
        limite_basse_elbow=0.15
        limite_mur=-0.10
        wrist= self.get_joint_positions("wrist_1_link")[2] > limite_basse_wrist
        elbow= self.get_joint_positions("forearm_link")[2] > limite_basse_elbow
        elbow1= self.get_joint_positions("forearm_link")[0] > limite_mur 
        wrist1= self.get_joint_positions("wrist_1_link")[0] > limite_mur
        return (wrist and elbow and elbow1 and wrist1)

    
    def transform_wrench_to_frame_alamain(self,wrench_msg):
        #transform the forces from the sensor frame to the robot base frame
        #there is also a test on the quaternions, because some of them are broken and can disturb the admittance 

        
        threshold_norme_q=0.1
        tool_frame=self.tool_frame
        base_frame=self.base_frame

       

        try:
            transform = self.tf_buffer.lookup_transform(
                base_frame,
                tool_frame,
                rospy.Time(0),  
                rospy.Duration(0.01)
            )
            
            

            q = transform.transform.rotation

            Rot_test = R.from_quat([q.x, q.y, q.z, q.w])
            R_mat_test = Rot_test.as_dcm()

            

            if self.last_R_mat_test is not None:
                diff = np.linalg.norm(R_mat_test - self.last_R_mat_test)
                
                if diff <= threshold_norme_q:  # Threshold for norms (if the difference between two consecutive ones is too high, it is ignored as broken)
                    Rot=Rot_test
                    Rot = Rot*self.correction 
                    self.last_Rot=Rot
                    self.last_R_mat_test = R_mat_test
                     
                else : 
                    Rot=self.last_Rot
                    R_mat_test=self.last_R_mat_test

            else :
                Rot=Rot_test*self.correction
                self.last_Rot=Rot
                self.last_R_mat_test=R_mat_test


    

            # Force and torque in sensor frame 
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

            #torques displacement sensor ----> handle
            d=np.array([0,0.05,0.07])
            torque_manche=torque + np.cross(d,force)

            Rot_qui_fonctionne=Rot.as_dcm()
            force_trans = np.dot(Rot_qui_fonctionne,force) 
            torque_trans = np.dot(Rot_qui_fonctionne,torque_manche)
            
            
            # Crée un nouveau message WrenchStamped
            new_wrench = WrenchStamped()
            new_wrench.header.stamp = wrench_msg.header.stamp
            new_wrench.header.frame_id = self.tool_frame
            new_wrench.wrench.force.x = force_trans[0]
            new_wrench.wrench.force.y = force_trans[1]
            new_wrench.wrench.force.z = force_trans[2]
            new_wrench.wrench.torque.x =torque_trans[0]
            new_wrench.wrench.torque.y =torque_trans[1]
            new_wrench.wrench.torque.z =torque_trans[2]

            return new_wrench
        

        except Exception as e:
            rospy.logwarn("Explicit transformation error : %s", str(e))
            return None





    # use of the teensy to use button which can stop/start the admittance

    # def PRESS(self):
    #     line = ser.readline().decode('utf-8', errors='ignore').strip()
    #     if line == "PRESS":
    #         # on bascule l'état
    #         self.button = not self.button
    #         print(f"→ Nouvel état : {'ACTIF' if self.button else 'INACTIF'}")
            
    #     return self.button
        

    def admittance_openloop(self) :

        rospy.loginfo("Starting full-body admittance control (6D: position + orientation via torque)...")

        # Admittance parameters
        c=13
        M = 6; B = c*M; K = 0        # translation
        M_rot = 0.06; B_rot = c*M_rot ; K_rot = 0  # rotation



        frequence=350 #Hz       be sure that this frequency is stable on your computer, use "rostopic hz /topic_name"
        dt =1/frequence
        force_dead_zone_cart = 0.05  # avoid useless publishment
        force_dead_zone_rot = 0.003
        F_alpha = 0.01  # force exponential low-pass filter (close to 0 = slow response but strong filtering)
        joint_velocity_smoothed = None  
        V_alpha = 0.15  # velocity exponential low-pass filter (close to 0 = slow response but strong filtering)

    
        filtered_force = np.zeros(6)
        smoothed_rel = np.zeros(6)

        B_mat=np.array([B,B,B,B_rot,B_rot,B_rot])
        K_mat=np.array([K,K,K,K_rot,K_rot,K_rot])
        M_mat=np.array([M,M,M,M_rot,M_rot,M_rot])
        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        vel_msg= Float64MultiArray()
        vel_msg.data=[0,0,0,0,0,0]

        lambda_dls = 0.015   # damping of the close singularity accelerations : Higher value = more damping, less speed

        



        
        joint_vel_pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)


        rospy.sleep(1.0)
        rate = rospy.Rate(frequence)
        


        while not rospy.is_shutdown() :   #and self.safety():     

            # #boutton
            # if  not self.PRESS():
            #     values=np.array(vel_msg.data)
            #     for _ in range (100) :
            #         values= values/1.03              
            #         vel_msg.data=values.tolist()
            #         joint_vel_pub.publish(vel_msg)
            #         rospy.sleep(0.0025)
                    

            #     vel_msg.data=[0,0,0,0,0,0]
            #     joint_vel_pub.publish(vel_msg)
            #     self.v_k = np.zeros(6)
            #     self.x_k = np.zeros(6)
            #     self.a_k = np.zeros(6)
            #     joint_velocity_smoothed = None  
            #     filtered_force = np.zeros(6)
            #     smoothed_rel = np.zeros(6)
            #     continue

            if self.force_data is None:
                rate.sleep()
                continue 

            if self.joint_state is None:
                rate.sleep()
                continue  
        
                



            # Transform force into base_link
            wrench_global = self.transform_wrench_to_frame_alamain(self.force_data)
            if wrench_global is None:
                rate.sleep()
                continue

            
            #force from force_sensor_publisher
            fx = wrench_global.wrench.force.x
            fy = wrench_global.wrench.force.y
            fz = wrench_global.wrench.force.z
            tx = wrench_global.wrench.torque.x
            ty = wrench_global.wrench.torque.y
            tz = wrench_global.wrench.torque.z
            current_force = np.array([fx,fy,fz,tx,ty,tz])

            #joint positions
            joint_values = [self.joint_state.position[self.joint_state.name.index(jn)] for jn in joint_names]  #arange l'ordre des joint
            joint_values_np = np.array(joint_values)


            # force exponential low-pass filter
            smoothed_rel = (1 - F_alpha) * smoothed_rel + F_alpha * current_force



            filtered_force = smoothed_rel



            # test dead zone 
            if np.linalg.norm(filtered_force[:3]) < force_dead_zone_cart :
                    filtered_force[0] = 0.0
                    filtered_force[1] = 0.0
                    filtered_force[2] = 0.0
            if np.linalg.norm(filtered_force[3:]) < force_dead_zone_rot :
                    filtered_force[3] = 0.0
                    filtered_force[4] = 0.0
                    filtered_force[5] = 0.0
            
            
            
            # computing the jacobian
            jnt_array = PyKDL.JntArray(len(joint_values))
            for i, pos in enumerate(joint_values):
                jnt_array[i] = pos
            fk_frame = PyKDL.Frame()
            self.kdl_fk_solver.JntToCart(jnt_array, fk_frame)
            jacobian = PyKDL.Jacobian(len(joint_values))
            self.kdl_jnt_to_jac_solver.JntToJac(jnt_array, jacobian)
            jacobian = np.array([[jacobian[i, j] for j in range(jacobian.columns())] for i in range(6)])


                

            # Admittance dynamics
            self.a_k = (filtered_force - B_mat* self.v_k - K_mat * self.x_k) /M_mat      
            self.v_k += self.a_k * dt
            self.x_k = self.v_k*dt   #useless because k=0 in this case 


            ee_vel = self.v_k


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



                
               



            rate.sleep()


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

        # useful is you use a special publisher for the forces, to be sure that it's closed before launching it
        os.system("pkill -f force_sensor_eth_publisher.py") 
        admittance=admittance_control()


        # launching the force publisher 
        subprocess.Popen(["rosrun", "carnicero", "force_sensor_eth_publisher.py"])  

        #controller switch (as a precaution)
        admittance.switch_controllers(['joint_group_vel_controller'], ['scaled_pos_joint_traj_controller'])


        #launching the admittance
        admittance.admittance_openloop()

        print("end of the admittance")
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()


          

    
