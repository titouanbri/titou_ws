#!/usr/bin/env python
from __future__ import print_function
from six.moves import input
import numpy as np
import sys
import copy
import rospy
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
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
import PyKDL
from kdl_parser_py.urdf import treeFromUrdfModel
import subprocess
from std_msgs.msg import String
import os
import serial
import time


#initialisation de la teensy 
PORT = '/dev/ttyACM0'
BAUD = 9600
try:
    ser = serial.Serial(PORT, BAUD, timeout=0)
    print(f"Connecté à {PORT} à {BAUD} bauds.")
    time.sleep(2)  # laisse la ligne série se stabiliser
except serial.SerialException as e:
    print(f"Erreur d'ouverture du port série : {e}")
    



class admittance_control(object):
    def __init__(self):
        rospy.init_node("sensor_test", anonymous=True)

        group_name = "manipulator"

        self.rel_force_pub = rospy.Publisher('/rel_force', WrenchStamped, queue_size=1)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.force_data = None  # pour stocker la dernière donnée
        self.joint_state = None

        rospy.Subscriber('/force_sensor', WrenchStamped, self.force_callback)
        rospy.Subscriber('/force_sensor_eth', WrenchStamped, self.force_callback_eth)
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)



        self.last_R_mat_test=None
        self.last_Rot=None

        # for time logging
        self.timer_name =[]
        self.timer_value =[]

        #variable pour changer de robot
        self.tool_frame="tool0"
        self.base_frame="base_link"

        robot_description = rospy.get_param("/robot_description")
        robot = URDF.from_xml_string(robot_description)
        ok, tree = treeFromUrdfModel(robot)
        chain = tree.getChain("base_link", self.tool_frame)  # adapte ces noms à ton robot

        self.kdl_chain = chain
        self.kdl_jnt_to_jac_solver = PyKDL.ChainJntToJacSolver(chain)


        # --- NOUVEAU : solveur FK pour récupérer la position réelle
        self.kdl_fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)

        self.correction = R.from_euler('z', -np.pi / 2)  #correction constante pour le capteur
        self.cg_history = deque(maxlen=100)  # Moyenne glissante sur 10 valeurs pour le centre de gravité 

        
        self.joint_frames = [     #nom des repères des joints
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




    def timer_init(self):
        # self.timer_name =["init"]
        # self.timer_value =[rospy.Time.now()]
        pass
        
    
    def timer_add(self, name):
        # self.timer_name.append(name)
        # self.timer_value.append(rospy.Time.now())
        pass
    
    def timer_info(self):
        # total_time = self.timer_value[-1] - self.timer_value[0]
        # total_hz = 1.0/total_time.to_sec()
        # sentence = []
        # for i in range(1,len(self.timer_value)):
        #     sentence.append(self.timer_name[i])
        #     sentence.append(str(round((100*(self.timer_value[i]-self.timer_value[i-1])/total_time),2))) 
        # rospy.loginfo_throttle(0.1,"\n".join(sentence)+ f" \n temp total : {round(total_hz,2)} Hz")
        pass

    def force_callback(self, msg):
        self.force_data = msg

    def force_callback_eth(self, msg):
        self.force_data_eth = msg

    def joint_state_callback(self, msg):
        self.joint_state = msg

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
            # lookup_transform(source, target, time, timeout)
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


    def safety(self):
        limite_basse_wrist=0.20
        limite_basse_elbow=0.15
        limite_mur=-0.10
        wrist= self.get_joint_positions("wrist_1_link")[2] > limite_basse_wrist
        elbow= self.get_joint_positions("forearm_link")[2] > limite_basse_elbow
        elbow1= self.get_joint_positions("forearm_link")[0] > limite_mur 
        wrist1= self.get_joint_positions("wrist_1_link")[0] > limite_mur
        return (wrist and elbow and elbow1 and wrist1)
        
    def In_Meat(self):
        #utilisation de l'ethercat pour savoir si on est dans la viande
        seuil = 3
        data = np.array([
            self.force_data_eth.wrench.force.x,
            self.force_data_eth.wrench.force.y,
            self.force_data_eth.wrench.force.z,
            # self.force_data_eth.wrench.torque.x,
            # self.force_data_eth.wrench.torque.y,
            # self.force_data_eth.wrench.torque.z,
        ])
        norme=np.linalg.norm(data)
        
        return norme > seuil

    def admittance_dyn(self):
        data = np.array([
            self.force_data_eth.wrench.force.x,
            self.force_data_eth.wrench.force.y,
            self.force_data_eth.wrench.force.z,
            # self.force_data_eth.wrench.torque.x,
            # self.force_data_eth.wrench.torque.y,
            # self.force_data_eth.wrench.torque.z,
        ])
        norme=np.linalg.norm(data)
        M=np.log(norme**2)+5
        B=15*M
        return None

    def transform_wrench_to_frame_alamain(self,wrench_msg):

        
        seuil_norme_q=0.1
        tool_frame=self.tool_frame
        base_frame=self.base_frame

       

        try:
            transform = self.tf_buffer.lookup_transform(
                base_frame,
                tool_frame,
                rospy.Time(0),  #wrench_msg.header.stamp
                rospy.Duration(0.01)
            )
            
            

            # Récupère la rotation
            q = transform.transform.rotation
            # rospy.loginfo("Quaternion TF : [%.4f %.4f %.4f %.4f]", q.x, q.y, q.z, q.w)

            # à tester si il est pas cassé
            Rot_test = R.from_quat([q.x, q.y, q.z, q.w])

            # check pour la matrice 
            R_mat_test = Rot_test.as_dcm()

            

            if self.last_R_mat_test is not None:
                diff = np.linalg.norm(R_mat_test - self.last_R_mat_test)
                
                if diff <= seuil_norme_q:  # ← seuil pour les normes à tej
                    Rot=Rot_test
                    Rot = Rot*self.correction 
                    self.last_Rot=Rot
                    self.last_R_mat_test = R_mat_test
                     
                else : 
                    Rot=self.last_Rot
                    R_mat_test=self.last_R_mat_test

                    # rospy.logwarn(" Changement brutal de la matrice de rotation détecté (diff = %.4f)", diff)
            else :
                Rot=Rot_test*self.correction
                self.last_Rot=Rot
                self.last_R_mat_test=R_mat_test


    

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
            # force_trans = Rot.apply(force)
            # torque_trans = Rot.apply(torque_manche)
            Rot_qui_fonctionne_inshallah=Rot.as_dcm()
            force_trans = np.dot(Rot_qui_fonctionne_inshallah,force) 
            torque_trans = np.dot(Rot_qui_fonctionne_inshallah,torque_manche)
            


            #estimer centre gravité
            # F_norm_sq = np.dot(force_trans, force_trans)

            # if F_norm_sq > 1e-6:  # éviter la division par zéro
            #     cg_vector = np.cross(force_trans, torque_trans) / F_norm_sq
            #     self.cg_history.append(cg_vector)
            #     cg_avg = np.mean(self.cg_history, axis=0)
            #     self.cg_estimate = cg_avg
            #     rospy.loginfo_throttle(1.0, f"CG (filtré): {cg_avg}")

                
            # else:
            #     rospy.logwarn_throttle(1.0, "Norme force trop faible pour estimer le CG")


            
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

            # publication force réelle pour plot

            # norme_F=np.linalg.norm(np.array([force_trans[0],force_trans[1],force_trans[2],torque_trans[0],torque_trans[1],torque_trans[2]]))
            # msg_rel = WrenchStamped()
            # msg_rel.header.stamp = rospy.Time.now()
            # msg_rel.header.frame_id = "base_link"  # ou le frame que tu utilises
            # msg_rel.wrench.force = Vector3(norme_F, force_trans[1], force_trans[2])
            # msg_rel.wrench.torque = Vector3(torque_trans[0], torque_trans[1], torque_trans[2])
            # self.rel_force_pub.publish(msg_rel)

            




            return new_wrench
        

        except Exception as e:
            rospy.logwarn("Erreur transformation explicite : %s", str(e))
            return None

    def PRESS(self):
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line == "PRESS":
            # on bascule l'état
            self.button = not self.button
            print(f"→ Nouvel état : {'ACTIF' if self.button else 'INACTIF'}")
            
        return self.button
        

    def admittance_openloop(self) :

        rospy.loginfo("Starting full-body admittance control (6D: position + orientation via torque)...")

        # Admittance parameters
        c=15
        M = 3.5; B = c*M; K = 0        # translation
        M_rot = 0.07; B_rot = c*M_rot ; K_rot = 0  # rotation

        #adimttance paramters in meat IM
        M_IM=10 ; M_rot_IM=0.12


        frequence=400 #Hz
        dt =1/frequence
        max_jerk_cart = 0.3      #limiter l'acceleration
        max_jerk_rot = 0.2
        force_dead_zone_cart = 0.05  # eviter de publier pour rien
        force_dead_zone_rot = 0.003
        F_alpha = 0.02  # filtre passe-bas exponentiel (proche de 0 = réponse lente mais beaucoup filtré)
        last_time=rospy.Time.now()
        joint_velocity_smoothed = None  # pour iget_joint_positions("wrist_3_link")[2]>limite_bassenitialiser
        V_alpha = 0.06  # coefficient de lissage (plus petit = plus lisse)

    
        filtered_force = np.zeros(6)
        smoothed_rel = np.zeros(6)

        B_mat=np.array([B,B,B,B_rot,B_rot,B_rot])
        K_mat=np.array([K,K,K,K_rot,K_rot,K_rot])
        M_mat=np.array([M,M,M,M_rot,M_rot,M_rot])
        M_mat_IM=np.array([M_IM,M_IM,M_IM,M_rot_IM,M_rot_IM,M_rot_IM])
        B_mat_IM=c*M_mat_IM
        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        vel_msg= Float64MultiArray()
        vel_msg.data=[0,0,0,0,0,0]

        



        

        joint_vel_pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)


        rospy.sleep(1.0)
        rate = rospy.Rate(frequence)
        


        while not rospy.is_shutdown() and self.safety():
            self.timer_init()

            #boutton
            if  not self.PRESS():
                values=np.array(vel_msg.data)
                for _ in range (100) :
                    values= values/1.03               #apres une etude tres serieuse sur geogebra
                    vel_msg.data=values.tolist()
                    joint_vel_pub.publish(vel_msg)
                    rospy.sleep(0.0025)
                    

                vel_msg.data=[0,0,0,0,0,0]
                joint_vel_pub.publish(vel_msg)
                self.v_k = np.zeros(6)
                self.x_k = np.zeros(6)
                self.a_k = np.zeros(6)
                joint_velocity_smoothed = None  
                filtered_force = np.zeros(6)
                smoothed_rel = np.zeros(6)
                continue
            # current_time=rospy.Time.now()
            # dt = (current_time - last_time).to_sec()
            # last_time = current_time


           

            if self.force_data is None:
                rate.sleep()
                continue

            if self.joint_state is None:
                rate.sleep()
                continue  # attend une valeur
        
                



            # Transform force into base_link
            wrench_global = self.transform_wrench_to_frame_alamain(self.force_data)
            if wrench_global is None:
                rate.sleep()
                continue

            
            #recup force (from force_sensor_publisher)
            fx = wrench_global.wrench.force.x
            fy = wrench_global.wrench.force.y
            fz = wrench_global.wrench.force.z
            tx = wrench_global.wrench.torque.x
            ty = wrench_global.wrench.torque.y
            tz = wrench_global.wrench.torque.z
            current_force = np.array([fx,fy,fz,tx,ty,tz])

            #recup pos joint
            joint_values = [self.joint_state.position[self.joint_state.name.index(jn)] for jn in joint_names]  #arange l'ordre des joint
            joint_values_np = np.array(joint_values)


            #filtre passe bas exponentiel
            smoothed_rel = (1 - F_alpha) * smoothed_rel + F_alpha * current_force



            filtered_force = smoothed_rel



            # test zone morte
            if np.linalg.norm(filtered_force[:3]) < force_dead_zone_cart :
                    filtered_force[0] = 0.0
                    filtered_force[1] = 0.0
                    filtered_force[2] = 0.0
            if np.linalg.norm(filtered_force[3:]) < force_dead_zone_rot :
                    filtered_force[3] = 0.0
                    filtered_force[4] = 0.0
                    filtered_force[5] = 0.0
            
            
            
            # 1. Remplir KDL.JntArray
            jnt_array = PyKDL.JntArray(len(joint_values))
            for i, pos in enumerate(joint_values):
                jnt_array[i] = pos


            # 1bis. CALCUL DE LA POSE RÉELLE DE L'EFFECTEUR
            fk_frame = PyKDL.Frame()
            self.kdl_fk_solver.JntToCart(jnt_array, fk_frame)
            # Translation
            current_pos = np.array([fk_frame.p[0], fk_frame.p[1], fk_frame.p[2]])

            # --- NOUVEAU : on récupère la quaternion KDL (x,y,z,w)
            qx, qy, qz, qw = fk_frame.M.GetQuaternion()
            # on convertit directement en vecteur de rotation
            current_rot = R.from_quat([qx, qy, qz, qw]).as_rotvec()

            x_k_real = np.hstack((current_pos, current_rot))


        

            # # # # 2. Calculer la Jacobienne
            jacobian = PyKDL.Jacobian(len(joint_values))
            self.kdl_jnt_to_jac_solver.JntToJac(jnt_array, jacobian)

            # # # # 3. Convertir en NumPy
            jacobian = np.array([[jacobian[i, j] for j in range(jacobian.columns())] for i in range(6)])


            if self.In_Meat() :
                M_dyn=M_mat_IM
                B_dyn=B_mat_IM
                print("on nez dan la viande")
            else :
                M_dyn=M_mat
                B_dyn=B_mat
                

            # Admittance dynamics
            self.a_k = (filtered_force - B_dyn* self.v_k - K_mat * self.x_k) /M_dyn      # K vaut 0 donc raf
            # a_k[axis] = max(min(a_k[axis], max_jerk), -max_jerk)
            self.v_k += self.a_k * dt
            self.x_k = x_k_real   #sert à rien 
            # print(x_k_real)
           
            # x_k += v_k * dt
            # v_k[axis] = max(min(v_k[axis], max_vel), -max_vel)


            ee_vel = self.v_k
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


            

            
            vel_msg.data= joint_velocity_smoothed.tolist()
            joint_vel_pub.publish(vel_msg)



                
               



            self.timer_add("fin")
            rate.sleep()

            self.timer_add("att")
            self.timer_info()

        values=np.array(vel_msg.data)
        for _ in range (100) :
            values= values/1.03               #apres une etude tres serieuse sur geogebra
            vel_msg.data=values.tolist()
            joint_vel_pub.publish(vel_msg)
            rospy.sleep(0.0025)
            print("zbi")

        vel_msg.data=[0,0,0,0,0,0]
        joint_vel_pub.publish(vel_msg)
        
        
def main():
    try:
        print("initialisation du node")
        os.system("pkill -f force_sensor_publisher_gp.py")
        os.system("pkill -f force_sensor_publisher.py")
        os.system("pkill -f force_sensor_eth_publisher.py")


        admittance=admittance_control()


        # input("============ Press `Enter` to start the admittance ...")

        #lancement du publisher du capteur
        subprocess.Popen(["rosrun", "force_sensor_node", "force_sensor_publisher.py"])
        subprocess.Popen(["rosrun", "force_sensor_node", "force_sensor_eth_publisher.py"])


        #switch controller au cas ou
        admittance.switch_controllers(['joint_group_vel_controller'], ['scaled_pos_joint_traj_controller'])


        #lancement admittance
        admittance.admittance_openloop()

        #print finito pour printer finito
        print("finito")
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()


          

    
