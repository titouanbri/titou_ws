#!/usr/bin/env python3
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
import matplotlib.pyplot as plt
import matplotlib.animation as animation






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
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)



        self.last_R_mat_test=None
        self.last_Rot=None

        self.last_R_mat_test2=None
        self.last_Rot2=None

        # for time logging
        self.timer_name =[]
        self.timer_value =[]

        robot_description = rospy.get_param("/robot_description")
        robot = URDF.from_xml_string(robot_description)
        ok, tree = treeFromUrdfModel(robot)
        chain = tree.getChain("base_link", "fake_gripper")  # adapte ces noms à ton robot

        self.kdl_chain = chain
        self.kdl_jnt_to_jac_solver = PyKDL.ChainJntToJacSolver(chain)

        self.correction = R.from_euler('z', -np.pi / 2)  #correction constante pour le capteur
        self.cg_history = deque(maxlen=100)  # Moyenne glissante sur 10 valeurs pour le centre de gravité
        self.force_offset=None 

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

    def calibrate_force_sensor_offset(self, wrench_msg, tool_mass=0, tool_com=np.array([0.00428, 0.02016, 0.05838]), n_samples=20):
        """
        Calibre l'offset du capteur de force en tenant compte du poids de l'outil fixé.
        :param tool_mass: masse en kg de l'outil fixé sur le capteur
        :param tool_com: position du centre de masse de l'outil dans le repère du capteur (en mètres)
        :param n_samples: nombre de mesures pour estimer l'offset
        """
        rospy.loginfo("Démarrage de la calibration de l'offset du capteur de force...")
        g = [0,0,-9.81]
        collected_forces = []
        collected_torques = []
        source_frame="base_link"
        target_frame="fake_gripper"
        rospy.sleep(1.0)
        rate = rospy.Rate(100)
        seuil_norme_q=0.1

        while len(collected_forces) < n_samples and not rospy.is_shutdown():
            if wrench_msg is None :
                print("oui")
                rate.sleep()
                continue




            #force brut du capteur
            f= np.array([
                wrench_msg.wrench.force.y,
                -wrench_msg.wrench.force.x,
                wrench_msg.wrench.force.z
            ])
            t = np.array([
                wrench_msg.wrench.torque.y,
                -wrench_msg.wrench.torque.x,
                wrench_msg.wrench.torque.z
            ])


            

            

            collected_forces.append(f)
            collected_torques.append(t)
            rate.sleep()

        if len(collected_forces) < n_samples:
            rospy.logwarn("Pas assez de données collectées pour calibrer l'offset.")
            return

        avg_force = np.mean(collected_forces, axis=0)
        avg_torque = np.mean(collected_torques, axis=0)

        #rotation base vers capteur pour la gravité
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rospy.Time(0),  
            rospy.Duration(0.01)
        )
        

        # Récupère la rotation
        q = transform.transform.rotation
        # rospy.loginfo("Quaternion TF : [%.4f %.4f %.4f %.4f]", q.x, q.y, q.z, q.w)

        # à tester si il est pas cassé
        Rot_test = R.from_quat([q.x, q.y, q.z, q.w])

        # check pour la matrice 
        R_mat_test = Rot_test.as_dcm()


        if self.last_R_mat_test2 is not None:
            diff = np.linalg.norm(R_mat_test - self.last_R_mat_test2)
            
            if diff <= seuil_norme_q:  # ← seuil pour les normes à tej
                Rot=Rot_test
                # Rot = Rot*self.correction.inv()
                self.last_Rot2=Rot
                self.last_R_mat_test2 = R_mat_test
                    
            else : 
                Rot=self.last_Rot2
                R_mat_test=self.last_R_mat_test2

                # rospy.logwarn(" Changement brutal de la matrice de rotation détecté (diff = %.4f)", diff)
        else :
            Rot=Rot_test  #*self.correction.inv()
            self.last_Rot2=Rot
            self.last_R_mat_test2=R_mat_test


        Rot_qui_fonctionne_inshallah=Rot.as_dcm()
        g_transf = np.dot(Rot_qui_fonctionne_inshallah,g)  # g dans repère capteur



        # Calcul de la force gravitationnelle théorique dans le repère du capteur
        gravity_force = tool_mass*g_transf

        # Moment généré par le centre de masse de l’outil
        gravity_torque = np.cross(tool_com, gravity_force)

        # Offset réel mesuré - contribution gravitationnelle attendue
        force_offset = avg_force - gravity_force
        torque_offset = avg_torque - gravity_torque

        force_offset = np.dot(np.linalg.inv(Rot_qui_fonctionne_inshallah),force_offset)
        torque_offset = np.dot(np.linalg.inv(Rot_qui_fonctionne_inshallah),torque_offset)
        self.force_offset = np.concatenate((force_offset, torque_offset))


        rospy.loginfo("Offset capteur estimé (après compensation gravité): %s", self.force_offset)

    def transform_wrench_to_frame_alamain(self,wrench_msg,source_frame="fake_gripper",target_frame="base_link"):

        
        seuil_norme_q=0.1
        

       

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(0),  
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
            # d=np.array([0,0.05,0.07])
            # torque_manche=torque + np.cross(d,force)

            # Transformation (rotation uniquement)
            Rot_qui_fonctionne_inshallah=Rot.as_dcm()
            force_trans = np.dot(Rot_qui_fonctionne_inshallah,force) 
            torque_trans = np.dot(Rot_qui_fonctionne_inshallah,torque_manche)

            self.timer_add("transf de la f")

            
            # Crée un nouveau message WrenchStamped
            new_wrench = WrenchStamped()
            new_wrench.header.stamp = wrench_msg.header.stamp
            new_wrench.header.frame_id = source_frame
            new_wrench.wrench.force.x = force_trans[0]
            new_wrench.wrench.force.y = force_trans[1]
            new_wrench.wrench.force.z = force_trans[2]
            new_wrench.wrench.torque.x =torque_trans[0]
            new_wrench.wrench.torque.y =torque_trans[1]
            new_wrench.wrench.torque.z =torque_trans[2]


            return new_wrench
        

        except Exception as e:
            rospy.logwarn("Erreur transformation explicite : %s", str(e))
            return None

    def admittance_openloop(self) :

        rospy.loginfo("Starting full-body admittance control (6D: position + orientation via torque)...")

        # Admittance parameters
        c=12
        
        M = 3; B = c*M; K = 0        # translation
        M_rot = 0.07; B_rot = c*M_rot ; K_rot = 0  # rotation
        frequence=450 #Hz
        dt =1/frequence
        max_jerk_cart = 0.3      #limiter l'acceleration
        max_jerk_rot = 0.2
        force_dead_zone_cart = 0.08  # eviter de publier pour rien
        force_dead_zone_rot = 0.008
        F_alpha = 0.02  # filtre passe-bas exponentiel (proche de 0 = réponse lente mais beaucoup filtré)
        last_time=rospy.Time.now()
        joint_velocity_smoothed = None  # pour initialiser
        V_alpha = 0.06  # coefficient de lissage (plus petit = plus lisse)

    
        filtered_force = np.zeros(6)
        v_k = np.zeros(6)
        x_k = np.zeros(6)
        smoothed_rel = np.zeros(6)

        B_mat=np.array([B,B,B,B_rot,B_rot,B_rot])
        K_mat=np.array([K,K,K,K_rot,K_rot,K_rot])
        M_mat=np.array([M,M,M,M_rot,M_rot,M_rot])
        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        

        #pour tracer
        # Pour tracer les 3 composantes de la force (Fx, Fy, Fz)
        # window_size = 300  # nombre de points visibles

        # fx_data = deque(maxlen=window_size)
        # fy_data = deque(maxlen=window_size)
        # fz_data = deque(maxlen=window_size)
        # time_data = deque(maxlen=window_size)
        # t0 = rospy.Time.now().to_sec()

        # Préparer le graphe

        # plt.ion()
        # fig, ax = plt.subplots()
        # line_fx, = ax.plot([], [], label='Fx')
        # line_fy, = ax.plot([], [], label='Fy')
        # line_fz, = ax.plot([], [], label='Fz')
        # ax.set_xlim(0, window_size / 100.0)  # affichage en secondes (~100Hz)
        # ax.set_ylim(-20, 20)
        # ax.legend()
        # ax.set_xlabel('Temps (s)')
        # ax.set_ylabel('Force (N)')
        

        joint_vel_pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)


        rospy.sleep(1.0)
        rate = rospy.Rate(frequence)
        


        while not rospy.is_shutdown():

            self.timer_init()

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

            


            fx = wrench_global.wrench.force.x
            fy = wrench_global.wrench.force.y
            fz = wrench_global.wrench.force.z
            tx = wrench_global.wrench.torque.x
            ty = wrench_global.wrench.torque.y
            tz = wrench_global.wrench.torque.z



            #TEST CG
            # Mesure brute
            # measured_force = np.array([fx, fy, fz])
            # measured_torque = np.array([tx, ty, tz])

            # # Offset capteur
            # if hasattr(self, 'force_offset'):
            #     offset_force = self.force_offset[:3]
            #     offset_torque = self.force_offset[3:]
            # else:
            #     offset_force = np.zeros(3)
            #     offset_torque = np.zeros(3)

            # # Compensation gravité
            # tool_mass = 0.118  # en kg
            # tool_com = np.array([0.00428, 0.02016, 0.05838])  # position du centre de masse dans le repère outil

            # gravity_force_base = np.array([0, 0, -9.81 * tool_mass])
            # gravity_torque_base = np.cross(tool_com, gravity_force_base)

            # # Force capteur - offset + compensation de la gravité
            # compensated_force = measured_force - offset_force - gravity_force_base
            # compensated_torque = measured_torque - offset_torque - gravity_torque_base

            # current_force = np.concatenate((compensated_force, compensated_torque))




            current_force = np.array([fx,fy,fz,tx,ty,tz])- self.force_offset


            #filtre passe bas exponentiel
            smoothed_rel = (1 - F_alpha) * smoothed_rel + F_alpha * current_force

            filtered_force = smoothed_rel

            # test nouvelle zone morte
            if np.linalg.norm(filtered_force[:3]) < force_dead_zone_cart :
                    filtered_force[0] = 0.0
                    filtered_force[1] = 0.0
                    filtered_force[2] = 0.0
            if np.linalg.norm(filtered_force[3:]) < force_dead_zone_rot :
                    filtered_force[3] = 0.0
                    filtered_force[4] = 0.0
                    filtered_force[5] = 0.0
            
            
            # Admittance dynamics
         
            a_k = (filtered_force - B_mat* v_k - K_mat * x_k) /M_mat
            v_k += a_k * dt
            x_k += v_k * dt



            

            try:
                #acquisition pos joint
                joint_values = [self.joint_state.position[self.joint_state.name.index(jn)] for jn in joint_names]  #arange l'ordre des joint

                # 1. Remplir KDL.JntArray
                jnt_array = PyKDL.JntArray(len(joint_values))
                for i, pos in enumerate(joint_values):
                    jnt_array[i] = pos

                # # # # 2. Calculer la Jacobienne
                jacobian = PyKDL.Jacobian(len(joint_values))
                self.kdl_jnt_to_jac_solver.JntToJac(jnt_array, jacobian)

                # # # # 3. Convertir en NumPy
                jacobian = np.array([[jacobian[i, j] for j in range(jacobian.columns())] for i in range(6)])

                ee_vel = v_k

                joint_velocities = np.linalg.pinv(jacobian).dot(ee_vel)



                if joint_velocity_smoothed is None:
                    joint_velocity_smoothed = joint_velocities
                else:
                    joint_velocity_smoothed = V_alpha * joint_velocities + (1 - V_alpha) * joint_velocity_smoothed


                

                
                vel_msg= Float64MultiArray()
                vel_msg.data= joint_velocity_smoothed.tolist()
                joint_vel_pub.publish(vel_msg)


                self.timer_add("fin")

                
               



            except Exception as e:
                rospy.logwarn("Erreur dans le calcul ou la publication : %s", str(e))


            # pour tracer
            # Ajout des nouvelles valeurs
            # now = rospy.Time.now().to_sec() - t0
            # time_data.append(now)
            # fx_data.append(current_force[0])
            # fy_data.append(current_force[1])
            # fz_data.append(current_force[2])

            # Mise à jour du graphe
            # line_fx.set_data(time_data, fx_data)
            # line_fy.set_data(time_data, fy_data)
            # line_fz.set_data(time_data, fz_data)
            # ax.set_xlim(max(0, now - window_size / 100.0), now + 0.1)
            # ax.figure.canvas.draw()
            # ax.figure.canvas.flush_events()

            rate.sleep()

            self.timer_info()



def main():
    try:

        os.system("pkill -f force_sensor_publisher.py")
        os.system("pkill -f force_sensor_publisher_gp.py")

        admittance=admittance_control()
        admittance.switch_controllers(['joint_group_vel_controller'], ['scaled_pos_joint_traj_controller'])


        
        #lancement du publisher du capteur
        subprocess.Popen(["rosrun", "force_sensor_node", "force_sensor_publisher_gp.py"])
        rospy.sleep(4)
        admittance.calibrate_force_sensor_offset(wrench_msg=admittance.force_data)        
        rospy.sleep(0.5)

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


          

    

