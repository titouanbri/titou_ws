#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from urdf_parser_py.urdf import URDF
import PyKDL
from kdl_parser_py.urdf import treeFromUrdfModel
from controller_manager_msgs.srv import SwitchController

# ------------------ contrôleur (PID position -> vitesse) ------------------
class PR_control(object):
    def __init__(self):
        rospy.init_node("PR", anonymous=True)

        # IMPORTANT: on contrôle et on mesure à la même frame
        self.tool_frame = "wrist_3_link"
        self.base_frame = "base_link"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber('right_arm/wrist', PointStamped, self.wrist_cb)
        rospy.Subscriber('joint_states', JointState, self.joint_state_cb)

        self.joint_state = None
        self.wrist_real = None

        # Référence "latchée"
        self.ref_pos = None            # [x y z]
        self.offset = np.zeros(3)

        # Gains et limites
        self.freq = 350.0
        self.dt = 1.0 / self.freq
        self.Kp_pos = 0.25
        self.Kd_pos = 0.8
        self.Ki_pos = 0.02              # [~1/s] (commencer petit)
        self.I_term_max = 0.10         # [m/s] limite par axe du terme intégral effectif
        self.I_leak = 0.0              # [1/s] fuite de l'intégrateur (0 pour désactiver)
        self.I_pos = np.zeros(3)       # état intégrateur (∫ e dt)

        self.lambda_dls = 0.18
        self.V_alpha = 0.1
        self.v_lin_max = 0.08    # m/s
        self.qdot_max  = 0.8      # rad/s

        # Seuil de MAJ de la consigne (si cible bouge)
        self.pos_tol_update = 0.005    # 5 mm
        # Deadband position pour commander zéro
        self.pos_deadband = 0.005      # 2 mm

        # KDL : chaîne base -> wrist_3_link (même que la pose mesurée)
        robot_description = rospy.get_param("/robot_description")
        robot = URDF.from_xml_string(robot_description)
        ok, tree = treeFromUrdfModel(robot)
        chain = tree.getChain(self.base_frame, self.tool_frame)
        self.kdl_jnt_to_jac_solver = PyKDL.ChainJntToJacSolver(chain)
        self.kdl_fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)

        self.joint_names = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
                            'wrist_1_joint','wrist_2_joint','wrist_3_joint']

        self.joint_vel_pub = rospy.Publisher('joint_group_vel_controller/command',
                                             Float64MultiArray, queue_size=1)

        self.last_p_real = None
        self.qdot_filt = None

    # ------------------ callbacks ------------------
    def wrist_cb(self, msg):
        # Conversion axes (identique à ton code original)
        self.wrist_real = np.array([msg.point.x, -msg.point.z, msg.point.y], dtype=float)

    def joint_state_cb(self, msg):
        self.joint_state = msg

    # ------------------ utilitaires ------------------
    def get_pose(self, frame):
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.base_frame, frame, rospy.Time(0), rospy.Duration(0.1)
            )
            t = tf_stamped.transform.translation
            r = tf_stamped.transform.rotation
            # Retourne [x y z qx qy qz qw] mais on n'utilisera ici que la position
            return np.array([t.x, t.y, t.z, r.x, r.y, r.z, r.w], dtype=float)
        except Exception as e:
            rospy.logwarn("TF fail %s: %s", frame, e)
            return None

    # ------------------ initialisation offset + latch ------------------
    def init_offset_and_latch(self):
        # estime l'offset pour que p_ref = p_real au départ, puis "latch" la consigne
        buf = []
        i = 0
        while i < 5 and not rospy.is_shutdown():
            if self.wrist_real is None:
                rospy.sleep(0.01); continue
            x = self.get_pose(self.tool_frame)
            if x is None:
                rospy.sleep(0.01); continue
            buf.append(-self.wrist_real + x[:3])
            i += 1
            rospy.sleep(0.01)

        self.offset = np.mean(np.vstack(buf), axis=0) if buf else np.zeros(3)
        rospy.loginfo("Offset estimé: %s", np.round(self.offset, 4))

        # Latch consigne = position actuelle -> zéro mouvement au départ
        self.ref_pos = self.wrist_real + self.offset
        self.I_pos = np.zeros(3)  # reset intégrateur au latch
        rospy.loginfo("Consigne latched (pos=%s)", np.round(self.ref_pos, 4))

    # ------------------ switch contrôleurs ------------------
    def switch_controllers(self, start_list, stop_list):
        rospy.wait_for_service('controller_manager/switch_controller')
        try:
            switch_controller = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
            resp = switch_controller(start_controllers=start_list, stop_controllers=stop_list, strictness=1)
            return resp.ok
        except Exception as e:
            rospy.logerr("Switch controller fail: %s", e)
            return False

    # ------------------ boucle principale ------------------
    def run(self):
        vel_msg = Float64MultiArray()
        vel_msg.data = [0,0,0,0,0,0]

        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            if self.joint_state is None or self.wrist_real is None:
                rate.sleep(); continue

            # ordre des joints
            try:
                q = [self.joint_state.position[self.joint_state.name.index(jn)] for jn in self.joint_names]
            except ValueError:
                rate.sleep(); continue

            # Jacobien 6xN à wrist_3_link, puis on ne garde que la partie linéaire 3xN
            jnt_array = PyKDL.JntArray(len(q))
            for i, pos in enumerate(q): jnt_array[i] = pos
            Jkdl = PyKDL.Jacobian(len(q))
            self.kdl_jnt_to_jac_solver.JntToJac(jnt_array, Jkdl)
            J6 = np.array([[Jkdl[i, j] for j in range(Jkdl.columns())] for i in range(6)], dtype=float)
            J = J6[:3, :]  # partie linéaire uniquement

            # Position réelle de l'outil (via TF de la même frame)
            x_real = self.get_pose(self.tool_frame)
            if x_real is None:
                rate.sleep(); continue
            p_real = x_real[:3]

            # --------- mise à jour de la consigne si la cible (wrist) bouge ----------
            p_tgt = self.wrist_real + self.offset
            moved_pos = np.linalg.norm(p_tgt - self.ref_pos) > self.pos_tol_update
            if moved_pos:
                self.ref_pos = p_tgt
                self.I_pos = np.zeros(3)  # éviter le "kick" intégral à changement de cible
                rospy.loginfo("Nouvelle consigne (Δpos: %.1f mm)", 1000.0*np.linalg.norm(p_tgt - p_real))

            # --------- erreur position ----------
            e_pos = self.ref_pos - p_real

            # vitesses mesurées (linéaire uniquement)
            if self.last_p_real is None:
                v_meas = np.zeros(3)
            else:
                v_meas = (p_real - self.last_p_real) / self.dt

            # --------- PID position -> vitesse ----------
            if np.linalg.norm(e_pos) < self.pos_deadband:
                # deadband: commande zéro + reset intégrateur
                v_cmd = np.zeros(3)
                self.I_pos = np.zeros(3)
            else:
                # intégrateur : accumulateur + fuite optionnelle
                self.I_pos += e_pos * self.dt
                if self.I_leak > 0.0:
                    self.I_pos *= (1.0 - self.I_leak * self.dt)

                # terme intégral limité et anti-windup basique
                if self.Ki_pos > 0.0:
                    # limite directement le terme I effectif (en [m/s]) puis reflète sur l'état
                    I_term = self.Ki_pos * self.I_pos
                    I_term = np.clip(I_term, -self.I_term_max, self.I_term_max)
                    self.I_pos = np.clip(self.I_pos,
                                         -self.I_term_max / self.Ki_pos,
                                          self.I_term_max / self.Ki_pos)
                else:
                    I_term = np.zeros(3)

                v_cmd = self.Kp_pos * e_pos + I_term - self.Kd_pos * v_meas
                v_cmd = np.clip(v_cmd, -self.v_lin_max, self.v_lin_max)

            # pseudo-inverse amortie (3xN)
            U, S, Vt = np.linalg.svd(J, full_matrices=False)  # J: 3x6 -> U:3x3, S:3, Vt:3x6
            S_damped = S / (S**2 + self.lambda_dls**2)
            J_pinv = Vt.T.dot(np.diag(S_damped)).dot(U.T)     # 6x3

            qdot = J_pinv.dot(v_cmd)  # orientation complètement ignorée

            # filtre + saturation
            if self.qdot_filt is None:
                self.qdot_filt = qdot
            else:
                self.qdot_filt = self.V_alpha * qdot + (1 - self.V_alpha) * self.qdot_filt
            self.qdot_filt = np.clip(self.qdot_filt, -self.qdot_max, self.qdot_max)

            vel_msg.data = self.qdot_filt.tolist()
            self.joint_vel_pub.publish(vel_msg)

            # log & mémoire
            self.last_p_real = p_real.copy()
            rospy.loginfo_throttle(0.5,
                "||e_pos||=%.3f mm | ||v_cmd||=%.3f mm/s",
                1000.0*np.linalg.norm(e_pos),
                1000.0*np.linalg.norm(v_cmd)
            )
            rate.sleep()

        # arrêt propre
        vals = np.array(vel_msg.data, dtype=float)
        for _ in range(100):
            vals /= 1.03
            vel_msg.data = vals.tolist()
            self.joint_vel_pub.publish(vel_msg)
            rospy.sleep(0.0025)
        vel_msg.data = [0]*6
        self.joint_vel_pub.publish(vel_msg)

# ------------------ main ------------------
def main():
    try:
        ctrl = PR_control()
        ctrl.switch_controllers(['joint_group_vel_controller'], ['scaled_pos_joint_traj_controller'])
        rospy.loginfo("Init offset + latch...")
        ctrl.init_offset_and_latch()
        rospy.loginfo("Go.")
        ctrl.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        return

if __name__ == "__main__":
    main()
