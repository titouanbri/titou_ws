#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
import tf2_ros
from tf.transformations import quaternion_multiply, quaternion_inverse
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from urdf_parser_py.urdf import URDF
import PyKDL
from kdl_parser_py.urdf import treeFromUrdfModel
from controller_manager_msgs.srv import SwitchController

# ------------------ utils quaternions ------------------
def xyzw_to_wxyz(q_xyzw):
    q = np.asarray(q_xyzw, dtype=float)
    return np.array([q[3], q[0], q[1], q[2]], dtype=float)

def wxyz_to_xyzw(q_wxyz):
    q = np.asarray(q_wxyz, dtype=float)
    return np.array([q[1], q[2], q[3], q[0]], dtype=float)

def quat_err_vec_and_angle(q_ref_wxyz, q_real_wxyz):
    """
    q_err = q_ref ⊗ inv(q_real)  (wxyz)
    Retourne (e_vec ≈ 2*sign(w)*v, angle)
    """
    q_err_xyzw = quaternion_multiply(
        wxyz_to_xyzw(q_ref_wxyz),
        quaternion_inverse(wxyz_to_xyzw(q_real_wxyz))
    )
    q_err_wxyz = xyzw_to_wxyz(q_err_xyzw)
    w, x, y, z = q_err_wxyz
    s = 1.0 if w >= 0.0 else -1.0
    e_vec = 2.0 * s * np.array([x, y, z], dtype=float)
    angle = 2.0 * np.arccos(np.clip(abs(w), 0.0, 1.0))
    return e_vec, angle

# ------------------ contrôleur ------------------
class admittance_control(object):
    def __init__(self):
        rospy.init_node("admittance", anonymous=True)

        # IMPORTANT: on contrôle et on mesure à la même frame
        self.tool_frame = "wrist_3_link"
        self.base_frame = "base_link"   

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber('right_arm/elbow', PointStamped, self.elbow_cb)
        rospy.Subscriber('right_arm/wrist', PointStamped, self.wrist_cb)
        rospy.Subscriber('joint_states', JointState, self.joint_state_cb)

        self.joint_state = None
        self.wrist_real = None
        self.elbow_real = None

        # Référence "latchée"
        self.ref_pos = None            # [x y z]
        self.ref_quat_wxyz = None      # [w x y z]
        self.anchor_wrist = None       # pour détecter le mouvement cible

        # Gains et limites
        self.freq = 350.0
        self.dt = 1.0 / self.freq
        self.Kp_pos = 0.5
        self.Kd_pos = 1.0
        self.Kp_ori = 1.0
        self.Kd_ori = 0.5
        self.lambda_dls = 0.08
        self.V_alpha = 0.1
        self.v_lin_max = 0.20     # m/s
        self.w_ang_max = 1.0      # rad/s
        self.qdot_max  = 0.8      # rad/s

        # Seuils de MAJ de la consigne (si cible bouge)
        self.pos_tol_update = 0.005            # 5 mm
        self.ori_tol_update = np.deg2rad(3.0)  # 3°
        # Deadbands pour commander zéro
        self.pos_deadband = 0.002              # 2 mm
        self.ori_deadband = np.deg2rad(1.0)    # 1°

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

        self.last_pose_real = None
        self.qdot_filt = None

    # ------------------ callbacks ------------------
    def wrist_cb(self, msg):
        self.wrist_real = np.array([msg.point.x, -msg.point.z, msg.point.y], dtype=float)

    def elbow_cb(self, msg):
        self.elbow_real = np.array([msg.point.x, -msg.point.z, msg.point.y], dtype=float)

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
            return np.array([t.x, t.y, t.z, r.x, r.y, r.z, r.w], dtype=float)  # XYZW
        except Exception as e:
            rospy.logwarn("TF fail %s: %s", frame, e)
            return None

    def quaternion_orientation_BA(self, A, B, axe_reference="z"):
        # (identique à ta version, renvoie wxyz)
        from math import sqrt
        def norm(v): return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
        def dot(a,b): return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
        def cross(a,b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
        def normalize(v):
            n = norm(v)
            if n < 1e-12: raise ValueError("Vecteur nul.")
            return (v[0]/n, v[1]/n, v[2]/n)
        A = tuple(map(float, A)); B = tuple(map(float, B))
        b = normalize((A[0]-B[0], A[1]-B[1], A[2]-B[2]))
        a = {'x':(1,0,0),'y':(0,1,0),'z':(0,0,1)}.get(axe_reference, (0,0,1))
        eps=1e-8; c = max(-1.0, min(1.0, a[0]*b[0]+a[1]*b[1]+a[2]*b[2]))
        if c > 1.0 - eps: return np.array([1.0,0.0,0.0,0.0], dtype=float)
        if c < -1.0 + eps:
            pick = (1,0,0) if abs(a[0])<0.9 else (0,1,0)
            axis = cross(a, pick); n = np.linalg.norm(axis)
            if n < eps: pick=(0,0,1); axis=cross(a,pick); n=np.linalg.norm(axis)
            axis = (axis[0]/n, axis[1]/n, axis[2]/n)
            return np.array([0.0, axis[0], axis[1], axis[2]], dtype=float)
        axis = cross(a, b); w = 1.0 + c
        q0,q1,q2,q3 = w, axis[0], axis[1], axis[2]
        qn = np.sqrt(q0*q0+q1*q1+q2*q2+q3*q3)
        return np.array([q0/qn, q1/qn, q2/qn, q3/qn], dtype=float)

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

        # Latch consigne = pose actuelle -> zéro mouvement au départ
        x0 = self.get_pose(self.tool_frame)
        p0 = x0[:3]
        q0_wxyz = xyzw_to_wxyz(x0[3:])
        self.ref_pos = self.wrist_real + self.offset
        self.ref_quat_wxyz = q0_wxyz.copy()     # on fige l’orientation initiale
        self.anchor_wrist = self.wrist_real.copy()
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
            if self.joint_state is None or self.wrist_real is None or self.elbow_real is None:
                rate.sleep(); continue

            # ordre des joints
            try:
                q = [self.joint_state.position[self.joint_state.name.index(jn)] for jn in self.joint_names]
            except ValueError:
                rate.sleep(); continue

            # Jacobien à wrist_3_link
            jnt_array = PyKDL.JntArray(len(q))
            for i, pos in enumerate(q): jnt_array[i] = pos
            Jkdl = PyKDL.Jacobian(len(q))
            self.kdl_jnt_to_jac_solver.JntToJac(jnt_array, Jkdl)
            J = np.array([[Jkdl[i, j] for j in range(Jkdl.columns())] for i in range(6)], dtype=float)

            # Pose réelle
            x_real = self.get_pose(self.tool_frame)
            if x_real is None:
                rate.sleep(); continue
            p_real = x_real[:3]
            q_real_wxyz = xyzw_to_wxyz(x_real[3:])

            # --------- mise à jour de la consigne si la cible bouge ----------
            p_tgt = self.wrist_real + self.offset
            q_tgt_wxyz = self.quaternion_orientation_BA(self.wrist_real, self.elbow_real)

            # Δ position cible vs consigne actuelle
            moved_pos = np.linalg.norm(p_tgt - self.ref_pos) > self.pos_tol_update
            # Δ orientation cible vs consigne actuelle
            _, ang_diff_consigne = quat_err_vec_and_angle(q_tgt_wxyz, self.ref_quat_wxyz)
            moved_ori = ang_diff_consigne > self.ori_tol_update

            if moved_pos or moved_ori:
                self.ref_pos = p_tgt
                self.ref_quat_wxyz = q_tgt_wxyz
                self.anchor_wrist = self.wrist_real.copy()
                rospy.loginfo("Nouvelle consigne (Δpos: %s, Δang: %.2f°)",
                              "oui" if moved_pos else "non",
                              np.degrees(ang_diff_consigne))

            # --------- erreurs par rapport à la consigne latchée ----------
            e_pos = self.ref_pos - p_real
            e_ori_vec, e_ori_ang = quat_err_vec_and_angle(self.ref_quat_wxyz, q_real_wxyz)

            # vitesses mesurées
            if self.last_pose_real is None:
                v_meas = np.zeros(3); w_meas = np.zeros(3)
            else:
                v_meas = (p_real - self.last_pose_real[:3]) / self.dt
                # ω via Δ quaternion
                _, e_ang_prev = quat_err_vec_and_angle(q_real_wxyz, self.last_pose_real[3:])
                # petite approximation : on réutilise e_ori_vec précédent si besoin
                # ici, on recalcule proprement :
                q_prev = self.last_pose_real[3:]
                q_delta_xyzw = quaternion_multiply(
                    wxyz_to_xyzw(q_real_wxyz),
                    quaternion_inverse(wxyz_to_xyzw(q_prev))
                )
                qd_w, qd_x, qd_y, qd_z = xyzw_to_wxyz(q_delta_xyzw)
                s = 1.0 if qd_w >= 0.0 else -1.0
                w_meas = (2.0 / self.dt) * s * np.array([qd_x, qd_y, qd_z], dtype=float)

            # --------- deadband : si la cible n'a pas bougé et l'erreur est petite -> 0 ---------
            if (np.linalg.norm(e_pos) < self.pos_deadband) and (e_ori_ang < self.ori_deadband):
                ee_twist = np.zeros(6)
            else:
                v_lin = self.Kp_pos * e_pos - self.Kd_pos * v_meas
                w_ang = self.Kp_ori * e_ori_vec - self.Kd_ori * w_meas
                v_lin = np.clip(v_lin, -self.v_lin_max, self.v_lin_max)
                w_ang = np.clip(w_ang, -self.w_ang_max, self.w_ang_max)
                ee_twist = np.hstack([v_lin, w_ang])

            # pseudo-inverse amortie
            U, S, Vt = np.linalg.svd(J, full_matrices=False)
            S_damped = S / (S**2 + self.lambda_dls**2)
            J_pinv = Vt.T.dot(np.diag(S_damped)).dot(U.T)
            qdot = J_pinv.dot(ee_twist)

            # filtre + saturation
            if self.qdot_filt is None:
                self.qdot_filt = qdot
            else:
                self.qdot_filt = self.V_alpha * qdot + (1 - self.V_alpha) * self.qdot_filt
            self.qdot_filt = np.clip(self.qdot_filt, -self.qdot_max, self.qdot_max)

            vel_msg.data = self.qdot_filt.tolist()
            self.joint_vel_pub.publish(vel_msg)

            # log & mémoire
            self.last_pose_real = np.hstack([p_real, q_real_wxyz])
            rospy.loginfo_throttle(0.5, "||e_pos||=%.3f mm | e_ang=%.2f deg",
                                   1000.0*np.linalg.norm(e_pos),
                                   np.degrees(e_ori_ang))
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
        ctrl = admittance_control()
        ctrl.switch_controllers(['joint_group_vel_controller'], ['scaled_pos_joint_traj_controller'])
        rospy.loginfo("Init offset + latch...")
        ctrl.init_offset_and_latch()
        rospy.loginfo("Go.")
        ctrl.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        return

if __name__ == "__main__":
    main()
