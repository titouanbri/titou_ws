#!/usr/bin/env python3
# coding: utf-8
"""
Logger ROS : /wrench (+ pose EE) + vidéo RealSense + données Arduino + EtherCAT dans un CSV
- Enregistre forces/torques dans un CSV avec la pose de l'end-effector (TF puis fallback PoseStamped) et données capteurs.
- Sauvegarde une vidéo .mp4 (optionnellement des images) depuis la caméra RealSense.

Paramètres ROS (rosparam ~ns) :
  duration         [int]    Durée de capture en secondes (def: 10)
  base_frame       [str]    Frame de base pour TF (def: base_link)
  ee_frame         [str]    Frame de l'EE pour TF (def: tool0)
  ee_pose_topic    [str]    Topic PoseStamped fallback (def: /ee_pose)
  color_width      [int]    Résolution couleur (def: 640)
  color_height     [int]    Résolution couleur (def: 480)
  color_fps        [int]    FPS couleur (def: 30)
  save_images      [bool]   Enregistrer aussi 1 image/s (def: False)
  video_codec      [str]    Codec OpenCV (def: mp4v)
  serial_port      [str]    Port série Arduino (def: /dev/ttyACM0)
  serial_baud      [int]    Baudrate série (def: 115200)
  serial_timeout   [float]  Timeout lecture série en s (def: 0.1)
"""

import os
import csv
from datetime import datetime
from threading import Thread

import numpy as np
import cv2
import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped
import tf  # tf (ROS1). Pour tf2, adapter.

# ----------- RealSense -----------
try:
    import pyrealsense2 as rs
except ImportError:
    rs = None
    rospy.logwarn("pyrealsense2 introuvable : la vidéo ne sera pas enregistrée.")

# ----------- Série / Arduino -----------
try:
    import serial
except ImportError:
    serial = None
    rospy.logwarn("pyserial introuvable : les données Arduino ne seront pas lues.")


class WrenchAndCameraLogger:
    def __init__(self):
        rospy.init_node('data', anonymous=True)

        # ------------------ Paramètres ------------------
        self.duration = rospy.get_param('~duration', 30)
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.ee_frame = rospy.get_param('~ee_frame', 'tool0')
        self.pose_topic = rospy.get_param('~ee_pose_topic', '/ee_pose')
        self.color_width = rospy.get_param('~color_width', 640)
        self.color_height = rospy.get_param('~color_height', 480)
        self.color_fps = rospy.get_param('~color_fps', 30)
        self.save_images = rospy.get_param('~save_images', False)
        self.video_codec = rospy.get_param('~video_codec', 'mp4v')
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.serial_baud = rospy.get_param('~serial_baud', 115200)
        self.serial_timeout = rospy.get_param('~serial_timeout', 0.1)

        # ------------------ Dossiers & fichiers ------------------
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        base_dir = os.path.expanduser('~/catkin_ws/src/wrench_camera_data')
        os.makedirs(base_dir, exist_ok=True)

        # CSV
        self.csv_path = os.path.join(base_dir, f'wrench_data_{timestamp}.csv')
        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # En-tête incluant EtherCAT (/force_sensor_eth)
        self.csv_writer.writerow([
            'timestamp_ms', 'timestamp',
            # Wrench ROS
            'force_x', 'force_y', 'force_z',
            'torque_x', 'torque_y', 'torque_z',
            # EtherCAT wrench
            'eth_force_x', 'eth_force_y', 'eth_force_z',
            'eth_torque_x', 'eth_torque_y', 'eth_torque_z',
            # End-effector pose
            'ee_x', 'ee_y', 'ee_z',
            'ee_qx', 'ee_qy', 'ee_qz', 'ee_qw',
            # Arduino
            'R1', 'R2', 'R3', 'R4', 'R5', 'R6',
            # Onrobot
            'OR_x','OR_y','OR_z',
            'OR_rx','OR_ry','OR_rz'

        ])

        # Vidéo & images
        self.video_path = os.path.join(base_dir, f'video_{timestamp}.mp4')
        self.image_dir = os.path.join(base_dir, f'images_{timestamp}')
        if self.save_images:
            os.makedirs(self.image_dir, exist_ok=True)

        # ------------------ Variables runtime ------------------
        self.start_time = rospy.get_time()
        self.last_progress = -1

        # Pose EE
        self.tf_listener = tf.TransformListener()
        self.latest_pose = None

        # Arduino data (valeurs courantes)
        self.arduino_data = {f'R{i+1}': float('nan') for i in range(6)}

        # EtherCAT wrench (valeurs courantes)
        self.eth_force = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
        self.eth_torque = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
        self.OR_force = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
        self.OR_torque = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
        # Threads
        self.serial_thread = None
        self.serial_running = False
        self.ser = None
        if serial is not None:
            self._init_serial()

        # RealSense
        self.pipeline = None
        self.config = None
        self.video_writer = None
        self.capturing = False
        self.capture_thread = None
        if rs is not None:
            self._init_realsense()

        # ------------------ ROS Comm ------------------
        rospy.Subscriber('/wrench', WrenchStamped, self.wrench_callback)
        # rospy.Subscriber('/force_sensor_eth', WrenchStamped, self.eth_callback)
        rospy.Subscriber('/bus0/ft_sensor0/ft_sensor_readings/wrench', WrenchStamped, self.eth_callback)
        rospy.Subscriber('/ft_sensor', WrenchStamped, self.OR_callback)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)

        rospy.Timer(rospy.Duration(self.duration), self.stop_recording, oneshot=True)
        rospy.Timer(rospy.Duration(1), self.print_progress)

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("data node démarré.")
        rospy.spin()

    # ------------------ Série / Arduino ------------------
    def _init_serial(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.serial_baud, timeout=self.serial_timeout)
            self.serial_running = True
            self.serial_thread = Thread(target=self._serial_loop)
            self.serial_thread.daemon = True
            self.serial_thread.start()
            rospy.loginfo(f"Arduino connecté sur {self.serial_port} @ {self.serial_baud} bauds")
        except Exception as e:
            rospy.logerr(f"Impossible d'ouvrir le port série {self.serial_port} : {e}")
            self.ser = None

    def _serial_loop(self):
        while not rospy.is_shutdown() and self.serial_running and self.ser is not None:
            try:
                line = self.ser.readline()
                if not line:
                    continue
                try:
                    line = line.decode('utf-8', errors='ignore').strip()
                except AttributeError:
                    line = line.strip()
                if not line:
                    continue
                data = self._parse_arduino_line(line)
                if data:
                    self.arduino_data.update(data)
            except Exception as e:
                rospy.logwarn(f"Erreur lecture série : {e}")
                
    @staticmethod
    def _parse_arduino_line(line):
        line = line.strip()
        if ',' in line and ':' not in line:
            try:
                vals = [float(x) for x in line.split(',') if x]
            except ValueError:
                return {}
            return {f'R{i+1}': vals[i] if i < len(vals) else float('nan') for i in range(6)}

        out = {}
        for token in line.split():
            if ':' not in token:
                continue
            k, v = token.split(':', 1)
            try:
                out[k] = float(v)
            except ValueError:
                continue
        return out

    # ------------------ RealSense ------------------
    def _init_realsense(self):
        try:
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            self.config.enable_stream(rs.stream.color,
                                      self.color_width,
                                      self.color_height,
                                      rs.format.bgr8,
                                      self.color_fps)
            self.pipeline.start(self.config)
            rospy.loginfo("RealSense camera initialized successfully")

            fourcc = cv2.VideoWriter_fourcc(*self.video_codec)
            self.video_writer = cv2.VideoWriter(
                self.video_path,
                fourcc,
                float(self.color_fps),
                (self.color_width, self.color_height)
            )
            if not self.video_writer.isOpened():
                rospy.logerr("Impossible d'ouvrir le VideoWriter. La vidéo ne sera pas sauvegardée.")
                self.video_writer = None

            self.capturing = True
            self.capture_thread = Thread(target=self._capture_loop)
            self.capture_thread.daemon = True
            self.capture_thread.start()
        except Exception as e:
            rospy.logerr(f"Could not initialize RealSense camera: {e}")
            self.pipeline = None
            self.video_writer = None

    def _capture_loop(self):
        while not rospy.is_shutdown() and self.capturing:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            img = np.asanyarray(color_frame.get_data())
            if self.video_writer:
                self.video_writer.write(img)
            if self.save_images:
                now_ms = int(rospy.get_time() * 1000)
                img_path = os.path.join(self.image_dir, f'image_{now_ms}.jpg')
                cv2.imwrite(img_path, img, [cv2.IMWRITE_JPEG_QUALITY, 90])

    # ------------------ Callbacks ------------------
    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg

    def wrench_callback(self, msg: WrenchStamped):
        # Callback principal pour /wrench
        self._log_data(msg, source='wrench')

    def eth_callback(self, msg: WrenchStamped):
        # Met à jour les valeurs EtherCAT
        self.eth_force = np.array([msg.wrench.force.x,
                                   msg.wrench.force.y,
                                   msg.wrench.force.z], dtype=np.float32)
        self.eth_torque = np.array([msg.wrench.torque.x,
                                    msg.wrench.torque.y,
                                    msg.wrench.torque.z], dtype=np.float32)
        
    def OR_callback(self, msg: WrenchStamped):
        # Met à jour les valeurs EtherCAT
        self.OR_force = np.array([msg.wrench.force.x,
                                   msg.wrench.force.y,
                                   msg.wrench.force.z], dtype=np.float32)
        self.OR_torque = np.array([msg.wrench.torque.x,
                                    msg.wrench.torque.y,
                                    msg.wrench.torque.z], dtype=np.float32)

    def _log_data(self, msg: WrenchStamped, source='wrench'):
        if self.csv_file.closed:
            return

        ts_ms = int(rospy.get_time() * 1000)
        ts_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')

        f = msg.wrench.force
        t = msg.wrench.torque

        # Pose via TF -> fallback PoseStamped
        px = py = pz = qx = qy = qz = qw = float('nan')
        try:
            trans, rot = self.tf_listener.lookupTransform(
                self.base_frame, self.ee_frame, rospy.Time(0))
            px, py, pz = trans
            qx, qy, qz, qw = rot
        except Exception:
            if self.latest_pose:
                p = self.latest_pose.pose.position
                o = self.latest_pose.pose.orientation
                px, py, pz = p.x, p.y, p.z
                qx, qy, qz, qw = o.x, o.y, o.z, o.w

        # Arduino snapshot
        R = [self.arduino_data.get(f'R{i+1}', float('nan')) for i in range(6)]

        # Écriture ligne CSV avec EtherCAT
        row = [
            ts_ms, ts_str,
            np.float32(f.x), np.float32(f.y), np.float32(f.z),
            np.float32(t.x), np.float32(t.y), np.float32(t.z),
            # EtherCAT
            *self.eth_force.tolist(),
            *self.eth_torque.tolist(),
            # Pose
            np.float32(px), np.float32(py), np.float32(pz),
            np.float32(qx), np.float32(qy), np.float32(qz), np.float32(qw),
            # Arduino
            *[np.float32(val) for val in R],
            #Onrobot
            *self.OR_force.tolist(),
            *self.OR_torque.tolist()
        ]
        self.csv_writer.writerow(row)

    # ------------------ Timers ------------------
    def print_progress(self, _event):
        elapsed = rospy.get_time() - self.start_time
        pct = int((elapsed / self.duration) * 100)
        pct5 = (pct // 5) * 5
        if pct5 != self.last_progress and pct5 <= 100:
            rospy.loginfo(f"Enregistrement en cours : {pct5}%")
            self.last_progress = pct5

    def stop_recording(self, _event):
        rospy.loginfo(f"Enregistrement terminé après {self.duration} secondes.")
        rospy.signal_shutdown("Durée écoulée")

    # ------------------ Shutdown ------------------
    def shutdown(self):
        self.capturing = False
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        self.serial_running = False
        if self.serial_thread and self.serial_thread.is_alive():
            self.serial_thread.join(timeout=2.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.pipeline:
            self.pipeline.stop()
        if self.video_writer:
            self.video_writer.release()
        if not self.csv_file.closed:
            self.csv_file.close()

        rospy.loginfo(f"CSV saved: {self.csv_path}")
        if self.video_writer:
            rospy.loginfo(f"Video saved: {self.video_path}")
        if self.save_images:
            rospy.loginfo(f"Images in: {self.image_dir}")


if __name__ == '__main__':
    try:
        WrenchAndCameraLogger()
    except rospy.ROSInterruptException:
        pass
