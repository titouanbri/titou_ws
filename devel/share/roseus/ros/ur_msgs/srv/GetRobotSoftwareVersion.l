;; Auto-generated. Do not edit!


(when (boundp 'ur_msgs::GetRobotSoftwareVersion)
  (if (not (find-package "UR_MSGS"))
    (make-package "UR_MSGS"))
  (shadow 'GetRobotSoftwareVersion (find-package "UR_MSGS")))
(unless (find-package "UR_MSGS::GETROBOTSOFTWAREVERSION")
  (make-package "UR_MSGS::GETROBOTSOFTWAREVERSION"))
(unless (find-package "UR_MSGS::GETROBOTSOFTWAREVERSIONREQUEST")
  (make-package "UR_MSGS::GETROBOTSOFTWAREVERSIONREQUEST"))
(unless (find-package "UR_MSGS::GETROBOTSOFTWAREVERSIONRESPONSE")
  (make-package "UR_MSGS::GETROBOTSOFTWAREVERSIONRESPONSE"))

(in-package "ROS")





(defclass ur_msgs::GetRobotSoftwareVersionRequest
  :super ros::object
  :slots ())

(defmethod ur_msgs::GetRobotSoftwareVersionRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass ur_msgs::GetRobotSoftwareVersionResponse
  :super ros::object
  :slots (_major _minor _bugfix _build ))

(defmethod ur_msgs::GetRobotSoftwareVersionResponse
  (:init
   (&key
    ((:major __major) 0)
    ((:minor __minor) 0)
    ((:bugfix __bugfix) 0)
    ((:build __build) 0)
    )
   (send-super :init)
   (setq _major (round __major))
   (setq _minor (round __minor))
   (setq _bugfix (round __bugfix))
   (setq _build (round __build))
   self)
  (:major
   (&optional __major)
   (if __major (setq _major __major)) _major)
  (:minor
   (&optional __minor)
   (if __minor (setq _minor __minor)) _minor)
  (:bugfix
   (&optional __bugfix)
   (if __bugfix (setq _bugfix __bugfix)) _bugfix)
  (:build
   (&optional __build)
   (if __build (setq _build __build)) _build)
  (:serialization-length
   ()
   (+
    ;; uint32 _major
    4
    ;; uint32 _minor
    4
    ;; uint32 _bugfix
    4
    ;; uint32 _build
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _major
       (write-long _major s)
     ;; uint32 _minor
       (write-long _minor s)
     ;; uint32 _bugfix
       (write-long _bugfix s)
     ;; uint32 _build
       (write-long _build s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _major
     (setq _major (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _minor
     (setq _minor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _bugfix
     (setq _bugfix (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _build
     (setq _build (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ur_msgs::GetRobotSoftwareVersion
  :super ros::object
  :slots ())

(setf (get ur_msgs::GetRobotSoftwareVersion :md5sum-) "eea0f1664f7955042558cb2bf2c766ad")
(setf (get ur_msgs::GetRobotSoftwareVersion :datatype-) "ur_msgs/GetRobotSoftwareVersion")
(setf (get ur_msgs::GetRobotSoftwareVersion :request) ur_msgs::GetRobotSoftwareVersionRequest)
(setf (get ur_msgs::GetRobotSoftwareVersion :response) ur_msgs::GetRobotSoftwareVersionResponse)

(defmethod ur_msgs::GetRobotSoftwareVersionRequest
  (:response () (instance ur_msgs::GetRobotSoftwareVersionResponse :init)))

(setf (get ur_msgs::GetRobotSoftwareVersionRequest :md5sum-) "eea0f1664f7955042558cb2bf2c766ad")
(setf (get ur_msgs::GetRobotSoftwareVersionRequest :datatype-) "ur_msgs/GetRobotSoftwareVersionRequest")
(setf (get ur_msgs::GetRobotSoftwareVersionRequest :definition-)
      "---
uint32 major    # Major version number
uint32 minor    # Minor version number
uint32 bugfix   # Bugfix version number
uint32 build    # Build number

")

(setf (get ur_msgs::GetRobotSoftwareVersionResponse :md5sum-) "eea0f1664f7955042558cb2bf2c766ad")
(setf (get ur_msgs::GetRobotSoftwareVersionResponse :datatype-) "ur_msgs/GetRobotSoftwareVersionResponse")
(setf (get ur_msgs::GetRobotSoftwareVersionResponse :definition-)
      "---
uint32 major    # Major version number
uint32 minor    # Minor version number
uint32 bugfix   # Bugfix version number
uint32 build    # Build number

")



(provide :ur_msgs/GetRobotSoftwareVersion "eea0f1664f7955042558cb2bf2c766ad")


