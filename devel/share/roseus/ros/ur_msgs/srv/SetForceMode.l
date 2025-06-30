;; Auto-generated. Do not edit!


(when (boundp 'ur_msgs::SetForceMode)
  (if (not (find-package "UR_MSGS"))
    (make-package "UR_MSGS"))
  (shadow 'SetForceMode (find-package "UR_MSGS")))
(unless (find-package "UR_MSGS::SETFORCEMODE")
  (make-package "UR_MSGS::SETFORCEMODE"))
(unless (find-package "UR_MSGS::SETFORCEMODEREQUEST")
  (make-package "UR_MSGS::SETFORCEMODEREQUEST"))
(unless (find-package "UR_MSGS::SETFORCEMODERESPONSE")
  (make-package "UR_MSGS::SETFORCEMODERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(intern "*TCP_TO_ORIGIN*" (find-package "UR_MSGS::SETFORCEMODEREQUEST"))
(shadow '*TCP_TO_ORIGIN* (find-package "UR_MSGS::SETFORCEMODEREQUEST"))
(defconstant ur_msgs::SetForceModeRequest::*TCP_TO_ORIGIN* 1)
(intern "*NO_TRANSFORM*" (find-package "UR_MSGS::SETFORCEMODEREQUEST"))
(shadow '*NO_TRANSFORM* (find-package "UR_MSGS::SETFORCEMODEREQUEST"))
(defconstant ur_msgs::SetForceModeRequest::*NO_TRANSFORM* 2)
(intern "*TCP_VELOCITY_TO_X_Y*" (find-package "UR_MSGS::SETFORCEMODEREQUEST"))
(shadow '*TCP_VELOCITY_TO_X_Y* (find-package "UR_MSGS::SETFORCEMODEREQUEST"))
(defconstant ur_msgs::SetForceModeRequest::*TCP_VELOCITY_TO_X_Y* 3)

(defun ur_msgs::SetForceModeRequest-to-symbol (const)
  (cond
        ((= const 1) 'ur_msgs::SetForceModeRequest::*TCP_TO_ORIGIN*)
        ((= const 2) 'ur_msgs::SetForceModeRequest::*NO_TRANSFORM*)
        ((= const 3) 'ur_msgs::SetForceModeRequest::*TCP_VELOCITY_TO_X_Y*)
        (t nil)))

(defclass ur_msgs::SetForceModeRequest
  :super ros::object
  :slots (_task_frame _selection_vector_x _selection_vector_y _selection_vector_z _selection_vector_rx _selection_vector_ry _selection_vector_rz _wrench _type _speed_limits _deviation_limits _damping_factor _gain_scaling ))

(defmethod ur_msgs::SetForceModeRequest
  (:init
   (&key
    ((:task_frame __task_frame) (instance geometry_msgs::PoseStamped :init))
    ((:selection_vector_x __selection_vector_x) nil)
    ((:selection_vector_y __selection_vector_y) nil)
    ((:selection_vector_z __selection_vector_z) nil)
    ((:selection_vector_rx __selection_vector_rx) nil)
    ((:selection_vector_ry __selection_vector_ry) nil)
    ((:selection_vector_rz __selection_vector_rz) nil)
    ((:wrench __wrench) (instance geometry_msgs::Wrench :init))
    ((:type __type) 0)
    ((:speed_limits __speed_limits) (instance geometry_msgs::Twist :init))
    ((:deviation_limits __deviation_limits) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:damping_factor __damping_factor) 0.0)
    ((:gain_scaling __gain_scaling) 0.0)
    )
   (send-super :init)
   (setq _task_frame __task_frame)
   (setq _selection_vector_x __selection_vector_x)
   (setq _selection_vector_y __selection_vector_y)
   (setq _selection_vector_z __selection_vector_z)
   (setq _selection_vector_rx __selection_vector_rx)
   (setq _selection_vector_ry __selection_vector_ry)
   (setq _selection_vector_rz __selection_vector_rz)
   (setq _wrench __wrench)
   (setq _type (round __type))
   (setq _speed_limits __speed_limits)
   (setq _deviation_limits __deviation_limits)
   (setq _damping_factor (float __damping_factor))
   (setq _gain_scaling (float __gain_scaling))
   self)
  (:task_frame
   (&rest __task_frame)
   (if (keywordp (car __task_frame))
       (send* _task_frame __task_frame)
     (progn
       (if __task_frame (setq _task_frame (car __task_frame)))
       _task_frame)))
  (:selection_vector_x
   (&optional (__selection_vector_x :null))
   (if (not (eq __selection_vector_x :null)) (setq _selection_vector_x __selection_vector_x)) _selection_vector_x)
  (:selection_vector_y
   (&optional (__selection_vector_y :null))
   (if (not (eq __selection_vector_y :null)) (setq _selection_vector_y __selection_vector_y)) _selection_vector_y)
  (:selection_vector_z
   (&optional (__selection_vector_z :null))
   (if (not (eq __selection_vector_z :null)) (setq _selection_vector_z __selection_vector_z)) _selection_vector_z)
  (:selection_vector_rx
   (&optional (__selection_vector_rx :null))
   (if (not (eq __selection_vector_rx :null)) (setq _selection_vector_rx __selection_vector_rx)) _selection_vector_rx)
  (:selection_vector_ry
   (&optional (__selection_vector_ry :null))
   (if (not (eq __selection_vector_ry :null)) (setq _selection_vector_ry __selection_vector_ry)) _selection_vector_ry)
  (:selection_vector_rz
   (&optional (__selection_vector_rz :null))
   (if (not (eq __selection_vector_rz :null)) (setq _selection_vector_rz __selection_vector_rz)) _selection_vector_rz)
  (:wrench
   (&rest __wrench)
   (if (keywordp (car __wrench))
       (send* _wrench __wrench)
     (progn
       (if __wrench (setq _wrench (car __wrench)))
       _wrench)))
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:speed_limits
   (&rest __speed_limits)
   (if (keywordp (car __speed_limits))
       (send* _speed_limits __speed_limits)
     (progn
       (if __speed_limits (setq _speed_limits (car __speed_limits)))
       _speed_limits)))
  (:deviation_limits
   (&optional __deviation_limits)
   (if __deviation_limits (setq _deviation_limits __deviation_limits)) _deviation_limits)
  (:damping_factor
   (&optional __damping_factor)
   (if __damping_factor (setq _damping_factor __damping_factor)) _damping_factor)
  (:gain_scaling
   (&optional __gain_scaling)
   (if __gain_scaling (setq _gain_scaling __gain_scaling)) _gain_scaling)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped _task_frame
    (send _task_frame :serialization-length)
    ;; bool _selection_vector_x
    1
    ;; bool _selection_vector_y
    1
    ;; bool _selection_vector_z
    1
    ;; bool _selection_vector_rx
    1
    ;; bool _selection_vector_ry
    1
    ;; bool _selection_vector_rz
    1
    ;; geometry_msgs/Wrench _wrench
    (send _wrench :serialization-length)
    ;; uint8 _type
    1
    ;; geometry_msgs/Twist _speed_limits
    (send _speed_limits :serialization-length)
    ;; float32[6] _deviation_limits
    (* 4    6)
    ;; float32 _damping_factor
    4
    ;; float32 _gain_scaling
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped _task_frame
       (send _task_frame :serialize s)
     ;; bool _selection_vector_x
       (if _selection_vector_x (write-byte -1 s) (write-byte 0 s))
     ;; bool _selection_vector_y
       (if _selection_vector_y (write-byte -1 s) (write-byte 0 s))
     ;; bool _selection_vector_z
       (if _selection_vector_z (write-byte -1 s) (write-byte 0 s))
     ;; bool _selection_vector_rx
       (if _selection_vector_rx (write-byte -1 s) (write-byte 0 s))
     ;; bool _selection_vector_ry
       (if _selection_vector_ry (write-byte -1 s) (write-byte 0 s))
     ;; bool _selection_vector_rz
       (if _selection_vector_rz (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Wrench _wrench
       (send _wrench :serialize s)
     ;; uint8 _type
       (write-byte _type s)
     ;; geometry_msgs/Twist _speed_limits
       (send _speed_limits :serialize s)
     ;; float32[6] _deviation_limits
     (dotimes (i 6)
       (sys::poke (elt _deviation_limits i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _damping_factor
       (sys::poke _damping_factor (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gain_scaling
       (sys::poke _gain_scaling (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped _task_frame
     (send _task_frame :deserialize buf ptr-) (incf ptr- (send _task_frame :serialization-length))
   ;; bool _selection_vector_x
     (setq _selection_vector_x (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _selection_vector_y
     (setq _selection_vector_y (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _selection_vector_z
     (setq _selection_vector_z (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _selection_vector_rx
     (setq _selection_vector_rx (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _selection_vector_ry
     (setq _selection_vector_ry (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _selection_vector_rz
     (setq _selection_vector_rz (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Wrench _wrench
     (send _wrench :deserialize buf ptr-) (incf ptr- (send _wrench :serialization-length))
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Twist _speed_limits
     (send _speed_limits :deserialize buf ptr-) (incf ptr- (send _speed_limits :serialization-length))
   ;; float32[6] _deviation_limits
   (dotimes (i (length _deviation_limits))
     (setf (elt _deviation_limits i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32 _damping_factor
     (setq _damping_factor (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gain_scaling
     (setq _gain_scaling (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ur_msgs::SetForceModeResponse
  :super ros::object
  :slots (_success ))

(defmethod ur_msgs::SetForceModeResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ur_msgs::SetForceMode
  :super ros::object
  :slots ())

(setf (get ur_msgs::SetForceMode :md5sum-) "ad1bcbfccfedd053ec6bb7eb365913d8")
(setf (get ur_msgs::SetForceMode :datatype-) "ur_msgs/SetForceMode")
(setf (get ur_msgs::SetForceMode :request) ur_msgs::SetForceModeRequest)
(setf (get ur_msgs::SetForceMode :response) ur_msgs::SetForceModeResponse)

(defmethod ur_msgs::SetForceModeRequest
  (:response () (instance ur_msgs::SetForceModeResponse :init)))

(setf (get ur_msgs::SetForceModeRequest :md5sum-) "ad1bcbfccfedd053ec6bb7eb365913d8")
(setf (get ur_msgs::SetForceModeRequest :datatype-) "ur_msgs/SetForceModeRequest")
(setf (get ur_msgs::SetForceModeRequest :definition-)
      "# A 6d pose vector that defines the force frame relative to the base frame
geometry_msgs/PoseStamped task_frame        

# A 6d vector of 0s and 1s. 1 means that the robot will be compliant in the corresponding axis of the task frame
bool selection_vector_x
bool selection_vector_y
bool selection_vector_z
bool selection_vector_rx
bool selection_vector_ry
bool selection_vector_rz  

# The forces/torques the robot will apply to its environment. For geometric interpretation, please
# see parameter `type`
geometry_msgs/Wrench wrench

# An integer [1;3] specifying how the robot interprets the force frame
# 1: The force frame is transformed in a way such that its y-axis is aligned with a vector pointing
#    from the robot tcp towards the origin of the force frame.
# 2: The force frame is not transformed.
# 3: The force frame is transformed in a way such that its x-axis is the projection of the robot tcp
#     velocity vector onto the x-y plane of the force frame.
uint8 type
# Type constants:
uint8 TCP_TO_ORIGIN=1
uint8 NO_TRANSFORM=2
uint8 TCP_VELOCITY_TO_X_Y=3

# Maximum allowed tcp speed (relative to the task frame).
# PLEASE NOTE: This is only relevant for axes marked as compliant in the selection_vector
geometry_msgs/Twist speed_limits

# For non-compliant axes, these values are the maximum allowed deviation along/about an axis
# between the actual tcp position and the one set by the program.
float32[6] deviation_limits

# Force mode damping factor. Sets the damping parameter in force mode. In range [0;1], default value is 0.025
# A value of 1 is full damping, so the robot will decelerate quickly if no force is present. A value of 0
# is no damping, here the robot will maintain the speed.
float32 damping_factor

# Force mode gain scaling factor. Scales the gain in force mode. scaling parameter is in range [0;2], default is 0.5. 
# A value larger than 1 can make force mode unstable, e.g. in case of collisions or pushing against hard surfaces.
float32 gain_scaling

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3 linear
Vector3 angular
---
bool success
")

(setf (get ur_msgs::SetForceModeResponse :md5sum-) "ad1bcbfccfedd053ec6bb7eb365913d8")
(setf (get ur_msgs::SetForceModeResponse :datatype-) "ur_msgs/SetForceModeResponse")
(setf (get ur_msgs::SetForceModeResponse :definition-)
      "# A 6d pose vector that defines the force frame relative to the base frame
geometry_msgs/PoseStamped task_frame        

# A 6d vector of 0s and 1s. 1 means that the robot will be compliant in the corresponding axis of the task frame
bool selection_vector_x
bool selection_vector_y
bool selection_vector_z
bool selection_vector_rx
bool selection_vector_ry
bool selection_vector_rz  

# The forces/torques the robot will apply to its environment. For geometric interpretation, please
# see parameter `type`
geometry_msgs/Wrench wrench

# An integer [1;3] specifying how the robot interprets the force frame
# 1: The force frame is transformed in a way such that its y-axis is aligned with a vector pointing
#    from the robot tcp towards the origin of the force frame.
# 2: The force frame is not transformed.
# 3: The force frame is transformed in a way such that its x-axis is the projection of the robot tcp
#     velocity vector onto the x-y plane of the force frame.
uint8 type
# Type constants:
uint8 TCP_TO_ORIGIN=1
uint8 NO_TRANSFORM=2
uint8 TCP_VELOCITY_TO_X_Y=3

# Maximum allowed tcp speed (relative to the task frame).
# PLEASE NOTE: This is only relevant for axes marked as compliant in the selection_vector
geometry_msgs/Twist speed_limits

# For non-compliant axes, these values are the maximum allowed deviation along/about an axis
# between the actual tcp position and the one set by the program.
float32[6] deviation_limits

# Force mode damping factor. Sets the damping parameter in force mode. In range [0;1], default value is 0.025
# A value of 1 is full damping, so the robot will decelerate quickly if no force is present. A value of 0
# is no damping, here the robot will maintain the speed.
float32 damping_factor

# Force mode gain scaling factor. Scales the gain in force mode. scaling parameter is in range [0;2], default is 0.5. 
# A value larger than 1 can make force mode unstable, e.g. in case of collisions or pushing against hard surfaces.
float32 gain_scaling

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3 linear
Vector3 angular
---
bool success
")



(provide :ur_msgs/SetForceMode "ad1bcbfccfedd053ec6bb7eb365913d8")


