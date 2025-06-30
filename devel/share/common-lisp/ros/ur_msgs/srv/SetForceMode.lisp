; Auto-generated. Do not edit!


(cl:in-package ur_msgs-srv)


;//! \htmlinclude SetForceMode-request.msg.html

(cl:defclass <SetForceMode-request> (roslisp-msg-protocol:ros-message)
  ((task_frame
    :reader task_frame
    :initarg :task_frame
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (selection_vector_x
    :reader selection_vector_x
    :initarg :selection_vector_x
    :type cl:boolean
    :initform cl:nil)
   (selection_vector_y
    :reader selection_vector_y
    :initarg :selection_vector_y
    :type cl:boolean
    :initform cl:nil)
   (selection_vector_z
    :reader selection_vector_z
    :initarg :selection_vector_z
    :type cl:boolean
    :initform cl:nil)
   (selection_vector_rx
    :reader selection_vector_rx
    :initarg :selection_vector_rx
    :type cl:boolean
    :initform cl:nil)
   (selection_vector_ry
    :reader selection_vector_ry
    :initarg :selection_vector_ry
    :type cl:boolean
    :initform cl:nil)
   (selection_vector_rz
    :reader selection_vector_rz
    :initarg :selection_vector_rz
    :type cl:boolean
    :initform cl:nil)
   (wrench
    :reader wrench
    :initarg :wrench
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (speed_limits
    :reader speed_limits
    :initarg :speed_limits
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (deviation_limits
    :reader deviation_limits
    :initarg :deviation_limits
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (damping_factor
    :reader damping_factor
    :initarg :damping_factor
    :type cl:float
    :initform 0.0)
   (gain_scaling
    :reader gain_scaling
    :initarg :gain_scaling
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetForceMode-request (<SetForceMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetForceMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetForceMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur_msgs-srv:<SetForceMode-request> is deprecated: use ur_msgs-srv:SetForceMode-request instead.")))

(cl:ensure-generic-function 'task_frame-val :lambda-list '(m))
(cl:defmethod task_frame-val ((m <SetForceMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:task_frame-val is deprecated.  Use ur_msgs-srv:task_frame instead.")
  (task_frame m))

(cl:ensure-generic-function 'selection_vector_x-val :lambda-list '(m))
(cl:defmethod selection_vector_x-val ((m <SetForceMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:selection_vector_x-val is deprecated.  Use ur_msgs-srv:selection_vector_x instead.")
  (selection_vector_x m))

(cl:ensure-generic-function 'selection_vector_y-val :lambda-list '(m))
(cl:defmethod selection_vector_y-val ((m <SetForceMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:selection_vector_y-val is deprecated.  Use ur_msgs-srv:selection_vector_y instead.")
  (selection_vector_y m))

(cl:ensure-generic-function 'selection_vector_z-val :lambda-list '(m))
(cl:defmethod selection_vector_z-val ((m <SetForceMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:selection_vector_z-val is deprecated.  Use ur_msgs-srv:selection_vector_z instead.")
  (selection_vector_z m))

(cl:ensure-generic-function 'selection_vector_rx-val :lambda-list '(m))
(cl:defmethod selection_vector_rx-val ((m <SetForceMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:selection_vector_rx-val is deprecated.  Use ur_msgs-srv:selection_vector_rx instead.")
  (selection_vector_rx m))

(cl:ensure-generic-function 'selection_vector_ry-val :lambda-list '(m))
(cl:defmethod selection_vector_ry-val ((m <SetForceMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:selection_vector_ry-val is deprecated.  Use ur_msgs-srv:selection_vector_ry instead.")
  (selection_vector_ry m))

(cl:ensure-generic-function 'selection_vector_rz-val :lambda-list '(m))
(cl:defmethod selection_vector_rz-val ((m <SetForceMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:selection_vector_rz-val is deprecated.  Use ur_msgs-srv:selection_vector_rz instead.")
  (selection_vector_rz m))

(cl:ensure-generic-function 'wrench-val :lambda-list '(m))
(cl:defmethod wrench-val ((m <SetForceMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:wrench-val is deprecated.  Use ur_msgs-srv:wrench instead.")
  (wrench m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <SetForceMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:type-val is deprecated.  Use ur_msgs-srv:type instead.")
  (type m))

(cl:ensure-generic-function 'speed_limits-val :lambda-list '(m))
(cl:defmethod speed_limits-val ((m <SetForceMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:speed_limits-val is deprecated.  Use ur_msgs-srv:speed_limits instead.")
  (speed_limits m))

(cl:ensure-generic-function 'deviation_limits-val :lambda-list '(m))
(cl:defmethod deviation_limits-val ((m <SetForceMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:deviation_limits-val is deprecated.  Use ur_msgs-srv:deviation_limits instead.")
  (deviation_limits m))

(cl:ensure-generic-function 'damping_factor-val :lambda-list '(m))
(cl:defmethod damping_factor-val ((m <SetForceMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:damping_factor-val is deprecated.  Use ur_msgs-srv:damping_factor instead.")
  (damping_factor m))

(cl:ensure-generic-function 'gain_scaling-val :lambda-list '(m))
(cl:defmethod gain_scaling-val ((m <SetForceMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:gain_scaling-val is deprecated.  Use ur_msgs-srv:gain_scaling instead.")
  (gain_scaling m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SetForceMode-request>)))
    "Constants for message type '<SetForceMode-request>"
  '((:TCP_TO_ORIGIN . 1)
    (:NO_TRANSFORM . 2)
    (:TCP_VELOCITY_TO_X_Y . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SetForceMode-request)))
    "Constants for message type 'SetForceMode-request"
  '((:TCP_TO_ORIGIN . 1)
    (:NO_TRANSFORM . 2)
    (:TCP_VELOCITY_TO_X_Y . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetForceMode-request>) ostream)
  "Serializes a message object of type '<SetForceMode-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'task_frame) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'selection_vector_x) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'selection_vector_y) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'selection_vector_z) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'selection_vector_rx) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'selection_vector_ry) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'selection_vector_rz) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'wrench) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'speed_limits) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'deviation_limits))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'damping_factor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gain_scaling))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetForceMode-request>) istream)
  "Deserializes a message object of type '<SetForceMode-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'task_frame) istream)
    (cl:setf (cl:slot-value msg 'selection_vector_x) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'selection_vector_y) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'selection_vector_z) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'selection_vector_rx) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'selection_vector_ry) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'selection_vector_rz) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'wrench) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'speed_limits) istream)
  (cl:setf (cl:slot-value msg 'deviation_limits) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'deviation_limits)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'damping_factor) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gain_scaling) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetForceMode-request>)))
  "Returns string type for a service object of type '<SetForceMode-request>"
  "ur_msgs/SetForceModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetForceMode-request)))
  "Returns string type for a service object of type 'SetForceMode-request"
  "ur_msgs/SetForceModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetForceMode-request>)))
  "Returns md5sum for a message object of type '<SetForceMode-request>"
  "ad1bcbfccfedd053ec6bb7eb365913d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetForceMode-request)))
  "Returns md5sum for a message object of type 'SetForceMode-request"
  "ad1bcbfccfedd053ec6bb7eb365913d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetForceMode-request>)))
  "Returns full string definition for message of type '<SetForceMode-request>"
  (cl:format cl:nil "# A 6d pose vector that defines the force frame relative to the base frame~%geometry_msgs/PoseStamped task_frame        ~%~%# A 6d vector of 0s and 1s. 1 means that the robot will be compliant in the corresponding axis of the task frame~%bool selection_vector_x~%bool selection_vector_y~%bool selection_vector_z~%bool selection_vector_rx~%bool selection_vector_ry~%bool selection_vector_rz  ~%~%# The forces/torques the robot will apply to its environment. For geometric interpretation, please~%# see parameter `type`~%geometry_msgs/Wrench wrench~%~%# An integer [1;3] specifying how the robot interprets the force frame~%# 1: The force frame is transformed in a way such that its y-axis is aligned with a vector pointing~%#    from the robot tcp towards the origin of the force frame.~%# 2: The force frame is not transformed.~%# 3: The force frame is transformed in a way such that its x-axis is the projection of the robot tcp~%#     velocity vector onto the x-y plane of the force frame.~%uint8 type~%# Type constants:~%uint8 TCP_TO_ORIGIN=1~%uint8 NO_TRANSFORM=2~%uint8 TCP_VELOCITY_TO_X_Y=3~%~%# Maximum allowed tcp speed (relative to the task frame).~%# PLEASE NOTE: This is only relevant for axes marked as compliant in the selection_vector~%geometry_msgs/Twist speed_limits~%~%# For non-compliant axes, these values are the maximum allowed deviation along/about an axis~%# between the actual tcp position and the one set by the program.~%float32[6] deviation_limits~%~%# Force mode damping factor. Sets the damping parameter in force mode. In range [0;1], default value is 0.025~%# A value of 1 is full damping, so the robot will decelerate quickly if no force is present. A value of 0~%# is no damping, here the robot will maintain the speed.~%float32 damping_factor~%~%# Force mode gain scaling factor. Scales the gain in force mode. scaling parameter is in range [0;2], default is 0.5. ~%# A value larger than 1 can make force mode unstable, e.g. in case of collisions or pushing against hard surfaces.~%float32 gain_scaling~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3 linear~%Vector3 angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetForceMode-request)))
  "Returns full string definition for message of type 'SetForceMode-request"
  (cl:format cl:nil "# A 6d pose vector that defines the force frame relative to the base frame~%geometry_msgs/PoseStamped task_frame        ~%~%# A 6d vector of 0s and 1s. 1 means that the robot will be compliant in the corresponding axis of the task frame~%bool selection_vector_x~%bool selection_vector_y~%bool selection_vector_z~%bool selection_vector_rx~%bool selection_vector_ry~%bool selection_vector_rz  ~%~%# The forces/torques the robot will apply to its environment. For geometric interpretation, please~%# see parameter `type`~%geometry_msgs/Wrench wrench~%~%# An integer [1;3] specifying how the robot interprets the force frame~%# 1: The force frame is transformed in a way such that its y-axis is aligned with a vector pointing~%#    from the robot tcp towards the origin of the force frame.~%# 2: The force frame is not transformed.~%# 3: The force frame is transformed in a way such that its x-axis is the projection of the robot tcp~%#     velocity vector onto the x-y plane of the force frame.~%uint8 type~%# Type constants:~%uint8 TCP_TO_ORIGIN=1~%uint8 NO_TRANSFORM=2~%uint8 TCP_VELOCITY_TO_X_Y=3~%~%# Maximum allowed tcp speed (relative to the task frame).~%# PLEASE NOTE: This is only relevant for axes marked as compliant in the selection_vector~%geometry_msgs/Twist speed_limits~%~%# For non-compliant axes, these values are the maximum allowed deviation along/about an axis~%# between the actual tcp position and the one set by the program.~%float32[6] deviation_limits~%~%# Force mode damping factor. Sets the damping parameter in force mode. In range [0;1], default value is 0.025~%# A value of 1 is full damping, so the robot will decelerate quickly if no force is present. A value of 0~%# is no damping, here the robot will maintain the speed.~%float32 damping_factor~%~%# Force mode gain scaling factor. Scales the gain in force mode. scaling parameter is in range [0;2], default is 0.5. ~%# A value larger than 1 can make force mode unstable, e.g. in case of collisions or pushing against hard surfaces.~%float32 gain_scaling~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3 linear~%Vector3 angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetForceMode-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'task_frame))
     1
     1
     1
     1
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'wrench))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'speed_limits))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'deviation_limits) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetForceMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetForceMode-request
    (cl:cons ':task_frame (task_frame msg))
    (cl:cons ':selection_vector_x (selection_vector_x msg))
    (cl:cons ':selection_vector_y (selection_vector_y msg))
    (cl:cons ':selection_vector_z (selection_vector_z msg))
    (cl:cons ':selection_vector_rx (selection_vector_rx msg))
    (cl:cons ':selection_vector_ry (selection_vector_ry msg))
    (cl:cons ':selection_vector_rz (selection_vector_rz msg))
    (cl:cons ':wrench (wrench msg))
    (cl:cons ':type (type msg))
    (cl:cons ':speed_limits (speed_limits msg))
    (cl:cons ':deviation_limits (deviation_limits msg))
    (cl:cons ':damping_factor (damping_factor msg))
    (cl:cons ':gain_scaling (gain_scaling msg))
))
;//! \htmlinclude SetForceMode-response.msg.html

(cl:defclass <SetForceMode-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetForceMode-response (<SetForceMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetForceMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetForceMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur_msgs-srv:<SetForceMode-response> is deprecated: use ur_msgs-srv:SetForceMode-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetForceMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:success-val is deprecated.  Use ur_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetForceMode-response>) ostream)
  "Serializes a message object of type '<SetForceMode-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetForceMode-response>) istream)
  "Deserializes a message object of type '<SetForceMode-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetForceMode-response>)))
  "Returns string type for a service object of type '<SetForceMode-response>"
  "ur_msgs/SetForceModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetForceMode-response)))
  "Returns string type for a service object of type 'SetForceMode-response"
  "ur_msgs/SetForceModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetForceMode-response>)))
  "Returns md5sum for a message object of type '<SetForceMode-response>"
  "ad1bcbfccfedd053ec6bb7eb365913d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetForceMode-response)))
  "Returns md5sum for a message object of type 'SetForceMode-response"
  "ad1bcbfccfedd053ec6bb7eb365913d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetForceMode-response>)))
  "Returns full string definition for message of type '<SetForceMode-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetForceMode-response)))
  "Returns full string definition for message of type 'SetForceMode-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetForceMode-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetForceMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetForceMode-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetForceMode)))
  'SetForceMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetForceMode)))
  'SetForceMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetForceMode)))
  "Returns string type for a service object of type '<SetForceMode>"
  "ur_msgs/SetForceMode")