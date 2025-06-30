; Auto-generated. Do not edit!


(cl:in-package rokubimini_msgs-msg)


;//! \htmlinclude Reading.msg.html

(cl:defclass <Reading> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (statusword
    :reader statusword
    :initarg :statusword
    :type cl:integer
    :initform 0)
   (imu
    :reader imu
    :initarg :imu
    :type sensor_msgs-msg:Imu
    :initform (cl:make-instance 'sensor_msgs-msg:Imu))
   (wrench
    :reader wrench
    :initarg :wrench
    :type geometry_msgs-msg:WrenchStamped
    :initform (cl:make-instance 'geometry_msgs-msg:WrenchStamped))
   (externalImu
    :reader externalImu
    :initarg :externalImu
    :type sensor_msgs-msg:Imu
    :initform (cl:make-instance 'sensor_msgs-msg:Imu))
   (isForceTorqueSaturated
    :reader isForceTorqueSaturated
    :initarg :isForceTorqueSaturated
    :type cl:boolean
    :initform cl:nil)
   (temperature
    :reader temperature
    :initarg :temperature
    :type sensor_msgs-msg:Temperature
    :initform (cl:make-instance 'sensor_msgs-msg:Temperature)))
)

(cl:defclass Reading (<Reading>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Reading>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Reading)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokubimini_msgs-msg:<Reading> is deprecated: use rokubimini_msgs-msg:Reading instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Reading>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokubimini_msgs-msg:header-val is deprecated.  Use rokubimini_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'statusword-val :lambda-list '(m))
(cl:defmethod statusword-val ((m <Reading>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokubimini_msgs-msg:statusword-val is deprecated.  Use rokubimini_msgs-msg:statusword instead.")
  (statusword m))

(cl:ensure-generic-function 'imu-val :lambda-list '(m))
(cl:defmethod imu-val ((m <Reading>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokubimini_msgs-msg:imu-val is deprecated.  Use rokubimini_msgs-msg:imu instead.")
  (imu m))

(cl:ensure-generic-function 'wrench-val :lambda-list '(m))
(cl:defmethod wrench-val ((m <Reading>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokubimini_msgs-msg:wrench-val is deprecated.  Use rokubimini_msgs-msg:wrench instead.")
  (wrench m))

(cl:ensure-generic-function 'externalImu-val :lambda-list '(m))
(cl:defmethod externalImu-val ((m <Reading>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokubimini_msgs-msg:externalImu-val is deprecated.  Use rokubimini_msgs-msg:externalImu instead.")
  (externalImu m))

(cl:ensure-generic-function 'isForceTorqueSaturated-val :lambda-list '(m))
(cl:defmethod isForceTorqueSaturated-val ((m <Reading>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokubimini_msgs-msg:isForceTorqueSaturated-val is deprecated.  Use rokubimini_msgs-msg:isForceTorqueSaturated instead.")
  (isForceTorqueSaturated m))

(cl:ensure-generic-function 'temperature-val :lambda-list '(m))
(cl:defmethod temperature-val ((m <Reading>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokubimini_msgs-msg:temperature-val is deprecated.  Use rokubimini_msgs-msg:temperature instead.")
  (temperature m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Reading>) ostream)
  "Serializes a message object of type '<Reading>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'statusword)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'statusword)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'statusword)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'statusword)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'imu) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'wrench) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'externalImu) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isForceTorqueSaturated) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'temperature) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Reading>) istream)
  "Deserializes a message object of type '<Reading>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'statusword)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'statusword)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'statusword)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'statusword)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'imu) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'wrench) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'externalImu) istream)
    (cl:setf (cl:slot-value msg 'isForceTorqueSaturated) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'temperature) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Reading>)))
  "Returns string type for a message object of type '<Reading>"
  "rokubimini_msgs/Reading")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Reading)))
  "Returns string type for a message object of type 'Reading"
  "rokubimini_msgs/Reading")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Reading>)))
  "Returns md5sum for a message object of type '<Reading>"
  "f632d7286fac45ac13e8083c2bf7f237")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Reading)))
  "Returns md5sum for a message object of type 'Reading"
  "f632d7286fac45ac13e8083c2bf7f237")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Reading>)))
  "Returns full string definition for message of type '<Reading>"
  (cl:format cl:nil "# Reading~%~%# Message header.~%Header header~%# Statusword.~%uint32 statusword~%sensor_msgs/Imu imu~%geometry_msgs/WrenchStamped wrench~%sensor_msgs/Imu externalImu~%bool isForceTorqueSaturated~%sensor_msgs/Temperature temperature~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Imu~%# This is a message to hold data from an IMU (Inertial Measurement Unit)~%#~%# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec~%#~%# If the covariance of the measurement is known, it should be filled in (if all you know is the ~%# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)~%# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the~%# data a covariance will have to be assumed or gotten from some other source~%#~%# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation ~%# estimate), please set element 0 of the associated covariance matrix to -1~%# If you are interpreting this message, please check for a value of -1 in the first element of each ~%# covariance matrix, and disregard the associated estimate.~%~%Header header~%~%geometry_msgs/Quaternion orientation~%float64[9] orientation_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 angular_velocity~%float64[9] angular_velocity_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 linear_acceleration~%float64[9] linear_acceleration_covariance # Row major x, y z ~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/WrenchStamped~%# A wrench with reference coordinate frame and timestamp~%Header header~%Wrench wrench~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: sensor_msgs/Temperature~%# Single temperature reading.~%~%Header header           # timestamp is the time the temperature was measured~%                         # frame_id is the location of the temperature reading~%~%float64 temperature     # Measurement of the Temperature in Degrees Celsius~%~%float64 variance        # 0 is interpreted as variance unknown~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Reading)))
  "Returns full string definition for message of type 'Reading"
  (cl:format cl:nil "# Reading~%~%# Message header.~%Header header~%# Statusword.~%uint32 statusword~%sensor_msgs/Imu imu~%geometry_msgs/WrenchStamped wrench~%sensor_msgs/Imu externalImu~%bool isForceTorqueSaturated~%sensor_msgs/Temperature temperature~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Imu~%# This is a message to hold data from an IMU (Inertial Measurement Unit)~%#~%# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec~%#~%# If the covariance of the measurement is known, it should be filled in (if all you know is the ~%# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)~%# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the~%# data a covariance will have to be assumed or gotten from some other source~%#~%# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation ~%# estimate), please set element 0 of the associated covariance matrix to -1~%# If you are interpreting this message, please check for a value of -1 in the first element of each ~%# covariance matrix, and disregard the associated estimate.~%~%Header header~%~%geometry_msgs/Quaternion orientation~%float64[9] orientation_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 angular_velocity~%float64[9] angular_velocity_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 linear_acceleration~%float64[9] linear_acceleration_covariance # Row major x, y z ~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/WrenchStamped~%# A wrench with reference coordinate frame and timestamp~%Header header~%Wrench wrench~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: sensor_msgs/Temperature~%# Single temperature reading.~%~%Header header           # timestamp is the time the temperature was measured~%                         # frame_id is the location of the temperature reading~%~%float64 temperature     # Measurement of the Temperature in Degrees Celsius~%~%float64 variance        # 0 is interpreted as variance unknown~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Reading>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'imu))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'wrench))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'externalImu))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'temperature))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Reading>))
  "Converts a ROS message object to a list"
  (cl:list 'Reading
    (cl:cons ':header (header msg))
    (cl:cons ':statusword (statusword msg))
    (cl:cons ':imu (imu msg))
    (cl:cons ':wrench (wrench msg))
    (cl:cons ':externalImu (externalImu msg))
    (cl:cons ':isForceTorqueSaturated (isForceTorqueSaturated msg))
    (cl:cons ':temperature (temperature msg))
))
