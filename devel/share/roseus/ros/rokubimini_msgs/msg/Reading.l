;; Auto-generated. Do not edit!


(when (boundp 'rokubimini_msgs::Reading)
  (if (not (find-package "ROKUBIMINI_MSGS"))
    (make-package "ROKUBIMINI_MSGS"))
  (shadow 'Reading (find-package "ROKUBIMINI_MSGS")))
(unless (find-package "ROKUBIMINI_MSGS::READING")
  (make-package "ROKUBIMINI_MSGS::READING"))

(in-package "ROS")
;;//! \htmlinclude Reading.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rokubimini_msgs::Reading
  :super ros::object
  :slots (_header _statusword _imu _wrench _externalImu _isForceTorqueSaturated _temperature ))

(defmethod rokubimini_msgs::Reading
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:statusword __statusword) 0)
    ((:imu __imu) (instance sensor_msgs::Imu :init))
    ((:wrench __wrench) (instance geometry_msgs::WrenchStamped :init))
    ((:externalImu __externalImu) (instance sensor_msgs::Imu :init))
    ((:isForceTorqueSaturated __isForceTorqueSaturated) nil)
    ((:temperature __temperature) (instance sensor_msgs::Temperature :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _statusword (round __statusword))
   (setq _imu __imu)
   (setq _wrench __wrench)
   (setq _externalImu __externalImu)
   (setq _isForceTorqueSaturated __isForceTorqueSaturated)
   (setq _temperature __temperature)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:statusword
   (&optional __statusword)
   (if __statusword (setq _statusword __statusword)) _statusword)
  (:imu
   (&rest __imu)
   (if (keywordp (car __imu))
       (send* _imu __imu)
     (progn
       (if __imu (setq _imu (car __imu)))
       _imu)))
  (:wrench
   (&rest __wrench)
   (if (keywordp (car __wrench))
       (send* _wrench __wrench)
     (progn
       (if __wrench (setq _wrench (car __wrench)))
       _wrench)))
  (:externalImu
   (&rest __externalImu)
   (if (keywordp (car __externalImu))
       (send* _externalImu __externalImu)
     (progn
       (if __externalImu (setq _externalImu (car __externalImu)))
       _externalImu)))
  (:isForceTorqueSaturated
   (&optional (__isForceTorqueSaturated :null))
   (if (not (eq __isForceTorqueSaturated :null)) (setq _isForceTorqueSaturated __isForceTorqueSaturated)) _isForceTorqueSaturated)
  (:temperature
   (&rest __temperature)
   (if (keywordp (car __temperature))
       (send* _temperature __temperature)
     (progn
       (if __temperature (setq _temperature (car __temperature)))
       _temperature)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _statusword
    4
    ;; sensor_msgs/Imu _imu
    (send _imu :serialization-length)
    ;; geometry_msgs/WrenchStamped _wrench
    (send _wrench :serialization-length)
    ;; sensor_msgs/Imu _externalImu
    (send _externalImu :serialization-length)
    ;; bool _isForceTorqueSaturated
    1
    ;; sensor_msgs/Temperature _temperature
    (send _temperature :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _statusword
       (write-long _statusword s)
     ;; sensor_msgs/Imu _imu
       (send _imu :serialize s)
     ;; geometry_msgs/WrenchStamped _wrench
       (send _wrench :serialize s)
     ;; sensor_msgs/Imu _externalImu
       (send _externalImu :serialize s)
     ;; bool _isForceTorqueSaturated
       (if _isForceTorqueSaturated (write-byte -1 s) (write-byte 0 s))
     ;; sensor_msgs/Temperature _temperature
       (send _temperature :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _statusword
     (setq _statusword (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; sensor_msgs/Imu _imu
     (send _imu :deserialize buf ptr-) (incf ptr- (send _imu :serialization-length))
   ;; geometry_msgs/WrenchStamped _wrench
     (send _wrench :deserialize buf ptr-) (incf ptr- (send _wrench :serialization-length))
   ;; sensor_msgs/Imu _externalImu
     (send _externalImu :deserialize buf ptr-) (incf ptr- (send _externalImu :serialization-length))
   ;; bool _isForceTorqueSaturated
     (setq _isForceTorqueSaturated (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; sensor_msgs/Temperature _temperature
     (send _temperature :deserialize buf ptr-) (incf ptr- (send _temperature :serialization-length))
   ;;
   self)
  )

(setf (get rokubimini_msgs::Reading :md5sum-) "f632d7286fac45ac13e8083c2bf7f237")
(setf (get rokubimini_msgs::Reading :datatype-) "rokubimini_msgs/Reading")
(setf (get rokubimini_msgs::Reading :definition-)
      "# Reading

# Message header.
Header header
# Statusword.
uint32 statusword
sensor_msgs/Imu imu
geometry_msgs/WrenchStamped wrench
sensor_msgs/Imu externalImu
bool isForceTorqueSaturated
sensor_msgs/Temperature temperature


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
MSG: sensor_msgs/Imu
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the 
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
# estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each 
# covariance matrix, and disregard the associated estimate.

Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z 

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

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
MSG: geometry_msgs/WrenchStamped
# A wrench with reference coordinate frame and timestamp
Header header
Wrench wrench

================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

================================================================================
MSG: sensor_msgs/Temperature
# Single temperature reading.

Header header           # timestamp is the time the temperature was measured
                         # frame_id is the location of the temperature reading

float64 temperature     # Measurement of the Temperature in Degrees Celsius

float64 variance        # 0 is interpreted as variance unknown

")



(provide :rokubimini_msgs/Reading "f632d7286fac45ac13e8083c2bf7f237")


