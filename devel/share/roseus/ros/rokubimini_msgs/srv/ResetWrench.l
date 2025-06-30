;; Auto-generated. Do not edit!


(when (boundp 'rokubimini_msgs::ResetWrench)
  (if (not (find-package "ROKUBIMINI_MSGS"))
    (make-package "ROKUBIMINI_MSGS"))
  (shadow 'ResetWrench (find-package "ROKUBIMINI_MSGS")))
(unless (find-package "ROKUBIMINI_MSGS::RESETWRENCH")
  (make-package "ROKUBIMINI_MSGS::RESETWRENCH"))
(unless (find-package "ROKUBIMINI_MSGS::RESETWRENCHREQUEST")
  (make-package "ROKUBIMINI_MSGS::RESETWRENCHREQUEST"))
(unless (find-package "ROKUBIMINI_MSGS::RESETWRENCHRESPONSE")
  (make-package "ROKUBIMINI_MSGS::RESETWRENCHRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass rokubimini_msgs::ResetWrenchRequest
  :super ros::object
  :slots (_desired_wrench ))

(defmethod rokubimini_msgs::ResetWrenchRequest
  (:init
   (&key
    ((:desired_wrench __desired_wrench) (instance geometry_msgs::Wrench :init))
    )
   (send-super :init)
   (setq _desired_wrench __desired_wrench)
   self)
  (:desired_wrench
   (&rest __desired_wrench)
   (if (keywordp (car __desired_wrench))
       (send* _desired_wrench __desired_wrench)
     (progn
       (if __desired_wrench (setq _desired_wrench (car __desired_wrench)))
       _desired_wrench)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Wrench _desired_wrench
    (send _desired_wrench :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Wrench _desired_wrench
       (send _desired_wrench :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Wrench _desired_wrench
     (send _desired_wrench :deserialize buf ptr-) (incf ptr- (send _desired_wrench :serialization-length))
   ;;
   self)
  )

(defclass rokubimini_msgs::ResetWrenchResponse
  :super ros::object
  :slots (_success ))

(defmethod rokubimini_msgs::ResetWrenchResponse
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

(defclass rokubimini_msgs::ResetWrench
  :super ros::object
  :slots ())

(setf (get rokubimini_msgs::ResetWrench :md5sum-) "a01e1ece25b40c645838e2e60d7f441c")
(setf (get rokubimini_msgs::ResetWrench :datatype-) "rokubimini_msgs/ResetWrench")
(setf (get rokubimini_msgs::ResetWrench :request) rokubimini_msgs::ResetWrenchRequest)
(setf (get rokubimini_msgs::ResetWrench :response) rokubimini_msgs::ResetWrenchResponse)

(defmethod rokubimini_msgs::ResetWrenchRequest
  (:response () (instance rokubimini_msgs::ResetWrenchResponse :init)))

(setf (get rokubimini_msgs::ResetWrenchRequest :md5sum-) "a01e1ece25b40c645838e2e60d7f441c")
(setf (get rokubimini_msgs::ResetWrenchRequest :datatype-) "rokubimini_msgs/ResetWrenchRequest")
(setf (get rokubimini_msgs::ResetWrenchRequest :definition-)
      "geometry_msgs/Wrench desired_wrench

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
---
bool success
")

(setf (get rokubimini_msgs::ResetWrenchResponse :md5sum-) "a01e1ece25b40c645838e2e60d7f441c")
(setf (get rokubimini_msgs::ResetWrenchResponse :datatype-) "rokubimini_msgs/ResetWrenchResponse")
(setf (get rokubimini_msgs::ResetWrenchResponse :definition-)
      "geometry_msgs/Wrench desired_wrench

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
---
bool success
")



(provide :rokubimini_msgs/ResetWrench "a01e1ece25b40c645838e2e60d7f441c")


