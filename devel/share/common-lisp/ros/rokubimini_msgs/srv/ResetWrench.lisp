; Auto-generated. Do not edit!


(cl:in-package rokubimini_msgs-srv)


;//! \htmlinclude ResetWrench-request.msg.html

(cl:defclass <ResetWrench-request> (roslisp-msg-protocol:ros-message)
  ((desired_wrench
    :reader desired_wrench
    :initarg :desired_wrench
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench)))
)

(cl:defclass ResetWrench-request (<ResetWrench-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetWrench-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetWrench-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokubimini_msgs-srv:<ResetWrench-request> is deprecated: use rokubimini_msgs-srv:ResetWrench-request instead.")))

(cl:ensure-generic-function 'desired_wrench-val :lambda-list '(m))
(cl:defmethod desired_wrench-val ((m <ResetWrench-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokubimini_msgs-srv:desired_wrench-val is deprecated.  Use rokubimini_msgs-srv:desired_wrench instead.")
  (desired_wrench m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetWrench-request>) ostream)
  "Serializes a message object of type '<ResetWrench-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'desired_wrench) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetWrench-request>) istream)
  "Deserializes a message object of type '<ResetWrench-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'desired_wrench) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetWrench-request>)))
  "Returns string type for a service object of type '<ResetWrench-request>"
  "rokubimini_msgs/ResetWrenchRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetWrench-request)))
  "Returns string type for a service object of type 'ResetWrench-request"
  "rokubimini_msgs/ResetWrenchRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetWrench-request>)))
  "Returns md5sum for a message object of type '<ResetWrench-request>"
  "a01e1ece25b40c645838e2e60d7f441c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetWrench-request)))
  "Returns md5sum for a message object of type 'ResetWrench-request"
  "a01e1ece25b40c645838e2e60d7f441c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetWrench-request>)))
  "Returns full string definition for message of type '<ResetWrench-request>"
  (cl:format cl:nil "geometry_msgs/Wrench desired_wrench~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetWrench-request)))
  "Returns full string definition for message of type 'ResetWrench-request"
  (cl:format cl:nil "geometry_msgs/Wrench desired_wrench~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetWrench-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'desired_wrench))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetWrench-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetWrench-request
    (cl:cons ':desired_wrench (desired_wrench msg))
))
;//! \htmlinclude ResetWrench-response.msg.html

(cl:defclass <ResetWrench-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ResetWrench-response (<ResetWrench-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetWrench-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetWrench-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokubimini_msgs-srv:<ResetWrench-response> is deprecated: use rokubimini_msgs-srv:ResetWrench-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ResetWrench-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokubimini_msgs-srv:success-val is deprecated.  Use rokubimini_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetWrench-response>) ostream)
  "Serializes a message object of type '<ResetWrench-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetWrench-response>) istream)
  "Deserializes a message object of type '<ResetWrench-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetWrench-response>)))
  "Returns string type for a service object of type '<ResetWrench-response>"
  "rokubimini_msgs/ResetWrenchResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetWrench-response)))
  "Returns string type for a service object of type 'ResetWrench-response"
  "rokubimini_msgs/ResetWrenchResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetWrench-response>)))
  "Returns md5sum for a message object of type '<ResetWrench-response>"
  "a01e1ece25b40c645838e2e60d7f441c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetWrench-response)))
  "Returns md5sum for a message object of type 'ResetWrench-response"
  "a01e1ece25b40c645838e2e60d7f441c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetWrench-response>)))
  "Returns full string definition for message of type '<ResetWrench-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetWrench-response)))
  "Returns full string definition for message of type 'ResetWrench-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetWrench-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetWrench-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetWrench-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetWrench)))
  'ResetWrench-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetWrench)))
  'ResetWrench-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetWrench)))
  "Returns string type for a service object of type '<ResetWrench>"
  "rokubimini_msgs/ResetWrench")