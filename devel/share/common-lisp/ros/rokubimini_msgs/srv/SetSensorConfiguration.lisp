; Auto-generated. Do not edit!


(cl:in-package rokubimini_msgs-srv)


;//! \htmlinclude SetSensorConfiguration-request.msg.html

(cl:defclass <SetSensorConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetSensorConfiguration-request (<SetSensorConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSensorConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSensorConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokubimini_msgs-srv:<SetSensorConfiguration-request> is deprecated: use rokubimini_msgs-srv:SetSensorConfiguration-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <SetSensorConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokubimini_msgs-srv:a-val is deprecated.  Use rokubimini_msgs-srv:a instead.")
  (a m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSensorConfiguration-request>) ostream)
  "Serializes a message object of type '<SetSensorConfiguration-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'a) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSensorConfiguration-request>) istream)
  "Deserializes a message object of type '<SetSensorConfiguration-request>"
    (cl:setf (cl:slot-value msg 'a) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSensorConfiguration-request>)))
  "Returns string type for a service object of type '<SetSensorConfiguration-request>"
  "rokubimini_msgs/SetSensorConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSensorConfiguration-request)))
  "Returns string type for a service object of type 'SetSensorConfiguration-request"
  "rokubimini_msgs/SetSensorConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSensorConfiguration-request>)))
  "Returns md5sum for a message object of type '<SetSensorConfiguration-request>"
  "81f01bfd9a951b1adf9102125874ff5b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSensorConfiguration-request)))
  "Returns md5sum for a message object of type 'SetSensorConfiguration-request"
  "81f01bfd9a951b1adf9102125874ff5b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSensorConfiguration-request>)))
  "Returns full string definition for message of type '<SetSensorConfiguration-request>"
  (cl:format cl:nil "bool a~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSensorConfiguration-request)))
  "Returns full string definition for message of type 'SetSensorConfiguration-request"
  (cl:format cl:nil "bool a~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSensorConfiguration-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSensorConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSensorConfiguration-request
    (cl:cons ':a (a msg))
))
;//! \htmlinclude SetSensorConfiguration-response.msg.html

(cl:defclass <SetSensorConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((b
    :reader b
    :initarg :b
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetSensorConfiguration-response (<SetSensorConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSensorConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSensorConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokubimini_msgs-srv:<SetSensorConfiguration-response> is deprecated: use rokubimini_msgs-srv:SetSensorConfiguration-response instead.")))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <SetSensorConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokubimini_msgs-srv:b-val is deprecated.  Use rokubimini_msgs-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSensorConfiguration-response>) ostream)
  "Serializes a message object of type '<SetSensorConfiguration-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'b) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSensorConfiguration-response>) istream)
  "Deserializes a message object of type '<SetSensorConfiguration-response>"
    (cl:setf (cl:slot-value msg 'b) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSensorConfiguration-response>)))
  "Returns string type for a service object of type '<SetSensorConfiguration-response>"
  "rokubimini_msgs/SetSensorConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSensorConfiguration-response)))
  "Returns string type for a service object of type 'SetSensorConfiguration-response"
  "rokubimini_msgs/SetSensorConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSensorConfiguration-response>)))
  "Returns md5sum for a message object of type '<SetSensorConfiguration-response>"
  "81f01bfd9a951b1adf9102125874ff5b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSensorConfiguration-response)))
  "Returns md5sum for a message object of type 'SetSensorConfiguration-response"
  "81f01bfd9a951b1adf9102125874ff5b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSensorConfiguration-response>)))
  "Returns full string definition for message of type '<SetSensorConfiguration-response>"
  (cl:format cl:nil "~%bool b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSensorConfiguration-response)))
  "Returns full string definition for message of type 'SetSensorConfiguration-response"
  (cl:format cl:nil "~%bool b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSensorConfiguration-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSensorConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSensorConfiguration-response
    (cl:cons ':b (b msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetSensorConfiguration)))
  'SetSensorConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetSensorConfiguration)))
  'SetSensorConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSensorConfiguration)))
  "Returns string type for a service object of type '<SetSensorConfiguration>"
  "rokubimini_msgs/SetSensorConfiguration")