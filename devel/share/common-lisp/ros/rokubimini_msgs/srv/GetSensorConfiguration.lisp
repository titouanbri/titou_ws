; Auto-generated. Do not edit!


(cl:in-package rokubimini_msgs-srv)


;//! \htmlinclude GetSensorConfiguration-request.msg.html

(cl:defclass <GetSensorConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetSensorConfiguration-request (<GetSensorConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSensorConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSensorConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokubimini_msgs-srv:<GetSensorConfiguration-request> is deprecated: use rokubimini_msgs-srv:GetSensorConfiguration-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <GetSensorConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokubimini_msgs-srv:a-val is deprecated.  Use rokubimini_msgs-srv:a instead.")
  (a m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSensorConfiguration-request>) ostream)
  "Serializes a message object of type '<GetSensorConfiguration-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'a) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSensorConfiguration-request>) istream)
  "Deserializes a message object of type '<GetSensorConfiguration-request>"
    (cl:setf (cl:slot-value msg 'a) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSensorConfiguration-request>)))
  "Returns string type for a service object of type '<GetSensorConfiguration-request>"
  "rokubimini_msgs/GetSensorConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSensorConfiguration-request)))
  "Returns string type for a service object of type 'GetSensorConfiguration-request"
  "rokubimini_msgs/GetSensorConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSensorConfiguration-request>)))
  "Returns md5sum for a message object of type '<GetSensorConfiguration-request>"
  "81f01bfd9a951b1adf9102125874ff5b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSensorConfiguration-request)))
  "Returns md5sum for a message object of type 'GetSensorConfiguration-request"
  "81f01bfd9a951b1adf9102125874ff5b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSensorConfiguration-request>)))
  "Returns full string definition for message of type '<GetSensorConfiguration-request>"
  (cl:format cl:nil "bool a~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSensorConfiguration-request)))
  "Returns full string definition for message of type 'GetSensorConfiguration-request"
  (cl:format cl:nil "bool a~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSensorConfiguration-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSensorConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSensorConfiguration-request
    (cl:cons ':a (a msg))
))
;//! \htmlinclude GetSensorConfiguration-response.msg.html

(cl:defclass <GetSensorConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((b
    :reader b
    :initarg :b
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetSensorConfiguration-response (<GetSensorConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSensorConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSensorConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokubimini_msgs-srv:<GetSensorConfiguration-response> is deprecated: use rokubimini_msgs-srv:GetSensorConfiguration-response instead.")))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <GetSensorConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokubimini_msgs-srv:b-val is deprecated.  Use rokubimini_msgs-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSensorConfiguration-response>) ostream)
  "Serializes a message object of type '<GetSensorConfiguration-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'b) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSensorConfiguration-response>) istream)
  "Deserializes a message object of type '<GetSensorConfiguration-response>"
    (cl:setf (cl:slot-value msg 'b) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSensorConfiguration-response>)))
  "Returns string type for a service object of type '<GetSensorConfiguration-response>"
  "rokubimini_msgs/GetSensorConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSensorConfiguration-response)))
  "Returns string type for a service object of type 'GetSensorConfiguration-response"
  "rokubimini_msgs/GetSensorConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSensorConfiguration-response>)))
  "Returns md5sum for a message object of type '<GetSensorConfiguration-response>"
  "81f01bfd9a951b1adf9102125874ff5b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSensorConfiguration-response)))
  "Returns md5sum for a message object of type 'GetSensorConfiguration-response"
  "81f01bfd9a951b1adf9102125874ff5b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSensorConfiguration-response>)))
  "Returns full string definition for message of type '<GetSensorConfiguration-response>"
  (cl:format cl:nil "~%bool b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSensorConfiguration-response)))
  "Returns full string definition for message of type 'GetSensorConfiguration-response"
  (cl:format cl:nil "~%bool b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSensorConfiguration-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSensorConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSensorConfiguration-response
    (cl:cons ':b (b msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetSensorConfiguration)))
  'GetSensorConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetSensorConfiguration)))
  'GetSensorConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSensorConfiguration)))
  "Returns string type for a service object of type '<GetSensorConfiguration>"
  "rokubimini_msgs/GetSensorConfiguration")