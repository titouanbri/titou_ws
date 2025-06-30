; Auto-generated. Do not edit!


(cl:in-package ur_msgs-srv)


;//! \htmlinclude SetAnalogOutput-request.msg.html

(cl:defclass <SetAnalogOutput-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type ur_msgs-msg:Analog
    :initform (cl:make-instance 'ur_msgs-msg:Analog)))
)

(cl:defclass SetAnalogOutput-request (<SetAnalogOutput-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAnalogOutput-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAnalogOutput-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur_msgs-srv:<SetAnalogOutput-request> is deprecated: use ur_msgs-srv:SetAnalogOutput-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <SetAnalogOutput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:data-val is deprecated.  Use ur_msgs-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAnalogOutput-request>) ostream)
  "Serializes a message object of type '<SetAnalogOutput-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAnalogOutput-request>) istream)
  "Deserializes a message object of type '<SetAnalogOutput-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAnalogOutput-request>)))
  "Returns string type for a service object of type '<SetAnalogOutput-request>"
  "ur_msgs/SetAnalogOutputRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAnalogOutput-request)))
  "Returns string type for a service object of type 'SetAnalogOutput-request"
  "ur_msgs/SetAnalogOutputRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAnalogOutput-request>)))
  "Returns md5sum for a message object of type '<SetAnalogOutput-request>"
  "eb3840e5f3632fc236409b92a9250f5b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAnalogOutput-request)))
  "Returns md5sum for a message object of type 'SetAnalogOutput-request"
  "eb3840e5f3632fc236409b92a9250f5b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAnalogOutput-request>)))
  "Returns full string definition for message of type '<SetAnalogOutput-request>"
  (cl:format cl:nil "Analog data~%~%================================================================================~%MSG: ur_msgs/Analog~%uint8 CURRENT=0~%uint8 VOLTAGE=1~%~%uint8 pin~%uint8 domain # can be VOLTAGE or CURRENT~%float32 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAnalogOutput-request)))
  "Returns full string definition for message of type 'SetAnalogOutput-request"
  (cl:format cl:nil "Analog data~%~%================================================================================~%MSG: ur_msgs/Analog~%uint8 CURRENT=0~%uint8 VOLTAGE=1~%~%uint8 pin~%uint8 domain # can be VOLTAGE or CURRENT~%float32 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAnalogOutput-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAnalogOutput-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAnalogOutput-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude SetAnalogOutput-response.msg.html

(cl:defclass <SetAnalogOutput-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetAnalogOutput-response (<SetAnalogOutput-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAnalogOutput-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAnalogOutput-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur_msgs-srv:<SetAnalogOutput-response> is deprecated: use ur_msgs-srv:SetAnalogOutput-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetAnalogOutput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:success-val is deprecated.  Use ur_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAnalogOutput-response>) ostream)
  "Serializes a message object of type '<SetAnalogOutput-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAnalogOutput-response>) istream)
  "Deserializes a message object of type '<SetAnalogOutput-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAnalogOutput-response>)))
  "Returns string type for a service object of type '<SetAnalogOutput-response>"
  "ur_msgs/SetAnalogOutputResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAnalogOutput-response)))
  "Returns string type for a service object of type 'SetAnalogOutput-response"
  "ur_msgs/SetAnalogOutputResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAnalogOutput-response>)))
  "Returns md5sum for a message object of type '<SetAnalogOutput-response>"
  "eb3840e5f3632fc236409b92a9250f5b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAnalogOutput-response)))
  "Returns md5sum for a message object of type 'SetAnalogOutput-response"
  "eb3840e5f3632fc236409b92a9250f5b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAnalogOutput-response>)))
  "Returns full string definition for message of type '<SetAnalogOutput-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAnalogOutput-response)))
  "Returns full string definition for message of type 'SetAnalogOutput-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAnalogOutput-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAnalogOutput-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAnalogOutput-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetAnalogOutput)))
  'SetAnalogOutput-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetAnalogOutput)))
  'SetAnalogOutput-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAnalogOutput)))
  "Returns string type for a service object of type '<SetAnalogOutput>"
  "ur_msgs/SetAnalogOutput")