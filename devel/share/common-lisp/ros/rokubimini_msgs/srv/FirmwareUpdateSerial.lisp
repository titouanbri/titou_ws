; Auto-generated. Do not edit!


(cl:in-package rokubimini_msgs-srv)


;//! \htmlinclude FirmwareUpdateSerial-request.msg.html

(cl:defclass <FirmwareUpdateSerial-request> (roslisp-msg-protocol:ros-message)
  ((file_path
    :reader file_path
    :initarg :file_path
    :type cl:string
    :initform ""))
)

(cl:defclass FirmwareUpdateSerial-request (<FirmwareUpdateSerial-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FirmwareUpdateSerial-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FirmwareUpdateSerial-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokubimini_msgs-srv:<FirmwareUpdateSerial-request> is deprecated: use rokubimini_msgs-srv:FirmwareUpdateSerial-request instead.")))

(cl:ensure-generic-function 'file_path-val :lambda-list '(m))
(cl:defmethod file_path-val ((m <FirmwareUpdateSerial-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokubimini_msgs-srv:file_path-val is deprecated.  Use rokubimini_msgs-srv:file_path instead.")
  (file_path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FirmwareUpdateSerial-request>) ostream)
  "Serializes a message object of type '<FirmwareUpdateSerial-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'file_path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'file_path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FirmwareUpdateSerial-request>) istream)
  "Deserializes a message object of type '<FirmwareUpdateSerial-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'file_path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'file_path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FirmwareUpdateSerial-request>)))
  "Returns string type for a service object of type '<FirmwareUpdateSerial-request>"
  "rokubimini_msgs/FirmwareUpdateSerialRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FirmwareUpdateSerial-request)))
  "Returns string type for a service object of type 'FirmwareUpdateSerial-request"
  "rokubimini_msgs/FirmwareUpdateSerialRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FirmwareUpdateSerial-request>)))
  "Returns md5sum for a message object of type '<FirmwareUpdateSerial-request>"
  "b223f245a1d13c9e179ae29717131752")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FirmwareUpdateSerial-request)))
  "Returns md5sum for a message object of type 'FirmwareUpdateSerial-request"
  "b223f245a1d13c9e179ae29717131752")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FirmwareUpdateSerial-request>)))
  "Returns full string definition for message of type '<FirmwareUpdateSerial-request>"
  (cl:format cl:nil "string file_path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FirmwareUpdateSerial-request)))
  "Returns full string definition for message of type 'FirmwareUpdateSerial-request"
  (cl:format cl:nil "string file_path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FirmwareUpdateSerial-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'file_path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FirmwareUpdateSerial-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FirmwareUpdateSerial-request
    (cl:cons ':file_path (file_path msg))
))
;//! \htmlinclude FirmwareUpdateSerial-response.msg.html

(cl:defclass <FirmwareUpdateSerial-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FirmwareUpdateSerial-response (<FirmwareUpdateSerial-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FirmwareUpdateSerial-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FirmwareUpdateSerial-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokubimini_msgs-srv:<FirmwareUpdateSerial-response> is deprecated: use rokubimini_msgs-srv:FirmwareUpdateSerial-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <FirmwareUpdateSerial-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokubimini_msgs-srv:result-val is deprecated.  Use rokubimini_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FirmwareUpdateSerial-response>) ostream)
  "Serializes a message object of type '<FirmwareUpdateSerial-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FirmwareUpdateSerial-response>) istream)
  "Deserializes a message object of type '<FirmwareUpdateSerial-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FirmwareUpdateSerial-response>)))
  "Returns string type for a service object of type '<FirmwareUpdateSerial-response>"
  "rokubimini_msgs/FirmwareUpdateSerialResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FirmwareUpdateSerial-response)))
  "Returns string type for a service object of type 'FirmwareUpdateSerial-response"
  "rokubimini_msgs/FirmwareUpdateSerialResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FirmwareUpdateSerial-response>)))
  "Returns md5sum for a message object of type '<FirmwareUpdateSerial-response>"
  "b223f245a1d13c9e179ae29717131752")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FirmwareUpdateSerial-response)))
  "Returns md5sum for a message object of type 'FirmwareUpdateSerial-response"
  "b223f245a1d13c9e179ae29717131752")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FirmwareUpdateSerial-response>)))
  "Returns full string definition for message of type '<FirmwareUpdateSerial-response>"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FirmwareUpdateSerial-response)))
  "Returns full string definition for message of type 'FirmwareUpdateSerial-response"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FirmwareUpdateSerial-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FirmwareUpdateSerial-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FirmwareUpdateSerial-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FirmwareUpdateSerial)))
  'FirmwareUpdateSerial-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FirmwareUpdateSerial)))
  'FirmwareUpdateSerial-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FirmwareUpdateSerial)))
  "Returns string type for a service object of type '<FirmwareUpdateSerial>"
  "rokubimini_msgs/FirmwareUpdateSerial")