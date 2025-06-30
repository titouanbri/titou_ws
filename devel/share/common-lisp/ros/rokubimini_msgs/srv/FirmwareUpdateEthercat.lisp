; Auto-generated. Do not edit!


(cl:in-package rokubimini_msgs-srv)


;//! \htmlinclude FirmwareUpdateEthercat-request.msg.html

(cl:defclass <FirmwareUpdateEthercat-request> (roslisp-msg-protocol:ros-message)
  ((file_name
    :reader file_name
    :initarg :file_name
    :type cl:string
    :initform "")
   (file_path
    :reader file_path
    :initarg :file_path
    :type cl:string
    :initform "")
   (password
    :reader password
    :initarg :password
    :type cl:integer
    :initform 0))
)

(cl:defclass FirmwareUpdateEthercat-request (<FirmwareUpdateEthercat-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FirmwareUpdateEthercat-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FirmwareUpdateEthercat-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokubimini_msgs-srv:<FirmwareUpdateEthercat-request> is deprecated: use rokubimini_msgs-srv:FirmwareUpdateEthercat-request instead.")))

(cl:ensure-generic-function 'file_name-val :lambda-list '(m))
(cl:defmethod file_name-val ((m <FirmwareUpdateEthercat-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokubimini_msgs-srv:file_name-val is deprecated.  Use rokubimini_msgs-srv:file_name instead.")
  (file_name m))

(cl:ensure-generic-function 'file_path-val :lambda-list '(m))
(cl:defmethod file_path-val ((m <FirmwareUpdateEthercat-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokubimini_msgs-srv:file_path-val is deprecated.  Use rokubimini_msgs-srv:file_path instead.")
  (file_path m))

(cl:ensure-generic-function 'password-val :lambda-list '(m))
(cl:defmethod password-val ((m <FirmwareUpdateEthercat-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokubimini_msgs-srv:password-val is deprecated.  Use rokubimini_msgs-srv:password instead.")
  (password m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FirmwareUpdateEthercat-request>) ostream)
  "Serializes a message object of type '<FirmwareUpdateEthercat-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'file_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'file_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'file_path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'file_path))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'password)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'password)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'password)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'password)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FirmwareUpdateEthercat-request>) istream)
  "Deserializes a message object of type '<FirmwareUpdateEthercat-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'file_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'file_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'file_path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'file_path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'password)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'password)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'password)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'password)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FirmwareUpdateEthercat-request>)))
  "Returns string type for a service object of type '<FirmwareUpdateEthercat-request>"
  "rokubimini_msgs/FirmwareUpdateEthercatRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FirmwareUpdateEthercat-request)))
  "Returns string type for a service object of type 'FirmwareUpdateEthercat-request"
  "rokubimini_msgs/FirmwareUpdateEthercatRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FirmwareUpdateEthercat-request>)))
  "Returns md5sum for a message object of type '<FirmwareUpdateEthercat-request>"
  "3e589c8a3c17e65c13355340a4ec94ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FirmwareUpdateEthercat-request)))
  "Returns md5sum for a message object of type 'FirmwareUpdateEthercat-request"
  "3e589c8a3c17e65c13355340a4ec94ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FirmwareUpdateEthercat-request>)))
  "Returns full string definition for message of type '<FirmwareUpdateEthercat-request>"
  (cl:format cl:nil "string file_name~%string file_path~%uint32 password~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FirmwareUpdateEthercat-request)))
  "Returns full string definition for message of type 'FirmwareUpdateEthercat-request"
  (cl:format cl:nil "string file_name~%string file_path~%uint32 password~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FirmwareUpdateEthercat-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'file_name))
     4 (cl:length (cl:slot-value msg 'file_path))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FirmwareUpdateEthercat-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FirmwareUpdateEthercat-request
    (cl:cons ':file_name (file_name msg))
    (cl:cons ':file_path (file_path msg))
    (cl:cons ':password (password msg))
))
;//! \htmlinclude FirmwareUpdateEthercat-response.msg.html

(cl:defclass <FirmwareUpdateEthercat-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FirmwareUpdateEthercat-response (<FirmwareUpdateEthercat-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FirmwareUpdateEthercat-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FirmwareUpdateEthercat-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokubimini_msgs-srv:<FirmwareUpdateEthercat-response> is deprecated: use rokubimini_msgs-srv:FirmwareUpdateEthercat-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <FirmwareUpdateEthercat-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokubimini_msgs-srv:result-val is deprecated.  Use rokubimini_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FirmwareUpdateEthercat-response>) ostream)
  "Serializes a message object of type '<FirmwareUpdateEthercat-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FirmwareUpdateEthercat-response>) istream)
  "Deserializes a message object of type '<FirmwareUpdateEthercat-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FirmwareUpdateEthercat-response>)))
  "Returns string type for a service object of type '<FirmwareUpdateEthercat-response>"
  "rokubimini_msgs/FirmwareUpdateEthercatResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FirmwareUpdateEthercat-response)))
  "Returns string type for a service object of type 'FirmwareUpdateEthercat-response"
  "rokubimini_msgs/FirmwareUpdateEthercatResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FirmwareUpdateEthercat-response>)))
  "Returns md5sum for a message object of type '<FirmwareUpdateEthercat-response>"
  "3e589c8a3c17e65c13355340a4ec94ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FirmwareUpdateEthercat-response)))
  "Returns md5sum for a message object of type 'FirmwareUpdateEthercat-response"
  "3e589c8a3c17e65c13355340a4ec94ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FirmwareUpdateEthercat-response>)))
  "Returns full string definition for message of type '<FirmwareUpdateEthercat-response>"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FirmwareUpdateEthercat-response)))
  "Returns full string definition for message of type 'FirmwareUpdateEthercat-response"
  (cl:format cl:nil "bool result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FirmwareUpdateEthercat-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FirmwareUpdateEthercat-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FirmwareUpdateEthercat-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FirmwareUpdateEthercat)))
  'FirmwareUpdateEthercat-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FirmwareUpdateEthercat)))
  'FirmwareUpdateEthercat-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FirmwareUpdateEthercat)))
  "Returns string type for a service object of type '<FirmwareUpdateEthercat>"
  "rokubimini_msgs/FirmwareUpdateEthercat")