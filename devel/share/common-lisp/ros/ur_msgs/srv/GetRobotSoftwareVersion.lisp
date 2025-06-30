; Auto-generated. Do not edit!


(cl:in-package ur_msgs-srv)


;//! \htmlinclude GetRobotSoftwareVersion-request.msg.html

(cl:defclass <GetRobotSoftwareVersion-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetRobotSoftwareVersion-request (<GetRobotSoftwareVersion-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRobotSoftwareVersion-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRobotSoftwareVersion-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur_msgs-srv:<GetRobotSoftwareVersion-request> is deprecated: use ur_msgs-srv:GetRobotSoftwareVersion-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRobotSoftwareVersion-request>) ostream)
  "Serializes a message object of type '<GetRobotSoftwareVersion-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRobotSoftwareVersion-request>) istream)
  "Deserializes a message object of type '<GetRobotSoftwareVersion-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRobotSoftwareVersion-request>)))
  "Returns string type for a service object of type '<GetRobotSoftwareVersion-request>"
  "ur_msgs/GetRobotSoftwareVersionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotSoftwareVersion-request)))
  "Returns string type for a service object of type 'GetRobotSoftwareVersion-request"
  "ur_msgs/GetRobotSoftwareVersionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRobotSoftwareVersion-request>)))
  "Returns md5sum for a message object of type '<GetRobotSoftwareVersion-request>"
  "eea0f1664f7955042558cb2bf2c766ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRobotSoftwareVersion-request)))
  "Returns md5sum for a message object of type 'GetRobotSoftwareVersion-request"
  "eea0f1664f7955042558cb2bf2c766ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRobotSoftwareVersion-request>)))
  "Returns full string definition for message of type '<GetRobotSoftwareVersion-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRobotSoftwareVersion-request)))
  "Returns full string definition for message of type 'GetRobotSoftwareVersion-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRobotSoftwareVersion-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRobotSoftwareVersion-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRobotSoftwareVersion-request
))
;//! \htmlinclude GetRobotSoftwareVersion-response.msg.html

(cl:defclass <GetRobotSoftwareVersion-response> (roslisp-msg-protocol:ros-message)
  ((major
    :reader major
    :initarg :major
    :type cl:integer
    :initform 0)
   (minor
    :reader minor
    :initarg :minor
    :type cl:integer
    :initform 0)
   (bugfix
    :reader bugfix
    :initarg :bugfix
    :type cl:integer
    :initform 0)
   (build
    :reader build
    :initarg :build
    :type cl:integer
    :initform 0))
)

(cl:defclass GetRobotSoftwareVersion-response (<GetRobotSoftwareVersion-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRobotSoftwareVersion-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRobotSoftwareVersion-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur_msgs-srv:<GetRobotSoftwareVersion-response> is deprecated: use ur_msgs-srv:GetRobotSoftwareVersion-response instead.")))

(cl:ensure-generic-function 'major-val :lambda-list '(m))
(cl:defmethod major-val ((m <GetRobotSoftwareVersion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:major-val is deprecated.  Use ur_msgs-srv:major instead.")
  (major m))

(cl:ensure-generic-function 'minor-val :lambda-list '(m))
(cl:defmethod minor-val ((m <GetRobotSoftwareVersion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:minor-val is deprecated.  Use ur_msgs-srv:minor instead.")
  (minor m))

(cl:ensure-generic-function 'bugfix-val :lambda-list '(m))
(cl:defmethod bugfix-val ((m <GetRobotSoftwareVersion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:bugfix-val is deprecated.  Use ur_msgs-srv:bugfix instead.")
  (bugfix m))

(cl:ensure-generic-function 'build-val :lambda-list '(m))
(cl:defmethod build-val ((m <GetRobotSoftwareVersion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_msgs-srv:build-val is deprecated.  Use ur_msgs-srv:build instead.")
  (build m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRobotSoftwareVersion-response>) ostream)
  "Serializes a message object of type '<GetRobotSoftwareVersion-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'major)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'major)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'major)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'major)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'minor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'minor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'minor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'minor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bugfix)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bugfix)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'bugfix)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'bugfix)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'build)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'build)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'build)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'build)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRobotSoftwareVersion-response>) istream)
  "Deserializes a message object of type '<GetRobotSoftwareVersion-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'major)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'major)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'major)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'major)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'minor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'minor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'minor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'minor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bugfix)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bugfix)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'bugfix)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'bugfix)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'build)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'build)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'build)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'build)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRobotSoftwareVersion-response>)))
  "Returns string type for a service object of type '<GetRobotSoftwareVersion-response>"
  "ur_msgs/GetRobotSoftwareVersionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotSoftwareVersion-response)))
  "Returns string type for a service object of type 'GetRobotSoftwareVersion-response"
  "ur_msgs/GetRobotSoftwareVersionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRobotSoftwareVersion-response>)))
  "Returns md5sum for a message object of type '<GetRobotSoftwareVersion-response>"
  "eea0f1664f7955042558cb2bf2c766ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRobotSoftwareVersion-response)))
  "Returns md5sum for a message object of type 'GetRobotSoftwareVersion-response"
  "eea0f1664f7955042558cb2bf2c766ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRobotSoftwareVersion-response>)))
  "Returns full string definition for message of type '<GetRobotSoftwareVersion-response>"
  (cl:format cl:nil "uint32 major    # Major version number~%uint32 minor    # Minor version number~%uint32 bugfix   # Bugfix version number~%uint32 build    # Build number~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRobotSoftwareVersion-response)))
  "Returns full string definition for message of type 'GetRobotSoftwareVersion-response"
  (cl:format cl:nil "uint32 major    # Major version number~%uint32 minor    # Minor version number~%uint32 bugfix   # Bugfix version number~%uint32 build    # Build number~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRobotSoftwareVersion-response>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRobotSoftwareVersion-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRobotSoftwareVersion-response
    (cl:cons ':major (major msg))
    (cl:cons ':minor (minor msg))
    (cl:cons ':bugfix (bugfix msg))
    (cl:cons ':build (build msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetRobotSoftwareVersion)))
  'GetRobotSoftwareVersion-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetRobotSoftwareVersion)))
  'GetRobotSoftwareVersion-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotSoftwareVersion)))
  "Returns string type for a service object of type '<GetRobotSoftwareVersion>"
  "ur_msgs/GetRobotSoftwareVersion")