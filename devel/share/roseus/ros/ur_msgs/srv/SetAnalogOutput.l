;; Auto-generated. Do not edit!


(when (boundp 'ur_msgs::SetAnalogOutput)
  (if (not (find-package "UR_MSGS"))
    (make-package "UR_MSGS"))
  (shadow 'SetAnalogOutput (find-package "UR_MSGS")))
(unless (find-package "UR_MSGS::SETANALOGOUTPUT")
  (make-package "UR_MSGS::SETANALOGOUTPUT"))
(unless (find-package "UR_MSGS::SETANALOGOUTPUTREQUEST")
  (make-package "UR_MSGS::SETANALOGOUTPUTREQUEST"))
(unless (find-package "UR_MSGS::SETANALOGOUTPUTRESPONSE")
  (make-package "UR_MSGS::SETANALOGOUTPUTRESPONSE"))

(in-package "ROS")





(defclass ur_msgs::SetAnalogOutputRequest
  :super ros::object
  :slots (_data ))

(defmethod ur_msgs::SetAnalogOutputRequest
  (:init
   (&key
    ((:data __data) (instance ur_msgs::Analog :init))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; ur_msgs/Analog _data
    (send _data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; ur_msgs/Analog _data
       (send _data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; ur_msgs/Analog _data
     (send _data :deserialize buf ptr-) (incf ptr- (send _data :serialization-length))
   ;;
   self)
  )

(defclass ur_msgs::SetAnalogOutputResponse
  :super ros::object
  :slots (_success ))

(defmethod ur_msgs::SetAnalogOutputResponse
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

(defclass ur_msgs::SetAnalogOutput
  :super ros::object
  :slots ())

(setf (get ur_msgs::SetAnalogOutput :md5sum-) "eb3840e5f3632fc236409b92a9250f5b")
(setf (get ur_msgs::SetAnalogOutput :datatype-) "ur_msgs/SetAnalogOutput")
(setf (get ur_msgs::SetAnalogOutput :request) ur_msgs::SetAnalogOutputRequest)
(setf (get ur_msgs::SetAnalogOutput :response) ur_msgs::SetAnalogOutputResponse)

(defmethod ur_msgs::SetAnalogOutputRequest
  (:response () (instance ur_msgs::SetAnalogOutputResponse :init)))

(setf (get ur_msgs::SetAnalogOutputRequest :md5sum-) "eb3840e5f3632fc236409b92a9250f5b")
(setf (get ur_msgs::SetAnalogOutputRequest :datatype-) "ur_msgs/SetAnalogOutputRequest")
(setf (get ur_msgs::SetAnalogOutputRequest :definition-)
      "Analog data

================================================================================
MSG: ur_msgs/Analog
uint8 CURRENT=0
uint8 VOLTAGE=1

uint8 pin
uint8 domain # can be VOLTAGE or CURRENT
float32 state
---
bool success

")

(setf (get ur_msgs::SetAnalogOutputResponse :md5sum-) "eb3840e5f3632fc236409b92a9250f5b")
(setf (get ur_msgs::SetAnalogOutputResponse :datatype-) "ur_msgs/SetAnalogOutputResponse")
(setf (get ur_msgs::SetAnalogOutputResponse :definition-)
      "Analog data

================================================================================
MSG: ur_msgs/Analog
uint8 CURRENT=0
uint8 VOLTAGE=1

uint8 pin
uint8 domain # can be VOLTAGE or CURRENT
float32 state
---
bool success

")



(provide :ur_msgs/SetAnalogOutput "eb3840e5f3632fc236409b92a9250f5b")


