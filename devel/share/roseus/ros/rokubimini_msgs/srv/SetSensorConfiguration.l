;; Auto-generated. Do not edit!


(when (boundp 'rokubimini_msgs::SetSensorConfiguration)
  (if (not (find-package "ROKUBIMINI_MSGS"))
    (make-package "ROKUBIMINI_MSGS"))
  (shadow 'SetSensorConfiguration (find-package "ROKUBIMINI_MSGS")))
(unless (find-package "ROKUBIMINI_MSGS::SETSENSORCONFIGURATION")
  (make-package "ROKUBIMINI_MSGS::SETSENSORCONFIGURATION"))
(unless (find-package "ROKUBIMINI_MSGS::SETSENSORCONFIGURATIONREQUEST")
  (make-package "ROKUBIMINI_MSGS::SETSENSORCONFIGURATIONREQUEST"))
(unless (find-package "ROKUBIMINI_MSGS::SETSENSORCONFIGURATIONRESPONSE")
  (make-package "ROKUBIMINI_MSGS::SETSENSORCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass rokubimini_msgs::SetSensorConfigurationRequest
  :super ros::object
  :slots (_a ))

(defmethod rokubimini_msgs::SetSensorConfigurationRequest
  (:init
   (&key
    ((:a __a) nil)
    )
   (send-super :init)
   (setq _a __a)
   self)
  (:a
   (&optional (__a :null))
   (if (not (eq __a :null)) (setq _a __a)) _a)
  (:serialization-length
   ()
   (+
    ;; bool _a
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _a
       (if _a (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _a
     (setq _a (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass rokubimini_msgs::SetSensorConfigurationResponse
  :super ros::object
  :slots (_b ))

(defmethod rokubimini_msgs::SetSensorConfigurationResponse
  (:init
   (&key
    ((:b __b) nil)
    )
   (send-super :init)
   (setq _b __b)
   self)
  (:b
   (&optional (__b :null))
   (if (not (eq __b :null)) (setq _b __b)) _b)
  (:serialization-length
   ()
   (+
    ;; bool _b
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _b
       (if _b (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _b
     (setq _b (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass rokubimini_msgs::SetSensorConfiguration
  :super ros::object
  :slots ())

(setf (get rokubimini_msgs::SetSensorConfiguration :md5sum-) "81f01bfd9a951b1adf9102125874ff5b")
(setf (get rokubimini_msgs::SetSensorConfiguration :datatype-) "rokubimini_msgs/SetSensorConfiguration")
(setf (get rokubimini_msgs::SetSensorConfiguration :request) rokubimini_msgs::SetSensorConfigurationRequest)
(setf (get rokubimini_msgs::SetSensorConfiguration :response) rokubimini_msgs::SetSensorConfigurationResponse)

(defmethod rokubimini_msgs::SetSensorConfigurationRequest
  (:response () (instance rokubimini_msgs::SetSensorConfigurationResponse :init)))

(setf (get rokubimini_msgs::SetSensorConfigurationRequest :md5sum-) "81f01bfd9a951b1adf9102125874ff5b")
(setf (get rokubimini_msgs::SetSensorConfigurationRequest :datatype-) "rokubimini_msgs/SetSensorConfigurationRequest")
(setf (get rokubimini_msgs::SetSensorConfigurationRequest :definition-)
      "bool a

---

bool b
")

(setf (get rokubimini_msgs::SetSensorConfigurationResponse :md5sum-) "81f01bfd9a951b1adf9102125874ff5b")
(setf (get rokubimini_msgs::SetSensorConfigurationResponse :datatype-) "rokubimini_msgs/SetSensorConfigurationResponse")
(setf (get rokubimini_msgs::SetSensorConfigurationResponse :definition-)
      "bool a

---

bool b
")



(provide :rokubimini_msgs/SetSensorConfiguration "81f01bfd9a951b1adf9102125874ff5b")


