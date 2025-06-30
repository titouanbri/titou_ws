;; Auto-generated. Do not edit!


(when (boundp 'rokubimini_msgs::GetSensorConfiguration)
  (if (not (find-package "ROKUBIMINI_MSGS"))
    (make-package "ROKUBIMINI_MSGS"))
  (shadow 'GetSensorConfiguration (find-package "ROKUBIMINI_MSGS")))
(unless (find-package "ROKUBIMINI_MSGS::GETSENSORCONFIGURATION")
  (make-package "ROKUBIMINI_MSGS::GETSENSORCONFIGURATION"))
(unless (find-package "ROKUBIMINI_MSGS::GETSENSORCONFIGURATIONREQUEST")
  (make-package "ROKUBIMINI_MSGS::GETSENSORCONFIGURATIONREQUEST"))
(unless (find-package "ROKUBIMINI_MSGS::GETSENSORCONFIGURATIONRESPONSE")
  (make-package "ROKUBIMINI_MSGS::GETSENSORCONFIGURATIONRESPONSE"))

(in-package "ROS")





(defclass rokubimini_msgs::GetSensorConfigurationRequest
  :super ros::object
  :slots (_a ))

(defmethod rokubimini_msgs::GetSensorConfigurationRequest
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

(defclass rokubimini_msgs::GetSensorConfigurationResponse
  :super ros::object
  :slots (_b ))

(defmethod rokubimini_msgs::GetSensorConfigurationResponse
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

(defclass rokubimini_msgs::GetSensorConfiguration
  :super ros::object
  :slots ())

(setf (get rokubimini_msgs::GetSensorConfiguration :md5sum-) "81f01bfd9a951b1adf9102125874ff5b")
(setf (get rokubimini_msgs::GetSensorConfiguration :datatype-) "rokubimini_msgs/GetSensorConfiguration")
(setf (get rokubimini_msgs::GetSensorConfiguration :request) rokubimini_msgs::GetSensorConfigurationRequest)
(setf (get rokubimini_msgs::GetSensorConfiguration :response) rokubimini_msgs::GetSensorConfigurationResponse)

(defmethod rokubimini_msgs::GetSensorConfigurationRequest
  (:response () (instance rokubimini_msgs::GetSensorConfigurationResponse :init)))

(setf (get rokubimini_msgs::GetSensorConfigurationRequest :md5sum-) "81f01bfd9a951b1adf9102125874ff5b")
(setf (get rokubimini_msgs::GetSensorConfigurationRequest :datatype-) "rokubimini_msgs/GetSensorConfigurationRequest")
(setf (get rokubimini_msgs::GetSensorConfigurationRequest :definition-)
      "bool a

---

bool b
")

(setf (get rokubimini_msgs::GetSensorConfigurationResponse :md5sum-) "81f01bfd9a951b1adf9102125874ff5b")
(setf (get rokubimini_msgs::GetSensorConfigurationResponse :datatype-) "rokubimini_msgs/GetSensorConfigurationResponse")
(setf (get rokubimini_msgs::GetSensorConfigurationResponse :definition-)
      "bool a

---

bool b
")



(provide :rokubimini_msgs/GetSensorConfiguration "81f01bfd9a951b1adf9102125874ff5b")


