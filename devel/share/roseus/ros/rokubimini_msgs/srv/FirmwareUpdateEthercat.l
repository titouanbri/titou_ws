;; Auto-generated. Do not edit!


(when (boundp 'rokubimini_msgs::FirmwareUpdateEthercat)
  (if (not (find-package "ROKUBIMINI_MSGS"))
    (make-package "ROKUBIMINI_MSGS"))
  (shadow 'FirmwareUpdateEthercat (find-package "ROKUBIMINI_MSGS")))
(unless (find-package "ROKUBIMINI_MSGS::FIRMWAREUPDATEETHERCAT")
  (make-package "ROKUBIMINI_MSGS::FIRMWAREUPDATEETHERCAT"))
(unless (find-package "ROKUBIMINI_MSGS::FIRMWAREUPDATEETHERCATREQUEST")
  (make-package "ROKUBIMINI_MSGS::FIRMWAREUPDATEETHERCATREQUEST"))
(unless (find-package "ROKUBIMINI_MSGS::FIRMWAREUPDATEETHERCATRESPONSE")
  (make-package "ROKUBIMINI_MSGS::FIRMWAREUPDATEETHERCATRESPONSE"))

(in-package "ROS")





(defclass rokubimini_msgs::FirmwareUpdateEthercatRequest
  :super ros::object
  :slots (_file_name _file_path _password ))

(defmethod rokubimini_msgs::FirmwareUpdateEthercatRequest
  (:init
   (&key
    ((:file_name __file_name) "")
    ((:file_path __file_path) "")
    ((:password __password) 0)
    )
   (send-super :init)
   (setq _file_name (string __file_name))
   (setq _file_path (string __file_path))
   (setq _password (round __password))
   self)
  (:file_name
   (&optional __file_name)
   (if __file_name (setq _file_name __file_name)) _file_name)
  (:file_path
   (&optional __file_path)
   (if __file_path (setq _file_path __file_path)) _file_path)
  (:password
   (&optional __password)
   (if __password (setq _password __password)) _password)
  (:serialization-length
   ()
   (+
    ;; string _file_name
    4 (length _file_name)
    ;; string _file_path
    4 (length _file_path)
    ;; uint32 _password
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _file_name
       (write-long (length _file_name) s) (princ _file_name s)
     ;; string _file_path
       (write-long (length _file_path) s) (princ _file_path s)
     ;; uint32 _password
       (write-long _password s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _file_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _file_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _file_path
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _file_path (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _password
     (setq _password (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass rokubimini_msgs::FirmwareUpdateEthercatResponse
  :super ros::object
  :slots (_result ))

(defmethod rokubimini_msgs::FirmwareUpdateEthercatResponse
  (:init
   (&key
    ((:result __result) nil)
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&optional (__result :null))
   (if (not (eq __result :null)) (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass rokubimini_msgs::FirmwareUpdateEthercat
  :super ros::object
  :slots ())

(setf (get rokubimini_msgs::FirmwareUpdateEthercat :md5sum-) "3e589c8a3c17e65c13355340a4ec94ff")
(setf (get rokubimini_msgs::FirmwareUpdateEthercat :datatype-) "rokubimini_msgs/FirmwareUpdateEthercat")
(setf (get rokubimini_msgs::FirmwareUpdateEthercat :request) rokubimini_msgs::FirmwareUpdateEthercatRequest)
(setf (get rokubimini_msgs::FirmwareUpdateEthercat :response) rokubimini_msgs::FirmwareUpdateEthercatResponse)

(defmethod rokubimini_msgs::FirmwareUpdateEthercatRequest
  (:response () (instance rokubimini_msgs::FirmwareUpdateEthercatResponse :init)))

(setf (get rokubimini_msgs::FirmwareUpdateEthercatRequest :md5sum-) "3e589c8a3c17e65c13355340a4ec94ff")
(setf (get rokubimini_msgs::FirmwareUpdateEthercatRequest :datatype-) "rokubimini_msgs/FirmwareUpdateEthercatRequest")
(setf (get rokubimini_msgs::FirmwareUpdateEthercatRequest :definition-)
      "string file_name
string file_path
uint32 password
---
bool result
")

(setf (get rokubimini_msgs::FirmwareUpdateEthercatResponse :md5sum-) "3e589c8a3c17e65c13355340a4ec94ff")
(setf (get rokubimini_msgs::FirmwareUpdateEthercatResponse :datatype-) "rokubimini_msgs/FirmwareUpdateEthercatResponse")
(setf (get rokubimini_msgs::FirmwareUpdateEthercatResponse :definition-)
      "string file_name
string file_path
uint32 password
---
bool result
")



(provide :rokubimini_msgs/FirmwareUpdateEthercat "3e589c8a3c17e65c13355340a4ec94ff")


