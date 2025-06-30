;; Auto-generated. Do not edit!


(when (boundp 'rokubimini_msgs::FirmwareUpdateSerial)
  (if (not (find-package "ROKUBIMINI_MSGS"))
    (make-package "ROKUBIMINI_MSGS"))
  (shadow 'FirmwareUpdateSerial (find-package "ROKUBIMINI_MSGS")))
(unless (find-package "ROKUBIMINI_MSGS::FIRMWAREUPDATESERIAL")
  (make-package "ROKUBIMINI_MSGS::FIRMWAREUPDATESERIAL"))
(unless (find-package "ROKUBIMINI_MSGS::FIRMWAREUPDATESERIALREQUEST")
  (make-package "ROKUBIMINI_MSGS::FIRMWAREUPDATESERIALREQUEST"))
(unless (find-package "ROKUBIMINI_MSGS::FIRMWAREUPDATESERIALRESPONSE")
  (make-package "ROKUBIMINI_MSGS::FIRMWAREUPDATESERIALRESPONSE"))

(in-package "ROS")





(defclass rokubimini_msgs::FirmwareUpdateSerialRequest
  :super ros::object
  :slots (_file_path ))

(defmethod rokubimini_msgs::FirmwareUpdateSerialRequest
  (:init
   (&key
    ((:file_path __file_path) "")
    )
   (send-super :init)
   (setq _file_path (string __file_path))
   self)
  (:file_path
   (&optional __file_path)
   (if __file_path (setq _file_path __file_path)) _file_path)
  (:serialization-length
   ()
   (+
    ;; string _file_path
    4 (length _file_path)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _file_path
       (write-long (length _file_path) s) (princ _file_path s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _file_path
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _file_path (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass rokubimini_msgs::FirmwareUpdateSerialResponse
  :super ros::object
  :slots (_result ))

(defmethod rokubimini_msgs::FirmwareUpdateSerialResponse
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

(defclass rokubimini_msgs::FirmwareUpdateSerial
  :super ros::object
  :slots ())

(setf (get rokubimini_msgs::FirmwareUpdateSerial :md5sum-) "b223f245a1d13c9e179ae29717131752")
(setf (get rokubimini_msgs::FirmwareUpdateSerial :datatype-) "rokubimini_msgs/FirmwareUpdateSerial")
(setf (get rokubimini_msgs::FirmwareUpdateSerial :request) rokubimini_msgs::FirmwareUpdateSerialRequest)
(setf (get rokubimini_msgs::FirmwareUpdateSerial :response) rokubimini_msgs::FirmwareUpdateSerialResponse)

(defmethod rokubimini_msgs::FirmwareUpdateSerialRequest
  (:response () (instance rokubimini_msgs::FirmwareUpdateSerialResponse :init)))

(setf (get rokubimini_msgs::FirmwareUpdateSerialRequest :md5sum-) "b223f245a1d13c9e179ae29717131752")
(setf (get rokubimini_msgs::FirmwareUpdateSerialRequest :datatype-) "rokubimini_msgs/FirmwareUpdateSerialRequest")
(setf (get rokubimini_msgs::FirmwareUpdateSerialRequest :definition-)
      "string file_path
---
bool result
")

(setf (get rokubimini_msgs::FirmwareUpdateSerialResponse :md5sum-) "b223f245a1d13c9e179ae29717131752")
(setf (get rokubimini_msgs::FirmwareUpdateSerialResponse :datatype-) "rokubimini_msgs/FirmwareUpdateSerialResponse")
(setf (get rokubimini_msgs::FirmwareUpdateSerialResponse :definition-)
      "string file_path
---
bool result
")



(provide :rokubimini_msgs/FirmwareUpdateSerial "b223f245a1d13c9e179ae29717131752")


