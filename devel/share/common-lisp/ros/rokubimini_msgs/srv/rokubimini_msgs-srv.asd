
(cl:in-package :asdf)

(defsystem "rokubimini_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "FirmwareUpdateEthercat" :depends-on ("_package_FirmwareUpdateEthercat"))
    (:file "_package_FirmwareUpdateEthercat" :depends-on ("_package"))
    (:file "FirmwareUpdateSerial" :depends-on ("_package_FirmwareUpdateSerial"))
    (:file "_package_FirmwareUpdateSerial" :depends-on ("_package"))
    (:file "GetSensorConfiguration" :depends-on ("_package_GetSensorConfiguration"))
    (:file "_package_GetSensorConfiguration" :depends-on ("_package"))
    (:file "ResetWrench" :depends-on ("_package_ResetWrench"))
    (:file "_package_ResetWrench" :depends-on ("_package"))
    (:file "SetSensorConfiguration" :depends-on ("_package_SetSensorConfiguration"))
    (:file "_package_SetSensorConfiguration" :depends-on ("_package"))
  ))