
(cl:in-package :asdf)

(defsystem "ur_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :ur_msgs-msg
)
  :components ((:file "_package")
    (:file "GetRobotSoftwareVersion" :depends-on ("_package_GetRobotSoftwareVersion"))
    (:file "_package_GetRobotSoftwareVersion" :depends-on ("_package"))
    (:file "SetAnalogOutput" :depends-on ("_package_SetAnalogOutput"))
    (:file "_package_SetAnalogOutput" :depends-on ("_package"))
    (:file "SetForceMode" :depends-on ("_package_SetForceMode"))
    (:file "_package_SetForceMode" :depends-on ("_package"))
    (:file "SetIO" :depends-on ("_package_SetIO"))
    (:file "_package_SetIO" :depends-on ("_package"))
    (:file "SetPayload" :depends-on ("_package_SetPayload"))
    (:file "_package_SetPayload" :depends-on ("_package"))
    (:file "SetSpeedSliderFraction" :depends-on ("_package_SetSpeedSliderFraction"))
    (:file "_package_SetSpeedSliderFraction" :depends-on ("_package"))
  ))