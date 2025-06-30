
(cl:in-package :asdf)

(defsystem "rokubimini_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Reading" :depends-on ("_package_Reading"))
    (:file "_package_Reading" :depends-on ("_package"))
  ))