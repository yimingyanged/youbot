
(cl:in-package :asdf)

(defsystem "geometric_verification-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Debug" :depends-on ("_package_Debug"))
    (:file "_package_Debug" :depends-on ("_package"))
  ))