
(cl:in-package :asdf)

(defsystem "youbot_common-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PowerBoardState" :depends-on ("_package_PowerBoardState"))
    (:file "_package_PowerBoardState" :depends-on ("_package"))
  ))