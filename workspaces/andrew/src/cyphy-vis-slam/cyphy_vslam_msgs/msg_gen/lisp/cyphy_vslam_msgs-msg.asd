
(cl:in-package :asdf)

(defsystem "cyphy_vslam_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Match" :depends-on ("_package_Match"))
    (:file "_package_Match" :depends-on ("_package"))
  ))