
(cl:in-package :asdf)

(defsystem "slam_backend-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AddLoopClosure" :depends-on ("_package_AddLoopClosure"))
    (:file "_package_AddLoopClosure" :depends-on ("_package"))
    (:file "Graph" :depends-on ("_package_Graph"))
    (:file "_package_Graph" :depends-on ("_package"))
    (:file "NodeAdded" :depends-on ("_package_NodeAdded"))
    (:file "_package_NodeAdded" :depends-on ("_package"))
  ))