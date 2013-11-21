
(cl:in-package :asdf)

(defsystem "image_cache-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "GetNeighbours" :depends-on ("_package_GetNeighbours"))
    (:file "_package_GetNeighbours" :depends-on ("_package"))
    (:file "GetInfo" :depends-on ("_package_GetInfo"))
    (:file "_package_GetInfo" :depends-on ("_package"))
    (:file "GetImage" :depends-on ("_package_GetImage"))
    (:file "_package_GetImage" :depends-on ("_package"))
  ))