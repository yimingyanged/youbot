; Auto-generated. Do not edit!


(cl:in-package slam_backend-msg)


;//! \htmlinclude NodeAdded.msg.html

(cl:defclass <NodeAdded> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (node_id
    :reader node_id
    :initarg :node_id
    :type cl:integer
    :initform 0))
)

(cl:defclass NodeAdded (<NodeAdded>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NodeAdded>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NodeAdded)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name slam_backend-msg:<NodeAdded> is deprecated: use slam_backend-msg:NodeAdded instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <NodeAdded>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader slam_backend-msg:header-val is deprecated.  Use slam_backend-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'node_id-val :lambda-list '(m))
(cl:defmethod node_id-val ((m <NodeAdded>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader slam_backend-msg:node_id-val is deprecated.  Use slam_backend-msg:node_id instead.")
  (node_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NodeAdded>) ostream)
  "Serializes a message object of type '<NodeAdded>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'node_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'node_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'node_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'node_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NodeAdded>) istream)
  "Deserializes a message object of type '<NodeAdded>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'node_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'node_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'node_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'node_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NodeAdded>)))
  "Returns string type for a message object of type '<NodeAdded>"
  "slam_backend/NodeAdded")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NodeAdded)))
  "Returns string type for a message object of type 'NodeAdded"
  "slam_backend/NodeAdded")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NodeAdded>)))
  "Returns md5sum for a message object of type '<NodeAdded>"
  "0f0f149dcbc88289a8b92f87d91db39b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NodeAdded)))
  "Returns md5sum for a message object of type 'NodeAdded"
  "0f0f149dcbc88289a8b92f87d91db39b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NodeAdded>)))
  "Returns full string definition for message of type '<NodeAdded>"
  (cl:format cl:nil "Header header~%uint32 node_id~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NodeAdded)))
  "Returns full string definition for message of type 'NodeAdded"
  (cl:format cl:nil "Header header~%uint32 node_id~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NodeAdded>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NodeAdded>))
  "Converts a ROS message object to a list"
  (cl:list 'NodeAdded
    (cl:cons ':header (header msg))
    (cl:cons ':node_id (node_id msg))
))
