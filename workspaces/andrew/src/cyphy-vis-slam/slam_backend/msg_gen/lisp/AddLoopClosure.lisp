; Auto-generated. Do not edit!


(cl:in-package slam_backend-msg)


;//! \htmlinclude AddLoopClosure.msg.html

(cl:defclass <AddLoopClosure> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (node_id1
    :reader node_id1
    :initarg :node_id1
    :type cl:integer
    :initform 0)
   (node_id2
    :reader node_id2
    :initarg :node_id2
    :type cl:integer
    :initform 0)
   (interpolated_time
    :reader interpolated_time
    :initarg :interpolated_time
    :type cl:real
    :initform 0)
   (frac
    :reader frac
    :initarg :frac
    :type cl:float
    :initform 0.0)
   (transform
    :reader transform
    :initarg :transform
    :type geometry_msgs-msg:PoseWithCovarianceStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseWithCovarianceStamped)))
)

(cl:defclass AddLoopClosure (<AddLoopClosure>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddLoopClosure>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddLoopClosure)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name slam_backend-msg:<AddLoopClosure> is deprecated: use slam_backend-msg:AddLoopClosure instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AddLoopClosure>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader slam_backend-msg:header-val is deprecated.  Use slam_backend-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'node_id1-val :lambda-list '(m))
(cl:defmethod node_id1-val ((m <AddLoopClosure>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader slam_backend-msg:node_id1-val is deprecated.  Use slam_backend-msg:node_id1 instead.")
  (node_id1 m))

(cl:ensure-generic-function 'node_id2-val :lambda-list '(m))
(cl:defmethod node_id2-val ((m <AddLoopClosure>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader slam_backend-msg:node_id2-val is deprecated.  Use slam_backend-msg:node_id2 instead.")
  (node_id2 m))

(cl:ensure-generic-function 'interpolated_time-val :lambda-list '(m))
(cl:defmethod interpolated_time-val ((m <AddLoopClosure>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader slam_backend-msg:interpolated_time-val is deprecated.  Use slam_backend-msg:interpolated_time instead.")
  (interpolated_time m))

(cl:ensure-generic-function 'frac-val :lambda-list '(m))
(cl:defmethod frac-val ((m <AddLoopClosure>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader slam_backend-msg:frac-val is deprecated.  Use slam_backend-msg:frac instead.")
  (frac m))

(cl:ensure-generic-function 'transform-val :lambda-list '(m))
(cl:defmethod transform-val ((m <AddLoopClosure>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader slam_backend-msg:transform-val is deprecated.  Use slam_backend-msg:transform instead.")
  (transform m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddLoopClosure>) ostream)
  "Serializes a message object of type '<AddLoopClosure>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'node_id1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'node_id2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'interpolated_time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'interpolated_time) (cl:floor (cl:slot-value msg 'interpolated_time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'frac))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'transform) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddLoopClosure>) istream)
  "Deserializes a message object of type '<AddLoopClosure>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'node_id1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'node_id2) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'interpolated_time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'frac) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'transform) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddLoopClosure>)))
  "Returns string type for a message object of type '<AddLoopClosure>"
  "slam_backend/AddLoopClosure")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddLoopClosure)))
  "Returns string type for a message object of type 'AddLoopClosure"
  "slam_backend/AddLoopClosure")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddLoopClosure>)))
  "Returns md5sum for a message object of type '<AddLoopClosure>"
  "d376227da7717c769b6aec116b010f98")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddLoopClosure)))
  "Returns md5sum for a message object of type 'AddLoopClosure"
  "d376227da7717c769b6aec116b010f98")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddLoopClosure>)))
  "Returns full string definition for message of type '<AddLoopClosure>"
  (cl:format cl:nil "Header header~%int32 node_id1~%int32 node_id2~%time interpolated_time~%float32 frac~%geometry_msgs/PoseWithCovarianceStamped transform~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovarianceStamped~%# This expresses an estimated pose with a reference coordinate frame and timestamp~%~%Header header~%PoseWithCovariance pose~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddLoopClosure)))
  "Returns full string definition for message of type 'AddLoopClosure"
  (cl:format cl:nil "Header header~%int32 node_id1~%int32 node_id2~%time interpolated_time~%float32 frac~%geometry_msgs/PoseWithCovarianceStamped transform~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovarianceStamped~%# This expresses an estimated pose with a reference coordinate frame and timestamp~%~%Header header~%PoseWithCovariance pose~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddLoopClosure>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     8
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'transform))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddLoopClosure>))
  "Converts a ROS message object to a list"
  (cl:list 'AddLoopClosure
    (cl:cons ':header (header msg))
    (cl:cons ':node_id1 (node_id1 msg))
    (cl:cons ':node_id2 (node_id2 msg))
    (cl:cons ':interpolated_time (interpolated_time msg))
    (cl:cons ':frac (frac msg))
    (cl:cons ':transform (transform msg))
))
