; Auto-generated. Do not edit!


(cl:in-package cyphy_vslam_msgs-msg)


;//! \htmlinclude Match.msg.html

(cl:defclass <Match> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (fromImgSeq
    :reader fromImgSeq
    :initarg :fromImgSeq
    :type cl:integer
    :initform 0)
   (toImgSeq
    :reader toImgSeq
    :initarg :toImgSeq
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (toImgMatch
    :reader toImgMatch
    :initarg :toImgMatch
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Match (<Match>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Match>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Match)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cyphy_vslam_msgs-msg:<Match> is deprecated: use cyphy_vslam_msgs-msg:Match instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Match>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyphy_vslam_msgs-msg:header-val is deprecated.  Use cyphy_vslam_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'fromImgSeq-val :lambda-list '(m))
(cl:defmethod fromImgSeq-val ((m <Match>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyphy_vslam_msgs-msg:fromImgSeq-val is deprecated.  Use cyphy_vslam_msgs-msg:fromImgSeq instead.")
  (fromImgSeq m))

(cl:ensure-generic-function 'toImgSeq-val :lambda-list '(m))
(cl:defmethod toImgSeq-val ((m <Match>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyphy_vslam_msgs-msg:toImgSeq-val is deprecated.  Use cyphy_vslam_msgs-msg:toImgSeq instead.")
  (toImgSeq m))

(cl:ensure-generic-function 'toImgMatch-val :lambda-list '(m))
(cl:defmethod toImgMatch-val ((m <Match>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyphy_vslam_msgs-msg:toImgMatch-val is deprecated.  Use cyphy_vslam_msgs-msg:toImgMatch instead.")
  (toImgMatch m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Match>) ostream)
  "Serializes a message object of type '<Match>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'fromImgSeq)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'toImgSeq))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'toImgSeq))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'toImgMatch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'toImgMatch))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Match>) istream)
  "Deserializes a message object of type '<Match>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fromImgSeq) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'toImgSeq) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'toImgSeq)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'toImgMatch) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'toImgMatch)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Match>)))
  "Returns string type for a message object of type '<Match>"
  "cyphy_vslam_msgs/Match")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Match)))
  "Returns string type for a message object of type 'Match"
  "cyphy_vslam_msgs/Match")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Match>)))
  "Returns md5sum for a message object of type '<Match>"
  "ba3cfc4913b84354e8ec41dc24ba0836")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Match)))
  "Returns md5sum for a message object of type 'Match"
  "ba3cfc4913b84354e8ec41dc24ba0836")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Match>)))
  "Returns full string definition for message of type '<Match>"
  (cl:format cl:nil "Header header~%int32 fromImgSeq~%int32[] toImgSeq~%float64[] toImgMatch~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Match)))
  "Returns full string definition for message of type 'Match"
  (cl:format cl:nil "Header header~%int32 fromImgSeq~%int32[] toImgSeq~%float64[] toImgMatch~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Match>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'toImgSeq) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'toImgMatch) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Match>))
  "Converts a ROS message object to a list"
  (cl:list 'Match
    (cl:cons ':header (header msg))
    (cl:cons ':fromImgSeq (fromImgSeq msg))
    (cl:cons ':toImgSeq (toImgSeq msg))
    (cl:cons ':toImgMatch (toImgMatch msg))
))
