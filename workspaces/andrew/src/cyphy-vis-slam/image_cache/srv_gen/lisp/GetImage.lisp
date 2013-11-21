; Auto-generated. Do not edit!


(cl:in-package image_cache-srv)


;//! \htmlinclude GetImage-request.msg.html

(cl:defclass <GetImage-request> (roslisp-msg-protocol:ros-message)
  ((from
    :reader from
    :initarg :from
    :type cl:integer
    :initform 0)
   (to
    :reader to
    :initarg :to
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (rectified
    :reader rectified
    :initarg :rectified
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetImage-request (<GetImage-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetImage-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetImage-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name image_cache-srv:<GetImage-request> is deprecated: use image_cache-srv:GetImage-request instead.")))

(cl:ensure-generic-function 'from-val :lambda-list '(m))
(cl:defmethod from-val ((m <GetImage-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_cache-srv:from-val is deprecated.  Use image_cache-srv:from instead.")
  (from m))

(cl:ensure-generic-function 'to-val :lambda-list '(m))
(cl:defmethod to-val ((m <GetImage-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_cache-srv:to-val is deprecated.  Use image_cache-srv:to instead.")
  (to m))

(cl:ensure-generic-function 'rectified-val :lambda-list '(m))
(cl:defmethod rectified-val ((m <GetImage-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_cache-srv:rectified-val is deprecated.  Use image_cache-srv:rectified instead.")
  (rectified m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetImage-request>) ostream)
  "Serializes a message object of type '<GetImage-request>"
  (cl:let* ((signed (cl:slot-value msg 'from)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'to))))
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
   (cl:slot-value msg 'to))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'rectified) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetImage-request>) istream)
  "Deserializes a message object of type '<GetImage-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'from) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'to) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'to)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
    (cl:setf (cl:slot-value msg 'rectified) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetImage-request>)))
  "Returns string type for a service object of type '<GetImage-request>"
  "image_cache/GetImageRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetImage-request)))
  "Returns string type for a service object of type 'GetImage-request"
  "image_cache/GetImageRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetImage-request>)))
  "Returns md5sum for a message object of type '<GetImage-request>"
  "977c89c0bbd082bbbbc98818a5de36b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetImage-request)))
  "Returns md5sum for a message object of type 'GetImage-request"
  "977c89c0bbd082bbbbc98818a5de36b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetImage-request>)))
  "Returns full string definition for message of type '<GetImage-request>"
  (cl:format cl:nil "~%int32 from~%int32[] to~%bool rectified~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetImage-request)))
  "Returns full string definition for message of type 'GetImage-request"
  (cl:format cl:nil "~%int32 from~%int32[] to~%bool rectified~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetImage-request>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'to) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetImage-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetImage-request
    (cl:cons ':from (from msg))
    (cl:cons ':to (to msg))
    (cl:cons ':rectified (rectified msg))
))
;//! \htmlinclude GetImage-response.msg.html

(cl:defclass <GetImage-response> (roslisp-msg-protocol:ros-message)
  ((from_id
    :reader from_id
    :initarg :from_id
    :type cl:integer
    :initform 0)
   (to_id
    :reader to_id
    :initarg :to_id
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (from_l
    :reader from_l
    :initarg :from_l
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (from_r
    :reader from_r
    :initarg :from_r
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (to_l
    :reader to_l
    :initarg :to_l
    :type (cl:vector sensor_msgs-msg:Image)
   :initform (cl:make-array 0 :element-type 'sensor_msgs-msg:Image :initial-element (cl:make-instance 'sensor_msgs-msg:Image)))
   (to_r
    :reader to_r
    :initarg :to_r
    :type (cl:vector sensor_msgs-msg:Image)
   :initform (cl:make-array 0 :element-type 'sensor_msgs-msg:Image :initial-element (cl:make-instance 'sensor_msgs-msg:Image))))
)

(cl:defclass GetImage-response (<GetImage-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetImage-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetImage-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name image_cache-srv:<GetImage-response> is deprecated: use image_cache-srv:GetImage-response instead.")))

(cl:ensure-generic-function 'from_id-val :lambda-list '(m))
(cl:defmethod from_id-val ((m <GetImage-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_cache-srv:from_id-val is deprecated.  Use image_cache-srv:from_id instead.")
  (from_id m))

(cl:ensure-generic-function 'to_id-val :lambda-list '(m))
(cl:defmethod to_id-val ((m <GetImage-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_cache-srv:to_id-val is deprecated.  Use image_cache-srv:to_id instead.")
  (to_id m))

(cl:ensure-generic-function 'from_l-val :lambda-list '(m))
(cl:defmethod from_l-val ((m <GetImage-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_cache-srv:from_l-val is deprecated.  Use image_cache-srv:from_l instead.")
  (from_l m))

(cl:ensure-generic-function 'from_r-val :lambda-list '(m))
(cl:defmethod from_r-val ((m <GetImage-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_cache-srv:from_r-val is deprecated.  Use image_cache-srv:from_r instead.")
  (from_r m))

(cl:ensure-generic-function 'to_l-val :lambda-list '(m))
(cl:defmethod to_l-val ((m <GetImage-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_cache-srv:to_l-val is deprecated.  Use image_cache-srv:to_l instead.")
  (to_l m))

(cl:ensure-generic-function 'to_r-val :lambda-list '(m))
(cl:defmethod to_r-val ((m <GetImage-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_cache-srv:to_r-val is deprecated.  Use image_cache-srv:to_r instead.")
  (to_r m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetImage-response>) ostream)
  "Serializes a message object of type '<GetImage-response>"
  (cl:let* ((signed (cl:slot-value msg 'from_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'to_id))))
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
   (cl:slot-value msg 'to_id))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'from_l) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'from_r) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'to_l))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'to_l))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'to_r))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'to_r))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetImage-response>) istream)
  "Deserializes a message object of type '<GetImage-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'from_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'to_id) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'to_id)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'from_l) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'from_r) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'to_l) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'to_l)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sensor_msgs-msg:Image))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'to_r) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'to_r)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sensor_msgs-msg:Image))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetImage-response>)))
  "Returns string type for a service object of type '<GetImage-response>"
  "image_cache/GetImageResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetImage-response)))
  "Returns string type for a service object of type 'GetImage-response"
  "image_cache/GetImageResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetImage-response>)))
  "Returns md5sum for a message object of type '<GetImage-response>"
  "977c89c0bbd082bbbbc98818a5de36b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetImage-response)))
  "Returns md5sum for a message object of type 'GetImage-response"
  "977c89c0bbd082bbbbc98818a5de36b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetImage-response>)))
  "Returns full string definition for message of type '<GetImage-response>"
  (cl:format cl:nil "int32 from_id~%int32[] to_id~%sensor_msgs/Image from_l~%sensor_msgs/Image from_r~%sensor_msgs/Image[] to_l~%sensor_msgs/Image[] to_r~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetImage-response)))
  "Returns full string definition for message of type 'GetImage-response"
  (cl:format cl:nil "int32 from_id~%int32[] to_id~%sensor_msgs/Image from_l~%sensor_msgs/Image from_r~%sensor_msgs/Image[] to_l~%sensor_msgs/Image[] to_r~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetImage-response>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'to_id) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'from_l))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'from_r))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'to_l) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'to_r) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetImage-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetImage-response
    (cl:cons ':from_id (from_id msg))
    (cl:cons ':to_id (to_id msg))
    (cl:cons ':from_l (from_l msg))
    (cl:cons ':from_r (from_r msg))
    (cl:cons ':to_l (to_l msg))
    (cl:cons ':to_r (to_r msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetImage)))
  'GetImage-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetImage)))
  'GetImage-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetImage)))
  "Returns string type for a service object of type '<GetImage>"
  "image_cache/GetImage")