; Auto-generated. Do not edit!


(cl:in-package image_cache-srv)


;//! \htmlinclude GetNeighbours-request.msg.html

(cl:defclass <GetNeighbours-request> (roslisp-msg-protocol:ros-message)
  ((nodeID
    :reader nodeID
    :initarg :nodeID
    :type cl:integer
    :initform 0))
)

(cl:defclass GetNeighbours-request (<GetNeighbours-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetNeighbours-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetNeighbours-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name image_cache-srv:<GetNeighbours-request> is deprecated: use image_cache-srv:GetNeighbours-request instead.")))

(cl:ensure-generic-function 'nodeID-val :lambda-list '(m))
(cl:defmethod nodeID-val ((m <GetNeighbours-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_cache-srv:nodeID-val is deprecated.  Use image_cache-srv:nodeID instead.")
  (nodeID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetNeighbours-request>) ostream)
  "Serializes a message object of type '<GetNeighbours-request>"
  (cl:let* ((signed (cl:slot-value msg 'nodeID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetNeighbours-request>) istream)
  "Deserializes a message object of type '<GetNeighbours-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nodeID) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetNeighbours-request>)))
  "Returns string type for a service object of type '<GetNeighbours-request>"
  "image_cache/GetNeighboursRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetNeighbours-request)))
  "Returns string type for a service object of type 'GetNeighbours-request"
  "image_cache/GetNeighboursRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetNeighbours-request>)))
  "Returns md5sum for a message object of type '<GetNeighbours-request>"
  "ccd112ae98ddab13d42702fa6352dc06")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetNeighbours-request)))
  "Returns md5sum for a message object of type 'GetNeighbours-request"
  "ccd112ae98ddab13d42702fa6352dc06")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetNeighbours-request>)))
  "Returns full string definition for message of type '<GetNeighbours-request>"
  (cl:format cl:nil "~%int32 nodeID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetNeighbours-request)))
  "Returns full string definition for message of type 'GetNeighbours-request"
  (cl:format cl:nil "~%int32 nodeID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetNeighbours-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetNeighbours-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetNeighbours-request
    (cl:cons ':nodeID (nodeID msg))
))
;//! \htmlinclude GetNeighbours-response.msg.html

(cl:defclass <GetNeighbours-response> (roslisp-msg-protocol:ros-message)
  ((node1
    :reader node1
    :initarg :node1
    :type cl:integer
    :initform 0)
   (node2
    :reader node2
    :initarg :node2
    :type cl:integer
    :initform 0)
   (frac
    :reader frac
    :initarg :frac
    :type cl:float
    :initform 0.0)
   (inter_time
    :reader inter_time
    :initarg :inter_time
    :type cl:real
    :initform 0))
)

(cl:defclass GetNeighbours-response (<GetNeighbours-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetNeighbours-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetNeighbours-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name image_cache-srv:<GetNeighbours-response> is deprecated: use image_cache-srv:GetNeighbours-response instead.")))

(cl:ensure-generic-function 'node1-val :lambda-list '(m))
(cl:defmethod node1-val ((m <GetNeighbours-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_cache-srv:node1-val is deprecated.  Use image_cache-srv:node1 instead.")
  (node1 m))

(cl:ensure-generic-function 'node2-val :lambda-list '(m))
(cl:defmethod node2-val ((m <GetNeighbours-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_cache-srv:node2-val is deprecated.  Use image_cache-srv:node2 instead.")
  (node2 m))

(cl:ensure-generic-function 'frac-val :lambda-list '(m))
(cl:defmethod frac-val ((m <GetNeighbours-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_cache-srv:frac-val is deprecated.  Use image_cache-srv:frac instead.")
  (frac m))

(cl:ensure-generic-function 'inter_time-val :lambda-list '(m))
(cl:defmethod inter_time-val ((m <GetNeighbours-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_cache-srv:inter_time-val is deprecated.  Use image_cache-srv:inter_time instead.")
  (inter_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetNeighbours-response>) ostream)
  "Serializes a message object of type '<GetNeighbours-response>"
  (cl:let* ((signed (cl:slot-value msg 'node1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'node2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'frac))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'inter_time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'inter_time) (cl:floor (cl:slot-value msg 'inter_time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetNeighbours-response>) istream)
  "Deserializes a message object of type '<GetNeighbours-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'node1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'node2) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'frac) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'inter_time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetNeighbours-response>)))
  "Returns string type for a service object of type '<GetNeighbours-response>"
  "image_cache/GetNeighboursResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetNeighbours-response)))
  "Returns string type for a service object of type 'GetNeighbours-response"
  "image_cache/GetNeighboursResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetNeighbours-response>)))
  "Returns md5sum for a message object of type '<GetNeighbours-response>"
  "ccd112ae98ddab13d42702fa6352dc06")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetNeighbours-response)))
  "Returns md5sum for a message object of type 'GetNeighbours-response"
  "ccd112ae98ddab13d42702fa6352dc06")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetNeighbours-response>)))
  "Returns full string definition for message of type '<GetNeighbours-response>"
  (cl:format cl:nil "int32 node1~%int32 node2~%float32 frac~%time inter_time~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetNeighbours-response)))
  "Returns full string definition for message of type 'GetNeighbours-response"
  (cl:format cl:nil "int32 node1~%int32 node2~%float32 frac~%time inter_time~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetNeighbours-response>))
  (cl:+ 0
     4
     4
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetNeighbours-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetNeighbours-response
    (cl:cons ':node1 (node1 msg))
    (cl:cons ':node2 (node2 msg))
    (cl:cons ':frac (frac msg))
    (cl:cons ':inter_time (inter_time msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetNeighbours)))
  'GetNeighbours-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetNeighbours)))
  'GetNeighbours-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetNeighbours)))
  "Returns string type for a service object of type '<GetNeighbours>"
  "image_cache/GetNeighbours")