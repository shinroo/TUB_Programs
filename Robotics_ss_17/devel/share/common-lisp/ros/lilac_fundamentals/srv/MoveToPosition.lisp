; Auto-generated. Do not edit!


(cl:in-package lilac_fundamentals-srv)


;//! \htmlinclude MoveToPosition-request.msg.html

(cl:defclass <MoveToPosition-request> (roslisp-msg-protocol:ros-message)
  ((row
    :reader row
    :initarg :row
    :type cl:integer
    :initform 0)
   (column
    :reader column
    :initarg :column
    :type cl:integer
    :initform 0))
)

(cl:defclass MoveToPosition-request (<MoveToPosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveToPosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveToPosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lilac_fundamentals-srv:<MoveToPosition-request> is deprecated: use lilac_fundamentals-srv:MoveToPosition-request instead.")))

(cl:ensure-generic-function 'row-val :lambda-list '(m))
(cl:defmethod row-val ((m <MoveToPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lilac_fundamentals-srv:row-val is deprecated.  Use lilac_fundamentals-srv:row instead.")
  (row m))

(cl:ensure-generic-function 'column-val :lambda-list '(m))
(cl:defmethod column-val ((m <MoveToPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lilac_fundamentals-srv:column-val is deprecated.  Use lilac_fundamentals-srv:column instead.")
  (column m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveToPosition-request>) ostream)
  "Serializes a message object of type '<MoveToPosition-request>"
  (cl:let* ((signed (cl:slot-value msg 'row)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'column)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveToPosition-request>) istream)
  "Deserializes a message object of type '<MoveToPosition-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'row) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'column) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveToPosition-request>)))
  "Returns string type for a service object of type '<MoveToPosition-request>"
  "lilac_fundamentals/MoveToPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveToPosition-request)))
  "Returns string type for a service object of type 'MoveToPosition-request"
  "lilac_fundamentals/MoveToPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveToPosition-request>)))
  "Returns md5sum for a message object of type '<MoveToPosition-request>"
  "5af7833001d12f0202ac6047222437e9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveToPosition-request)))
  "Returns md5sum for a message object of type 'MoveToPosition-request"
  "5af7833001d12f0202ac6047222437e9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveToPosition-request>)))
  "Returns full string definition for message of type '<MoveToPosition-request>"
  (cl:format cl:nil "~%int32 row~%int32 column~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveToPosition-request)))
  "Returns full string definition for message of type 'MoveToPosition-request"
  (cl:format cl:nil "~%int32 row~%int32 column~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveToPosition-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveToPosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveToPosition-request
    (cl:cons ':row (row msg))
    (cl:cons ':column (column msg))
))
;//! \htmlinclude MoveToPosition-response.msg.html

(cl:defclass <MoveToPosition-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MoveToPosition-response (<MoveToPosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveToPosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveToPosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lilac_fundamentals-srv:<MoveToPosition-response> is deprecated: use lilac_fundamentals-srv:MoveToPosition-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <MoveToPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lilac_fundamentals-srv:success-val is deprecated.  Use lilac_fundamentals-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveToPosition-response>) ostream)
  "Serializes a message object of type '<MoveToPosition-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveToPosition-response>) istream)
  "Deserializes a message object of type '<MoveToPosition-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveToPosition-response>)))
  "Returns string type for a service object of type '<MoveToPosition-response>"
  "lilac_fundamentals/MoveToPositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveToPosition-response)))
  "Returns string type for a service object of type 'MoveToPosition-response"
  "lilac_fundamentals/MoveToPositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveToPosition-response>)))
  "Returns md5sum for a message object of type '<MoveToPosition-response>"
  "5af7833001d12f0202ac6047222437e9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveToPosition-response)))
  "Returns md5sum for a message object of type 'MoveToPosition-response"
  "5af7833001d12f0202ac6047222437e9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveToPosition-response>)))
  "Returns full string definition for message of type '<MoveToPosition-response>"
  (cl:format cl:nil "~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveToPosition-response)))
  "Returns full string definition for message of type 'MoveToPosition-response"
  (cl:format cl:nil "~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveToPosition-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveToPosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveToPosition-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveToPosition)))
  'MoveToPosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveToPosition)))
  'MoveToPosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveToPosition)))
  "Returns string type for a service object of type '<MoveToPosition>"
  "lilac_fundamentals/MoveToPosition")