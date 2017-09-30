; Auto-generated. Do not edit!


(cl:in-package lilac_fundamentals-msg)


;//! \htmlinclude Cell.msg.html

(cl:defclass <Cell> (roslisp-msg-protocol:ros-message)
  ((walls
    :reader walls
    :initarg :walls
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass Cell (<Cell>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Cell>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Cell)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lilac_fundamentals-msg:<Cell> is deprecated: use lilac_fundamentals-msg:Cell instead.")))

(cl:ensure-generic-function 'walls-val :lambda-list '(m))
(cl:defmethod walls-val ((m <Cell>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lilac_fundamentals-msg:walls-val is deprecated.  Use lilac_fundamentals-msg:walls instead.")
  (walls m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Cell>)))
    "Constants for message type '<Cell>"
  '((:RIGHT . 0)
    (:TOP . 1)
    (:LEFT . 2)
    (:BOTTOM . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Cell)))
    "Constants for message type 'Cell"
  '((:RIGHT . 0)
    (:TOP . 1)
    (:LEFT . 2)
    (:BOTTOM . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Cell>) ostream)
  "Serializes a message object of type '<Cell>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'walls))))
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
   (cl:slot-value msg 'walls))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Cell>) istream)
  "Deserializes a message object of type '<Cell>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'walls) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'walls)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Cell>)))
  "Returns string type for a message object of type '<Cell>"
  "lilac_fundamentals/Cell")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Cell)))
  "Returns string type for a message object of type 'Cell"
  "lilac_fundamentals/Cell")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Cell>)))
  "Returns md5sum for a message object of type '<Cell>"
  "95bb061dc101cae41f56cabb3faafc66")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Cell)))
  "Returns md5sum for a message object of type 'Cell"
  "95bb061dc101cae41f56cabb3faafc66")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Cell>)))
  "Returns full string definition for message of type '<Cell>"
  (cl:format cl:nil "# constants for walls~%int32 RIGHT = 0~%int32 TOP = 1~%int32 LEFT = 2~%int32 BOTTOM = 3~%# walls that are present in this cell~%int32[] walls~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Cell)))
  "Returns full string definition for message of type 'Cell"
  (cl:format cl:nil "# constants for walls~%int32 RIGHT = 0~%int32 TOP = 1~%int32 LEFT = 2~%int32 BOTTOM = 3~%# walls that are present in this cell~%int32[] walls~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Cell>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'walls) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Cell>))
  "Converts a ROS message object to a list"
  (cl:list 'Cell
    (cl:cons ':walls (walls msg))
))
