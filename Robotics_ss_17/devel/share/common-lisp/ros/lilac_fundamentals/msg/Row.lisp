; Auto-generated. Do not edit!


(cl:in-package lilac_fundamentals-msg)


;//! \htmlinclude Row.msg.html

(cl:defclass <Row> (roslisp-msg-protocol:ros-message)
  ((cells
    :reader cells
    :initarg :cells
    :type (cl:vector lilac_fundamentals-msg:Cell)
   :initform (cl:make-array 0 :element-type 'lilac_fundamentals-msg:Cell :initial-element (cl:make-instance 'lilac_fundamentals-msg:Cell))))
)

(cl:defclass Row (<Row>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Row>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Row)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lilac_fundamentals-msg:<Row> is deprecated: use lilac_fundamentals-msg:Row instead.")))

(cl:ensure-generic-function 'cells-val :lambda-list '(m))
(cl:defmethod cells-val ((m <Row>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lilac_fundamentals-msg:cells-val is deprecated.  Use lilac_fundamentals-msg:cells instead.")
  (cells m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Row>) ostream)
  "Serializes a message object of type '<Row>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cells))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'cells))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Row>) istream)
  "Deserializes a message object of type '<Row>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cells) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cells)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'lilac_fundamentals-msg:Cell))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Row>)))
  "Returns string type for a message object of type '<Row>"
  "lilac_fundamentals/Row")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Row)))
  "Returns string type for a message object of type 'Row"
  "lilac_fundamentals/Row")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Row>)))
  "Returns md5sum for a message object of type '<Row>"
  "7c660e46d29bbf9fcdd244a209ce096c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Row)))
  "Returns md5sum for a message object of type 'Row"
  "7c660e46d29bbf9fcdd244a209ce096c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Row>)))
  "Returns full string definition for message of type '<Row>"
  (cl:format cl:nil "Cell[] cells~%~%================================================================================~%MSG: lilac_fundamentals/Cell~%# constants for walls~%int32 RIGHT = 0~%int32 TOP = 1~%int32 LEFT = 2~%int32 BOTTOM = 3~%# walls that are present in this cell~%int32[] walls~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Row)))
  "Returns full string definition for message of type 'Row"
  (cl:format cl:nil "Cell[] cells~%~%================================================================================~%MSG: lilac_fundamentals/Cell~%# constants for walls~%int32 RIGHT = 0~%int32 TOP = 1~%int32 LEFT = 2~%int32 BOTTOM = 3~%# walls that are present in this cell~%int32[] walls~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Row>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cells) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Row>))
  "Converts a ROS message object to a list"
  (cl:list 'Row
    (cl:cons ':cells (cells msg))
))
