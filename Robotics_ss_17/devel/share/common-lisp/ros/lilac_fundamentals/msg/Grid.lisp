; Auto-generated. Do not edit!


(cl:in-package lilac_fundamentals-msg)


;//! \htmlinclude Grid.msg.html

(cl:defclass <Grid> (roslisp-msg-protocol:ros-message)
  ((rows
    :reader rows
    :initarg :rows
    :type (cl:vector lilac_fundamentals-msg:Row)
   :initform (cl:make-array 0 :element-type 'lilac_fundamentals-msg:Row :initial-element (cl:make-instance 'lilac_fundamentals-msg:Row))))
)

(cl:defclass Grid (<Grid>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Grid>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Grid)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lilac_fundamentals-msg:<Grid> is deprecated: use lilac_fundamentals-msg:Grid instead.")))

(cl:ensure-generic-function 'rows-val :lambda-list '(m))
(cl:defmethod rows-val ((m <Grid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lilac_fundamentals-msg:rows-val is deprecated.  Use lilac_fundamentals-msg:rows instead.")
  (rows m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Grid>) ostream)
  "Serializes a message object of type '<Grid>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'rows))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'rows))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Grid>) istream)
  "Deserializes a message object of type '<Grid>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rows) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'rows)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'lilac_fundamentals-msg:Row))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Grid>)))
  "Returns string type for a message object of type '<Grid>"
  "lilac_fundamentals/Grid")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Grid)))
  "Returns string type for a message object of type 'Grid"
  "lilac_fundamentals/Grid")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Grid>)))
  "Returns md5sum for a message object of type '<Grid>"
  "f8a346b2eb1f1badd86d28e470d95fd4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Grid)))
  "Returns md5sum for a message object of type 'Grid"
  "f8a346b2eb1f1badd86d28e470d95fd4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Grid>)))
  "Returns full string definition for message of type '<Grid>"
  (cl:format cl:nil "Row[] rows~%~%================================================================================~%MSG: lilac_fundamentals/Row~%Cell[] cells~%~%================================================================================~%MSG: lilac_fundamentals/Cell~%# constants for walls~%int32 RIGHT = 0~%int32 TOP = 1~%int32 LEFT = 2~%int32 BOTTOM = 3~%# walls that are present in this cell~%int32[] walls~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Grid)))
  "Returns full string definition for message of type 'Grid"
  (cl:format cl:nil "Row[] rows~%~%================================================================================~%MSG: lilac_fundamentals/Row~%Cell[] cells~%~%================================================================================~%MSG: lilac_fundamentals/Cell~%# constants for walls~%int32 RIGHT = 0~%int32 TOP = 1~%int32 LEFT = 2~%int32 BOTTOM = 3~%# walls that are present in this cell~%int32[] walls~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Grid>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'rows) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Grid>))
  "Converts a ROS message object to a list"
  (cl:list 'Grid
    (cl:cons ':rows (rows msg))
))
