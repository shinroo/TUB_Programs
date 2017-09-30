; Auto-generated. Do not edit!


(cl:in-package lilac_fundamentals-msg)


;//! \htmlinclude ActualLocalization.msg.html

(cl:defclass <ActualLocalization> (roslisp-msg-protocol:ros-message)
  ((row
    :reader row
    :initarg :row
    :type cl:fixnum
    :initform 0)
   (column
    :reader column
    :initarg :column
    :type cl:fixnum
    :initform 0)
   (orientation
    :reader orientation
    :initarg :orientation
    :type cl:fixnum
    :initform 0)
   (localized
    :reader localized
    :initarg :localized
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ActualLocalization (<ActualLocalization>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActualLocalization>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActualLocalization)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lilac_fundamentals-msg:<ActualLocalization> is deprecated: use lilac_fundamentals-msg:ActualLocalization instead.")))

(cl:ensure-generic-function 'row-val :lambda-list '(m))
(cl:defmethod row-val ((m <ActualLocalization>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lilac_fundamentals-msg:row-val is deprecated.  Use lilac_fundamentals-msg:row instead.")
  (row m))

(cl:ensure-generic-function 'column-val :lambda-list '(m))
(cl:defmethod column-val ((m <ActualLocalization>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lilac_fundamentals-msg:column-val is deprecated.  Use lilac_fundamentals-msg:column instead.")
  (column m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <ActualLocalization>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lilac_fundamentals-msg:orientation-val is deprecated.  Use lilac_fundamentals-msg:orientation instead.")
  (orientation m))

(cl:ensure-generic-function 'localized-val :lambda-list '(m))
(cl:defmethod localized-val ((m <ActualLocalization>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lilac_fundamentals-msg:localized-val is deprecated.  Use lilac_fundamentals-msg:localized instead.")
  (localized m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActualLocalization>) ostream)
  "Serializes a message object of type '<ActualLocalization>"
  (cl:let* ((signed (cl:slot-value msg 'row)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'column)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'orientation)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'localized) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActualLocalization>) istream)
  "Deserializes a message object of type '<ActualLocalization>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'row) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'column) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'orientation) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:slot-value msg 'localized) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActualLocalization>)))
  "Returns string type for a message object of type '<ActualLocalization>"
  "lilac_fundamentals/ActualLocalization")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActualLocalization)))
  "Returns string type for a message object of type 'ActualLocalization"
  "lilac_fundamentals/ActualLocalization")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActualLocalization>)))
  "Returns md5sum for a message object of type '<ActualLocalization>"
  "893e067a4f69fb209f2b2e037a5aa78b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActualLocalization)))
  "Returns md5sum for a message object of type 'ActualLocalization"
  "893e067a4f69fb209f2b2e037a5aa78b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActualLocalization>)))
  "Returns full string definition for message of type '<ActualLocalization>"
  (cl:format cl:nil "int16 row~%int16 column~%int16 orientation~%bool localized~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActualLocalization)))
  "Returns full string definition for message of type 'ActualLocalization"
  (cl:format cl:nil "int16 row~%int16 column~%int16 orientation~%bool localized~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActualLocalization>))
  (cl:+ 0
     2
     2
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActualLocalization>))
  "Converts a ROS message object to a list"
  (cl:list 'ActualLocalization
    (cl:cons ':row (row msg))
    (cl:cons ':column (column msg))
    (cl:cons ':orientation (orientation msg))
    (cl:cons ':localized (localized msg))
))
