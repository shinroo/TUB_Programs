; Auto-generated. Do not edit!


(cl:in-package lilac_fundamentals-srv)


;//! \htmlinclude ExecutePlan-request.msg.html

(cl:defclass <ExecutePlan-request> (roslisp-msg-protocol:ros-message)
  ((plan
    :reader plan
    :initarg :plan
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass ExecutePlan-request (<ExecutePlan-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecutePlan-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecutePlan-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lilac_fundamentals-srv:<ExecutePlan-request> is deprecated: use lilac_fundamentals-srv:ExecutePlan-request instead.")))

(cl:ensure-generic-function 'plan-val :lambda-list '(m))
(cl:defmethod plan-val ((m <ExecutePlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lilac_fundamentals-srv:plan-val is deprecated.  Use lilac_fundamentals-srv:plan instead.")
  (plan m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ExecutePlan-request>)))
    "Constants for message type '<ExecutePlan-request>"
  '((:RIGHT . 0)
    (:UP . 1)
    (:LEFT . 2)
    (:DOWN . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ExecutePlan-request)))
    "Constants for message type 'ExecutePlan-request"
  '((:RIGHT . 0)
    (:UP . 1)
    (:LEFT . 2)
    (:DOWN . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecutePlan-request>) ostream)
  "Serializes a message object of type '<ExecutePlan-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'plan))))
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
   (cl:slot-value msg 'plan))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecutePlan-request>) istream)
  "Deserializes a message object of type '<ExecutePlan-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'plan) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'plan)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecutePlan-request>)))
  "Returns string type for a service object of type '<ExecutePlan-request>"
  "lilac_fundamentals/ExecutePlanRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecutePlan-request)))
  "Returns string type for a service object of type 'ExecutePlan-request"
  "lilac_fundamentals/ExecutePlanRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecutePlan-request>)))
  "Returns md5sum for a message object of type '<ExecutePlan-request>"
  "8b63ba8f8960301ada9bc2fc61f718b7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecutePlan-request)))
  "Returns md5sum for a message object of type 'ExecutePlan-request"
  "8b63ba8f8960301ada9bc2fc61f718b7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecutePlan-request>)))
  "Returns full string definition for message of type '<ExecutePlan-request>"
  (cl:format cl:nil "~%int32 RIGHT = 0~%int32 UP = 1~%int32 LEFT = 2~%int32 DOWN = 3~%~%int32[] plan~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecutePlan-request)))
  "Returns full string definition for message of type 'ExecutePlan-request"
  (cl:format cl:nil "~%int32 RIGHT = 0~%int32 UP = 1~%int32 LEFT = 2~%int32 DOWN = 3~%~%int32[] plan~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecutePlan-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'plan) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecutePlan-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecutePlan-request
    (cl:cons ':plan (plan msg))
))
;//! \htmlinclude ExecutePlan-response.msg.html

(cl:defclass <ExecutePlan-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ExecutePlan-response (<ExecutePlan-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecutePlan-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecutePlan-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lilac_fundamentals-srv:<ExecutePlan-response> is deprecated: use lilac_fundamentals-srv:ExecutePlan-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ExecutePlan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lilac_fundamentals-srv:success-val is deprecated.  Use lilac_fundamentals-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecutePlan-response>) ostream)
  "Serializes a message object of type '<ExecutePlan-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecutePlan-response>) istream)
  "Deserializes a message object of type '<ExecutePlan-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecutePlan-response>)))
  "Returns string type for a service object of type '<ExecutePlan-response>"
  "lilac_fundamentals/ExecutePlanResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecutePlan-response)))
  "Returns string type for a service object of type 'ExecutePlan-response"
  "lilac_fundamentals/ExecutePlanResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecutePlan-response>)))
  "Returns md5sum for a message object of type '<ExecutePlan-response>"
  "8b63ba8f8960301ada9bc2fc61f718b7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecutePlan-response)))
  "Returns md5sum for a message object of type 'ExecutePlan-response"
  "8b63ba8f8960301ada9bc2fc61f718b7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecutePlan-response>)))
  "Returns full string definition for message of type '<ExecutePlan-response>"
  (cl:format cl:nil "~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecutePlan-response)))
  "Returns full string definition for message of type 'ExecutePlan-response"
  (cl:format cl:nil "~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecutePlan-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecutePlan-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecutePlan-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ExecutePlan)))
  'ExecutePlan-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ExecutePlan)))
  'ExecutePlan-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecutePlan)))
  "Returns string type for a service object of type '<ExecutePlan>"
  "lilac_fundamentals/ExecutePlan")