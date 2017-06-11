; Auto-generated. Do not edit!


(cl:in-package a3_help-srv)


;//! \htmlinclude RequestGoal-request.msg.html

(cl:defclass <RequestGoal-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:integer
    :initform 0))
)

(cl:defclass RequestGoal-request (<RequestGoal-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RequestGoal-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RequestGoal-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name a3_help-srv:<RequestGoal-request> is deprecated: use a3_help-srv:RequestGoal-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <RequestGoal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a3_help-srv:x-val is deprecated.  Use a3_help-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <RequestGoal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a3_help-srv:y-val is deprecated.  Use a3_help-srv:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RequestGoal-request>) ostream)
  "Serializes a message object of type '<RequestGoal-request>"
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RequestGoal-request>) istream)
  "Deserializes a message object of type '<RequestGoal-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RequestGoal-request>)))
  "Returns string type for a service object of type '<RequestGoal-request>"
  "a3_help/RequestGoalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestGoal-request)))
  "Returns string type for a service object of type 'RequestGoal-request"
  "a3_help/RequestGoalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RequestGoal-request>)))
  "Returns md5sum for a message object of type '<RequestGoal-request>"
  "33e40041c161ed97cccea8af6f436cfd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RequestGoal-request)))
  "Returns md5sum for a message object of type 'RequestGoal-request"
  "33e40041c161ed97cccea8af6f436cfd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RequestGoal-request>)))
  "Returns full string definition for message of type '<RequestGoal-request>"
  (cl:format cl:nil "int64 x~%int64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RequestGoal-request)))
  "Returns full string definition for message of type 'RequestGoal-request"
  (cl:format cl:nil "int64 x~%int64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RequestGoal-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RequestGoal-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RequestGoal-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
;//! \htmlinclude RequestGoal-response.msg.html

(cl:defclass <RequestGoal-response> (roslisp-msg-protocol:ros-message)
  ((ack
    :reader ack
    :initarg :ack
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RequestGoal-response (<RequestGoal-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RequestGoal-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RequestGoal-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name a3_help-srv:<RequestGoal-response> is deprecated: use a3_help-srv:RequestGoal-response instead.")))

(cl:ensure-generic-function 'ack-val :lambda-list '(m))
(cl:defmethod ack-val ((m <RequestGoal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a3_help-srv:ack-val is deprecated.  Use a3_help-srv:ack instead.")
  (ack m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RequestGoal-response>) ostream)
  "Serializes a message object of type '<RequestGoal-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ack) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RequestGoal-response>) istream)
  "Deserializes a message object of type '<RequestGoal-response>"
    (cl:setf (cl:slot-value msg 'ack) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RequestGoal-response>)))
  "Returns string type for a service object of type '<RequestGoal-response>"
  "a3_help/RequestGoalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestGoal-response)))
  "Returns string type for a service object of type 'RequestGoal-response"
  "a3_help/RequestGoalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RequestGoal-response>)))
  "Returns md5sum for a message object of type '<RequestGoal-response>"
  "33e40041c161ed97cccea8af6f436cfd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RequestGoal-response)))
  "Returns md5sum for a message object of type 'RequestGoal-response"
  "33e40041c161ed97cccea8af6f436cfd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RequestGoal-response>)))
  "Returns full string definition for message of type '<RequestGoal-response>"
  (cl:format cl:nil "bool ack~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RequestGoal-response)))
  "Returns full string definition for message of type 'RequestGoal-response"
  (cl:format cl:nil "bool ack~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RequestGoal-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RequestGoal-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RequestGoal-response
    (cl:cons ':ack (ack msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RequestGoal)))
  'RequestGoal-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RequestGoal)))
  'RequestGoal-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestGoal)))
  "Returns string type for a service object of type '<RequestGoal>"
  "a3_help/RequestGoal")