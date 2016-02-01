; Auto-generated. Do not edit!


(cl:in-package sp_pos-msg)


;//! \htmlinclude utmData.msg.html

(cl:defclass <utmData> (roslisp-msg-protocol:ros-message)
  ((utmNorthing
    :reader utmNorthing
    :initarg :utmNorthing
    :type cl:float
    :initform 0.0)
   (utmEasting
    :reader utmEasting
    :initarg :utmEasting
    :type cl:float
    :initform 0.0)
   (utmZone
    :reader utmZone
    :initarg :utmZone
    :type cl:string
    :initform ""))
)

(cl:defclass utmData (<utmData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <utmData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'utmData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sp_pos-msg:<utmData> is deprecated: use sp_pos-msg:utmData instead.")))

(cl:ensure-generic-function 'utmNorthing-val :lambda-list '(m))
(cl:defmethod utmNorthing-val ((m <utmData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sp_pos-msg:utmNorthing-val is deprecated.  Use sp_pos-msg:utmNorthing instead.")
  (utmNorthing m))

(cl:ensure-generic-function 'utmEasting-val :lambda-list '(m))
(cl:defmethod utmEasting-val ((m <utmData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sp_pos-msg:utmEasting-val is deprecated.  Use sp_pos-msg:utmEasting instead.")
  (utmEasting m))

(cl:ensure-generic-function 'utmZone-val :lambda-list '(m))
(cl:defmethod utmZone-val ((m <utmData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sp_pos-msg:utmZone-val is deprecated.  Use sp_pos-msg:utmZone instead.")
  (utmZone m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <utmData>) ostream)
  "Serializes a message object of type '<utmData>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'utmNorthing))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'utmEasting))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'utmZone))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'utmZone))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <utmData>) istream)
  "Deserializes a message object of type '<utmData>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'utmNorthing) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'utmEasting) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'utmZone) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'utmZone) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<utmData>)))
  "Returns string type for a message object of type '<utmData>"
  "sp_pos/utmData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'utmData)))
  "Returns string type for a message object of type 'utmData"
  "sp_pos/utmData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<utmData>)))
  "Returns md5sum for a message object of type '<utmData>"
  "49b19eb1db7e204c2b9962b01694fa27")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'utmData)))
  "Returns md5sum for a message object of type 'utmData"
  "49b19eb1db7e204c2b9962b01694fa27")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<utmData>)))
  "Returns full string definition for message of type '<utmData>"
  (cl:format cl:nil "float64 utmNorthing~%float64 utmEasting~%string utmZone~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'utmData)))
  "Returns full string definition for message of type 'utmData"
  (cl:format cl:nil "float64 utmNorthing~%float64 utmEasting~%string utmZone~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <utmData>))
  (cl:+ 0
     8
     8
     4 (cl:length (cl:slot-value msg 'utmZone))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <utmData>))
  "Converts a ROS message object to a list"
  (cl:list 'utmData
    (cl:cons ':utmNorthing (utmNorthing msg))
    (cl:cons ':utmEasting (utmEasting msg))
    (cl:cons ':utmZone (utmZone msg))
))
