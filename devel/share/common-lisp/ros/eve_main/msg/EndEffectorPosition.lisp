; Auto-generated. Do not edit!


(cl:in-package eve_main-msg)


;//! \htmlinclude EndEffectorPosition.msg.html

(cl:defclass <EndEffectorPosition> (roslisp-msg-protocol:ros-message)
  ((xPosition
    :reader xPosition
    :initarg :xPosition
    :type cl:float
    :initform 0.0)
   (yPosition
    :reader yPosition
    :initarg :yPosition
    :type cl:float
    :initform 0.0)
   (zPosition
    :reader zPosition
    :initarg :zPosition
    :type cl:float
    :initform 0.0))
)

(cl:defclass EndEffectorPosition (<EndEffectorPosition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EndEffectorPosition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EndEffectorPosition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eve_main-msg:<EndEffectorPosition> is deprecated: use eve_main-msg:EndEffectorPosition instead.")))

(cl:ensure-generic-function 'xPosition-val :lambda-list '(m))
(cl:defmethod xPosition-val ((m <EndEffectorPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eve_main-msg:xPosition-val is deprecated.  Use eve_main-msg:xPosition instead.")
  (xPosition m))

(cl:ensure-generic-function 'yPosition-val :lambda-list '(m))
(cl:defmethod yPosition-val ((m <EndEffectorPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eve_main-msg:yPosition-val is deprecated.  Use eve_main-msg:yPosition instead.")
  (yPosition m))

(cl:ensure-generic-function 'zPosition-val :lambda-list '(m))
(cl:defmethod zPosition-val ((m <EndEffectorPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eve_main-msg:zPosition-val is deprecated.  Use eve_main-msg:zPosition instead.")
  (zPosition m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EndEffectorPosition>) ostream)
  "Serializes a message object of type '<EndEffectorPosition>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'xPosition))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yPosition))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'zPosition))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EndEffectorPosition>) istream)
  "Deserializes a message object of type '<EndEffectorPosition>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'xPosition) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yPosition) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zPosition) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EndEffectorPosition>)))
  "Returns string type for a message object of type '<EndEffectorPosition>"
  "eve_main/EndEffectorPosition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EndEffectorPosition)))
  "Returns string type for a message object of type 'EndEffectorPosition"
  "eve_main/EndEffectorPosition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EndEffectorPosition>)))
  "Returns md5sum for a message object of type '<EndEffectorPosition>"
  "f98d79514754ade5731789b7227e61f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EndEffectorPosition)))
  "Returns md5sum for a message object of type 'EndEffectorPosition"
  "f98d79514754ade5731789b7227e61f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EndEffectorPosition>)))
  "Returns full string definition for message of type '<EndEffectorPosition>"
  (cl:format cl:nil "float32 xPosition~%float32 yPosition~%float32 zPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EndEffectorPosition)))
  "Returns full string definition for message of type 'EndEffectorPosition"
  (cl:format cl:nil "float32 xPosition~%float32 yPosition~%float32 zPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EndEffectorPosition>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EndEffectorPosition>))
  "Converts a ROS message object to a list"
  (cl:list 'EndEffectorPosition
    (cl:cons ':xPosition (xPosition msg))
    (cl:cons ':yPosition (yPosition msg))
    (cl:cons ':zPosition (zPosition msg))
))
