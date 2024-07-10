; Auto-generated. Do not edit!


(cl:in-package eve_main-srv)


;//! \htmlinclude GetPosition-request.msg.html

(cl:defclass <GetPosition-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetPosition-request (<GetPosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eve_main-srv:<GetPosition-request> is deprecated: use eve_main-srv:GetPosition-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPosition-request>) ostream)
  "Serializes a message object of type '<GetPosition-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPosition-request>) istream)
  "Deserializes a message object of type '<GetPosition-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPosition-request>)))
  "Returns string type for a service object of type '<GetPosition-request>"
  "eve_main/GetPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPosition-request)))
  "Returns string type for a service object of type 'GetPosition-request"
  "eve_main/GetPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPosition-request>)))
  "Returns md5sum for a message object of type '<GetPosition-request>"
  "f98d79514754ade5731789b7227e61f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPosition-request)))
  "Returns md5sum for a message object of type 'GetPosition-request"
  "f98d79514754ade5731789b7227e61f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPosition-request>)))
  "Returns full string definition for message of type '<GetPosition-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPosition-request)))
  "Returns full string definition for message of type 'GetPosition-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPosition-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPosition-request
))
;//! \htmlinclude GetPosition-response.msg.html

(cl:defclass <GetPosition-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GetPosition-response (<GetPosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eve_main-srv:<GetPosition-response> is deprecated: use eve_main-srv:GetPosition-response instead.")))

(cl:ensure-generic-function 'xPosition-val :lambda-list '(m))
(cl:defmethod xPosition-val ((m <GetPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eve_main-srv:xPosition-val is deprecated.  Use eve_main-srv:xPosition instead.")
  (xPosition m))

(cl:ensure-generic-function 'yPosition-val :lambda-list '(m))
(cl:defmethod yPosition-val ((m <GetPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eve_main-srv:yPosition-val is deprecated.  Use eve_main-srv:yPosition instead.")
  (yPosition m))

(cl:ensure-generic-function 'zPosition-val :lambda-list '(m))
(cl:defmethod zPosition-val ((m <GetPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eve_main-srv:zPosition-val is deprecated.  Use eve_main-srv:zPosition instead.")
  (zPosition m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPosition-response>) ostream)
  "Serializes a message object of type '<GetPosition-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPosition-response>) istream)
  "Deserializes a message object of type '<GetPosition-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPosition-response>)))
  "Returns string type for a service object of type '<GetPosition-response>"
  "eve_main/GetPositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPosition-response)))
  "Returns string type for a service object of type 'GetPosition-response"
  "eve_main/GetPositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPosition-response>)))
  "Returns md5sum for a message object of type '<GetPosition-response>"
  "f98d79514754ade5731789b7227e61f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPosition-response)))
  "Returns md5sum for a message object of type 'GetPosition-response"
  "f98d79514754ade5731789b7227e61f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPosition-response>)))
  "Returns full string definition for message of type '<GetPosition-response>"
  (cl:format cl:nil "float32 xPosition~%float32 yPosition~%float32 zPosition~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPosition-response)))
  "Returns full string definition for message of type 'GetPosition-response"
  (cl:format cl:nil "float32 xPosition~%float32 yPosition~%float32 zPosition~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPosition-response>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPosition-response
    (cl:cons ':xPosition (xPosition msg))
    (cl:cons ':yPosition (yPosition msg))
    (cl:cons ':zPosition (zPosition msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPosition)))
  'GetPosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPosition)))
  'GetPosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPosition)))
  "Returns string type for a service object of type '<GetPosition>"
  "eve_main/GetPosition")