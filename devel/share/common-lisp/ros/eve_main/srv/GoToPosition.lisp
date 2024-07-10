; Auto-generated. Do not edit!


(cl:in-package eve_main-srv)


;//! \htmlinclude GoToPosition-request.msg.html

(cl:defclass <GoToPosition-request> (roslisp-msg-protocol:ros-message)
  ((desiredXPosition
    :reader desiredXPosition
    :initarg :desiredXPosition
    :type cl:float
    :initform 0.0)
   (desiredZPosition
    :reader desiredZPosition
    :initarg :desiredZPosition
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass GoToPosition-request (<GoToPosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoToPosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoToPosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eve_main-srv:<GoToPosition-request> is deprecated: use eve_main-srv:GoToPosition-request instead.")))

(cl:ensure-generic-function 'desiredXPosition-val :lambda-list '(m))
(cl:defmethod desiredXPosition-val ((m <GoToPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eve_main-srv:desiredXPosition-val is deprecated.  Use eve_main-srv:desiredXPosition instead.")
  (desiredXPosition m))

(cl:ensure-generic-function 'desiredZPosition-val :lambda-list '(m))
(cl:defmethod desiredZPosition-val ((m <GoToPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eve_main-srv:desiredZPosition-val is deprecated.  Use eve_main-srv:desiredZPosition instead.")
  (desiredZPosition m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <GoToPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eve_main-srv:speed-val is deprecated.  Use eve_main-srv:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoToPosition-request>) ostream)
  "Serializes a message object of type '<GoToPosition-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'desiredXPosition))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'desiredZPosition))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoToPosition-request>) istream)
  "Deserializes a message object of type '<GoToPosition-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'desiredXPosition) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'desiredZPosition) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoToPosition-request>)))
  "Returns string type for a service object of type '<GoToPosition-request>"
  "eve_main/GoToPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoToPosition-request)))
  "Returns string type for a service object of type 'GoToPosition-request"
  "eve_main/GoToPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoToPosition-request>)))
  "Returns md5sum for a message object of type '<GoToPosition-request>"
  "c8d13069b95fb9c6b68d27613e61705e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoToPosition-request)))
  "Returns md5sum for a message object of type 'GoToPosition-request"
  "c8d13069b95fb9c6b68d27613e61705e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoToPosition-request>)))
  "Returns full string definition for message of type '<GoToPosition-request>"
  (cl:format cl:nil "float32 desiredXPosition~%float32 desiredZPosition~%float32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoToPosition-request)))
  "Returns full string definition for message of type 'GoToPosition-request"
  (cl:format cl:nil "float32 desiredXPosition~%float32 desiredZPosition~%float32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoToPosition-request>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoToPosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GoToPosition-request
    (cl:cons ':desiredXPosition (desiredXPosition msg))
    (cl:cons ':desiredZPosition (desiredZPosition msg))
    (cl:cons ':speed (speed msg))
))
;//! \htmlinclude GoToPosition-response.msg.html

(cl:defclass <GoToPosition-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GoToPosition-response (<GoToPosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoToPosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoToPosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eve_main-srv:<GoToPosition-response> is deprecated: use eve_main-srv:GoToPosition-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoToPosition-response>) ostream)
  "Serializes a message object of type '<GoToPosition-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoToPosition-response>) istream)
  "Deserializes a message object of type '<GoToPosition-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoToPosition-response>)))
  "Returns string type for a service object of type '<GoToPosition-response>"
  "eve_main/GoToPositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoToPosition-response)))
  "Returns string type for a service object of type 'GoToPosition-response"
  "eve_main/GoToPositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoToPosition-response>)))
  "Returns md5sum for a message object of type '<GoToPosition-response>"
  "c8d13069b95fb9c6b68d27613e61705e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoToPosition-response)))
  "Returns md5sum for a message object of type 'GoToPosition-response"
  "c8d13069b95fb9c6b68d27613e61705e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoToPosition-response>)))
  "Returns full string definition for message of type '<GoToPosition-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoToPosition-response)))
  "Returns full string definition for message of type 'GoToPosition-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoToPosition-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoToPosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GoToPosition-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GoToPosition)))
  'GoToPosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GoToPosition)))
  'GoToPosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoToPosition)))
  "Returns string type for a service object of type '<GoToPosition>"
  "eve_main/GoToPosition")