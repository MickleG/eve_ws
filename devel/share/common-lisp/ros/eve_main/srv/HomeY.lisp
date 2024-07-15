; Auto-generated. Do not edit!


(cl:in-package eve_main-srv)


;//! \htmlinclude HomeY-request.msg.html

(cl:defclass <HomeY-request> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass HomeY-request (<HomeY-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HomeY-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HomeY-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eve_main-srv:<HomeY-request> is deprecated: use eve_main-srv:HomeY-request instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <HomeY-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eve_main-srv:speed-val is deprecated.  Use eve_main-srv:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HomeY-request>) ostream)
  "Serializes a message object of type '<HomeY-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HomeY-request>) istream)
  "Deserializes a message object of type '<HomeY-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HomeY-request>)))
  "Returns string type for a service object of type '<HomeY-request>"
  "eve_main/HomeYRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HomeY-request)))
  "Returns string type for a service object of type 'HomeY-request"
  "eve_main/HomeYRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HomeY-request>)))
  "Returns md5sum for a message object of type '<HomeY-request>"
  "ca65bba734a79b4a6707341d829f4d5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HomeY-request)))
  "Returns md5sum for a message object of type 'HomeY-request"
  "ca65bba734a79b4a6707341d829f4d5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HomeY-request>)))
  "Returns full string definition for message of type '<HomeY-request>"
  (cl:format cl:nil "float32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HomeY-request)))
  "Returns full string definition for message of type 'HomeY-request"
  (cl:format cl:nil "float32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HomeY-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HomeY-request>))
  "Converts a ROS message object to a list"
  (cl:list 'HomeY-request
    (cl:cons ':speed (speed msg))
))
;//! \htmlinclude HomeY-response.msg.html

(cl:defclass <HomeY-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass HomeY-response (<HomeY-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HomeY-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HomeY-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eve_main-srv:<HomeY-response> is deprecated: use eve_main-srv:HomeY-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HomeY-response>) ostream)
  "Serializes a message object of type '<HomeY-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HomeY-response>) istream)
  "Deserializes a message object of type '<HomeY-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HomeY-response>)))
  "Returns string type for a service object of type '<HomeY-response>"
  "eve_main/HomeYResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HomeY-response)))
  "Returns string type for a service object of type 'HomeY-response"
  "eve_main/HomeYResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HomeY-response>)))
  "Returns md5sum for a message object of type '<HomeY-response>"
  "ca65bba734a79b4a6707341d829f4d5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HomeY-response)))
  "Returns md5sum for a message object of type 'HomeY-response"
  "ca65bba734a79b4a6707341d829f4d5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HomeY-response>)))
  "Returns full string definition for message of type '<HomeY-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HomeY-response)))
  "Returns full string definition for message of type 'HomeY-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HomeY-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HomeY-response>))
  "Converts a ROS message object to a list"
  (cl:list 'HomeY-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'HomeY)))
  'HomeY-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'HomeY)))
  'HomeY-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HomeY)))
  "Returns string type for a service object of type '<HomeY>"
  "eve_main/HomeY")