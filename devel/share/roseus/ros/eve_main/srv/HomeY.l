;; Auto-generated. Do not edit!


(when (boundp 'eve_main::HomeY)
  (if (not (find-package "EVE_MAIN"))
    (make-package "EVE_MAIN"))
  (shadow 'HomeY (find-package "EVE_MAIN")))
(unless (find-package "EVE_MAIN::HOMEY")
  (make-package "EVE_MAIN::HOMEY"))
(unless (find-package "EVE_MAIN::HOMEYREQUEST")
  (make-package "EVE_MAIN::HOMEYREQUEST"))
(unless (find-package "EVE_MAIN::HOMEYRESPONSE")
  (make-package "EVE_MAIN::HOMEYRESPONSE"))

(in-package "ROS")





(defclass eve_main::HomeYRequest
  :super ros::object
  :slots (_speed ))

(defmethod eve_main::HomeYRequest
  (:init
   (&key
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _speed (float __speed))
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; float32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass eve_main::HomeYResponse
  :super ros::object
  :slots ())

(defmethod eve_main::HomeYResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass eve_main::HomeY
  :super ros::object
  :slots ())

(setf (get eve_main::HomeY :md5sum-) "ca65bba734a79b4a6707341d829f4d5c")
(setf (get eve_main::HomeY :datatype-) "eve_main/HomeY")
(setf (get eve_main::HomeY :request) eve_main::HomeYRequest)
(setf (get eve_main::HomeY :response) eve_main::HomeYResponse)

(defmethod eve_main::HomeYRequest
  (:response () (instance eve_main::HomeYResponse :init)))

(setf (get eve_main::HomeYRequest :md5sum-) "ca65bba734a79b4a6707341d829f4d5c")
(setf (get eve_main::HomeYRequest :datatype-) "eve_main/HomeYRequest")
(setf (get eve_main::HomeYRequest :definition-)
      "float32 speed
---
")

(setf (get eve_main::HomeYResponse :md5sum-) "ca65bba734a79b4a6707341d829f4d5c")
(setf (get eve_main::HomeYResponse :datatype-) "eve_main/HomeYResponse")
(setf (get eve_main::HomeYResponse :definition-)
      "float32 speed
---
")



(provide :eve_main/HomeY "ca65bba734a79b4a6707341d829f4d5c")


