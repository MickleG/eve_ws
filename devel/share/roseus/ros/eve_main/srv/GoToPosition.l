;; Auto-generated. Do not edit!


(when (boundp 'eve_main::GoToPosition)
  (if (not (find-package "EVE_MAIN"))
    (make-package "EVE_MAIN"))
  (shadow 'GoToPosition (find-package "EVE_MAIN")))
(unless (find-package "EVE_MAIN::GOTOPOSITION")
  (make-package "EVE_MAIN::GOTOPOSITION"))
(unless (find-package "EVE_MAIN::GOTOPOSITIONREQUEST")
  (make-package "EVE_MAIN::GOTOPOSITIONREQUEST"))
(unless (find-package "EVE_MAIN::GOTOPOSITIONRESPONSE")
  (make-package "EVE_MAIN::GOTOPOSITIONRESPONSE"))

(in-package "ROS")





(defclass eve_main::GoToPositionRequest
  :super ros::object
  :slots (_desiredXPosition _desiredZPosition _speed ))

(defmethod eve_main::GoToPositionRequest
  (:init
   (&key
    ((:desiredXPosition __desiredXPosition) 0.0)
    ((:desiredZPosition __desiredZPosition) 0.0)
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _desiredXPosition (float __desiredXPosition))
   (setq _desiredZPosition (float __desiredZPosition))
   (setq _speed (float __speed))
   self)
  (:desiredXPosition
   (&optional __desiredXPosition)
   (if __desiredXPosition (setq _desiredXPosition __desiredXPosition)) _desiredXPosition)
  (:desiredZPosition
   (&optional __desiredZPosition)
   (if __desiredZPosition (setq _desiredZPosition __desiredZPosition)) _desiredZPosition)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; float32 _desiredXPosition
    4
    ;; float32 _desiredZPosition
    4
    ;; float32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _desiredXPosition
       (sys::poke _desiredXPosition (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _desiredZPosition
       (sys::poke _desiredZPosition (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _desiredXPosition
     (setq _desiredXPosition (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _desiredZPosition
     (setq _desiredZPosition (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass eve_main::GoToPositionResponse
  :super ros::object
  :slots ())

(defmethod eve_main::GoToPositionResponse
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

(defclass eve_main::GoToPosition
  :super ros::object
  :slots ())

(setf (get eve_main::GoToPosition :md5sum-) "c8d13069b95fb9c6b68d27613e61705e")
(setf (get eve_main::GoToPosition :datatype-) "eve_main/GoToPosition")
(setf (get eve_main::GoToPosition :request) eve_main::GoToPositionRequest)
(setf (get eve_main::GoToPosition :response) eve_main::GoToPositionResponse)

(defmethod eve_main::GoToPositionRequest
  (:response () (instance eve_main::GoToPositionResponse :init)))

(setf (get eve_main::GoToPositionRequest :md5sum-) "c8d13069b95fb9c6b68d27613e61705e")
(setf (get eve_main::GoToPositionRequest :datatype-) "eve_main/GoToPositionRequest")
(setf (get eve_main::GoToPositionRequest :definition-)
      "float32 desiredXPosition
float32 desiredZPosition
float32 speed
---
")

(setf (get eve_main::GoToPositionResponse :md5sum-) "c8d13069b95fb9c6b68d27613e61705e")
(setf (get eve_main::GoToPositionResponse :datatype-) "eve_main/GoToPositionResponse")
(setf (get eve_main::GoToPositionResponse :definition-)
      "float32 desiredXPosition
float32 desiredZPosition
float32 speed
---
")



(provide :eve_main/GoToPosition "c8d13069b95fb9c6b68d27613e61705e")


