;; Auto-generated. Do not edit!


(when (boundp 'eve_main::EndEffectorPosition)
  (if (not (find-package "EVE_MAIN"))
    (make-package "EVE_MAIN"))
  (shadow 'EndEffectorPosition (find-package "EVE_MAIN")))
(unless (find-package "EVE_MAIN::ENDEFFECTORPOSITION")
  (make-package "EVE_MAIN::ENDEFFECTORPOSITION"))

(in-package "ROS")
;;//! \htmlinclude EndEffectorPosition.msg.html


(defclass eve_main::EndEffectorPosition
  :super ros::object
  :slots (_xPosition _yPosition _zPosition ))

(defmethod eve_main::EndEffectorPosition
  (:init
   (&key
    ((:xPosition __xPosition) 0.0)
    ((:yPosition __yPosition) 0.0)
    ((:zPosition __zPosition) 0.0)
    )
   (send-super :init)
   (setq _xPosition (float __xPosition))
   (setq _yPosition (float __yPosition))
   (setq _zPosition (float __zPosition))
   self)
  (:xPosition
   (&optional __xPosition)
   (if __xPosition (setq _xPosition __xPosition)) _xPosition)
  (:yPosition
   (&optional __yPosition)
   (if __yPosition (setq _yPosition __yPosition)) _yPosition)
  (:zPosition
   (&optional __zPosition)
   (if __zPosition (setq _zPosition __zPosition)) _zPosition)
  (:serialization-length
   ()
   (+
    ;; float32 _xPosition
    4
    ;; float32 _yPosition
    4
    ;; float32 _zPosition
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _xPosition
       (sys::poke _xPosition (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yPosition
       (sys::poke _yPosition (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _zPosition
       (sys::poke _zPosition (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _xPosition
     (setq _xPosition (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yPosition
     (setq _yPosition (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _zPosition
     (setq _zPosition (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get eve_main::EndEffectorPosition :md5sum-) "f98d79514754ade5731789b7227e61f3")
(setf (get eve_main::EndEffectorPosition :datatype-) "eve_main/EndEffectorPosition")
(setf (get eve_main::EndEffectorPosition :definition-)
      "float32 xPosition
float32 yPosition
float32 zPosition

")



(provide :eve_main/EndEffectorPosition "f98d79514754ade5731789b7227e61f3")


