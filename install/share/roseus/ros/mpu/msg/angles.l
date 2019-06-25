;; Auto-generated. Do not edit!


(when (boundp 'mpu::angles)
  (if (not (find-package "MPU"))
    (make-package "MPU"))
  (shadow 'angles (find-package "MPU")))
(unless (find-package "MPU::ANGLES")
  (make-package "MPU::ANGLES"))

(in-package "ROS")
;;//! \htmlinclude angles.msg.html


(defclass mpu::angles
  :super ros::object
  :slots (_angleX _angleY _angleZ ))

(defmethod mpu::angles
  (:init
   (&key
    ((:angleX __angleX) 0)
    ((:angleY __angleY) 0)
    ((:angleZ __angleZ) 0)
    )
   (send-super :init)
   (setq _angleX (round __angleX))
   (setq _angleY (round __angleY))
   (setq _angleZ (round __angleZ))
   self)
  (:angleX
   (&optional __angleX)
   (if __angleX (setq _angleX __angleX)) _angleX)
  (:angleY
   (&optional __angleY)
   (if __angleY (setq _angleY __angleY)) _angleY)
  (:angleZ
   (&optional __angleZ)
   (if __angleZ (setq _angleZ __angleZ)) _angleZ)
  (:serialization-length
   ()
   (+
    ;; int16 _angleX
    2
    ;; int16 _angleY
    2
    ;; int16 _angleZ
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _angleX
       (write-word _angleX s)
     ;; int16 _angleY
       (write-word _angleY s)
     ;; int16 _angleZ
       (write-word _angleZ s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _angleX
     (setq _angleX (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _angleY
     (setq _angleY (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _angleZ
     (setq _angleZ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get mpu::angles :md5sum-) "76513374ad214381430fd6261a078518")
(setf (get mpu::angles :datatype-) "mpu/angles")
(setf (get mpu::angles :definition-)
      "int16 angleX
int16 angleY
int16 angleZ

")



(provide :mpu/angles "76513374ad214381430fd6261a078518")


