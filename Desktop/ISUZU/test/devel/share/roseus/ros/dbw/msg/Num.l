;; Auto-generated. Do not edit!


(when (boundp 'dbw::Num)
  (if (not (find-package "DBW"))
    (make-package "DBW"))
  (shadow 'Num (find-package "DBW")))
(unless (find-package "DBW::NUM")
  (make-package "DBW::NUM"))

(in-package "ROS")
;;//! \htmlinclude Num.msg.html


(defclass dbw::Num
  :super ros::object
  :slots (_num ))

(defmethod dbw::Num
  (:init
   (&key
    ((:num __num) 0)
    )
   (send-super :init)
   (setq _num (round __num))
   self)
  (:num
   (&optional __num)
   (if __num (setq _num __num)) _num)
  (:serialization-length
   ()
   (+
    ;; int32 _num
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _num
       (write-long _num s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _num
     (setq _num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get dbw::Num :md5sum-) "54b3c80efd6fae6e6ffff8a4b9facd69")
(setf (get dbw::Num :datatype-) "dbw/Num")
(setf (get dbw::Num :definition-)
      "int32 num

")



(provide :dbw/Num "54b3c80efd6fae6e6ffff8a4b9facd69")


