;; Auto-generated. Do not edit!


(when (boundp 'hellocm_msgs::CM2Ext_vhcl)
  (if (not (find-package "HELLOCM_MSGS"))
    (make-package "HELLOCM_MSGS"))
  (shadow 'CM2Ext_vhcl (find-package "HELLOCM_MSGS")))
(unless (find-package "HELLOCM_MSGS::CM2EXT_VHCL")
  (make-package "HELLOCM_MSGS::CM2EXT_VHCL"))

(in-package "ROS")
;;//! \htmlinclude CM2Ext_vhcl.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass hellocm_msgs::CM2Ext_vhcl
  :super ros::object
  :slots (_header _time _cycleno _synthdelay _velocity _heading _pitch _roll ))

(defmethod hellocm_msgs::CM2Ext_vhcl
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:time __time) (instance ros::time :init))
    ((:cycleno __cycleno) 0)
    ((:synthdelay __synthdelay) 0.0)
    ((:velocity __velocity) 0.0)
    ((:heading __heading) 0.0)
    ((:pitch __pitch) 0.0)
    ((:roll __roll) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _time __time)
   (setq _cycleno (round __cycleno))
   (setq _synthdelay (float __synthdelay))
   (setq _velocity (float __velocity))
   (setq _heading (float __heading))
   (setq _pitch (float __pitch))
   (setq _roll (float __roll))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:cycleno
   (&optional __cycleno)
   (if __cycleno (setq _cycleno __cycleno)) _cycleno)
  (:synthdelay
   (&optional __synthdelay)
   (if __synthdelay (setq _synthdelay __synthdelay)) _synthdelay)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; time _time
    8
    ;; int64 _cycleno
    8
    ;; float64 _synthdelay
    8
    ;; float64 _velocity
    8
    ;; float64 _heading
    8
    ;; float64 _pitch
    8
    ;; float64 _roll
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; time _time
       (write-long (send _time :sec) s) (write-long (send _time :nsec) s)
     ;; int64 _cycleno
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _cycleno (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _cycleno) (= (length (_cycleno . bv)) 2)) ;; bignum
              (write-long (ash (elt (_cycleno . bv) 0) 0) s)
              (write-long (ash (elt (_cycleno . bv) 1) -1) s))
             ((and (class _cycleno) (= (length (_cycleno . bv)) 1)) ;; big1
              (write-long (elt (_cycleno . bv) 0) s)
              (write-long (if (>= _cycleno 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _cycleno s)(write-long (if (>= _cycleno 0) 0 #xffffffff) s)))
     ;; float64 _synthdelay
       (sys::poke _synthdelay (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _heading
       (sys::poke _heading (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; time _time
     (send _time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int64 _cycleno
#+(or :alpha :irix6 :x86_64)
      (setf _cycleno (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _cycleno (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _synthdelay
     (setq _synthdelay (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _velocity
     (setq _velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _heading
     (setq _heading (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pitch
     (setq _pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _roll
     (setq _roll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get hellocm_msgs::CM2Ext_vhcl :md5sum-) "3290c3ba936ec21da9927a50bff0314a")
(setf (get hellocm_msgs::CM2Ext_vhcl :datatype-) "hellocm_msgs/CM2Ext_vhcl")
(setf (get hellocm_msgs::CM2Ext_vhcl :definition-)
      "#
# Message from CarMaker ROS Node to external ROS Node
#

# General
Header  header                                 # General ROS Header (optional)
time    time                                   # ROS time when message was sent (optional)
int64   cycleno                                # Cycle number since simulation start (optional)
float64 synthdelay                             # Synthetic delay to check synchronization 

float64 velocity
float64 heading
float64 pitch
float64 roll

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :hellocm_msgs/CM2Ext_vhcl "3290c3ba936ec21da9927a50bff0314a")


