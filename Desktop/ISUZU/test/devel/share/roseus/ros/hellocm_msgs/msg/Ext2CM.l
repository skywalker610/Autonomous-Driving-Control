;; Auto-generated. Do not edit!


(when (boundp 'hellocm_msgs::Ext2CM)
  (if (not (find-package "HELLOCM_MSGS"))
    (make-package "HELLOCM_MSGS"))
  (shadow 'Ext2CM (find-package "HELLOCM_MSGS")))
(unless (find-package "HELLOCM_MSGS::EXT2CM")
  (make-package "HELLOCM_MSGS::EXT2CM"))

(in-package "ROS")
;;//! \htmlinclude Ext2CM.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass hellocm_msgs::Ext2CM
  :super ros::object
  :slots (_header _time _cycleno _throttle _brake _steering _gear _GolableEnable ))

(defmethod hellocm_msgs::Ext2CM
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:time __time) (instance ros::time :init))
    ((:cycleno __cycleno) 0)
    ((:throttle __throttle) 0.0)
    ((:brake __brake) 0.0)
    ((:steering __steering) 0.0)
    ((:gear __gear) 0)
    ((:GolableEnable __GolableEnable) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _time __time)
   (setq _cycleno (round __cycleno))
   (setq _throttle (float __throttle))
   (setq _brake (float __brake))
   (setq _steering (float __steering))
   (setq _gear (round __gear))
   (setq _GolableEnable __GolableEnable)
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
  (:throttle
   (&optional __throttle)
   (if __throttle (setq _throttle __throttle)) _throttle)
  (:brake
   (&optional __brake)
   (if __brake (setq _brake __brake)) _brake)
  (:steering
   (&optional __steering)
   (if __steering (setq _steering __steering)) _steering)
  (:gear
   (&optional __gear)
   (if __gear (setq _gear __gear)) _gear)
  (:GolableEnable
   (&optional __GolableEnable)
   (if __GolableEnable (setq _GolableEnable __GolableEnable)) _GolableEnable)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; time _time
    8
    ;; int64 _cycleno
    8
    ;; float64 _throttle
    8
    ;; float64 _brake
    8
    ;; float64 _steering
    8
    ;; int64 _gear
    8
    ;; bool _GolableEnable
    1
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
     ;; float64 _throttle
       (sys::poke _throttle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _brake
       (sys::poke _brake (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _steering
       (sys::poke _steering (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int64 _gear
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _gear (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _gear) (= (length (_gear . bv)) 2)) ;; bignum
              (write-long (ash (elt (_gear . bv) 0) 0) s)
              (write-long (ash (elt (_gear . bv) 1) -1) s))
             ((and (class _gear) (= (length (_gear . bv)) 1)) ;; big1
              (write-long (elt (_gear . bv) 0) s)
              (write-long (if (>= _gear 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _gear s)(write-long (if (>= _gear 0) 0 #xffffffff) s)))
     ;; bool _GolableEnable
       (if _GolableEnable (write-byte -1 s) (write-byte 0 s))
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
   ;; float64 _throttle
     (setq _throttle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _brake
     (setq _brake (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _steering
     (setq _steering (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int64 _gear
#+(or :alpha :irix6 :x86_64)
      (setf _gear (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _gear (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; bool _GolableEnable
     (setq _GolableEnable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get hellocm_msgs::Ext2CM :md5sum-) "ce178e234de1fb8a500aea5e12e7372f")
(setf (get hellocm_msgs::Ext2CM :datatype-) "hellocm_msgs/Ext2CM")
(setf (get hellocm_msgs::Ext2CM :definition-)
      "#
# Message from external ROS Node to CarMaker ROS Node
#

# General
Header  header                                 # General ROS Header (optional)
time    time                                   # ROS time when message was sent (optional)
int64   cycleno                                # Cycle number since simulation start (optional)
float64 throttle
float64 brake
float64 steering
int64 gear
bool GolableEnable

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



(provide :hellocm_msgs/Ext2CM "ce178e234de1fb8a500aea5e12e7372f")


