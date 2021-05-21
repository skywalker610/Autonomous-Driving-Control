; Auto-generated. Do not edit!


(cl:in-package dbw-msg)


;//! \htmlinclude SteerCmd.msg.html

(cl:defclass <SteerCmd> (roslisp-msg-protocol:ros-message)
  ((steering_wheel_angle_cmd
    :reader steering_wheel_angle_cmd
    :initarg :steering_wheel_angle_cmd
    :type cl:float
    :initform 0.0)
   (steering_wheel_angle_velocity
    :reader steering_wheel_angle_velocity
    :initarg :steering_wheel_angle_velocity
    :type cl:float
    :initform 0.0)
   (steering_wheel_torque_cmd
    :reader steering_wheel_torque_cmd
    :initarg :steering_wheel_torque_cmd
    :type cl:float
    :initform 0.0)
   (cmd_type
    :reader cmd_type
    :initarg :cmd_type
    :type cl:fixnum
    :initform 0)
   (enable
    :reader enable
    :initarg :enable
    :type cl:boolean
    :initform cl:nil)
   (clear
    :reader clear
    :initarg :clear
    :type cl:boolean
    :initform cl:nil)
   (ignore
    :reader ignore
    :initarg :ignore
    :type cl:boolean
    :initform cl:nil)
   (quiet
    :reader quiet
    :initarg :quiet
    :type cl:boolean
    :initform cl:nil)
   (count
    :reader count
    :initarg :count
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SteerCmd (<SteerCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SteerCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SteerCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dbw-msg:<SteerCmd> is deprecated: use dbw-msg:SteerCmd instead.")))

(cl:ensure-generic-function 'steering_wheel_angle_cmd-val :lambda-list '(m))
(cl:defmethod steering_wheel_angle_cmd-val ((m <SteerCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dbw-msg:steering_wheel_angle_cmd-val is deprecated.  Use dbw-msg:steering_wheel_angle_cmd instead.")
  (steering_wheel_angle_cmd m))

(cl:ensure-generic-function 'steering_wheel_angle_velocity-val :lambda-list '(m))
(cl:defmethod steering_wheel_angle_velocity-val ((m <SteerCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dbw-msg:steering_wheel_angle_velocity-val is deprecated.  Use dbw-msg:steering_wheel_angle_velocity instead.")
  (steering_wheel_angle_velocity m))

(cl:ensure-generic-function 'steering_wheel_torque_cmd-val :lambda-list '(m))
(cl:defmethod steering_wheel_torque_cmd-val ((m <SteerCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dbw-msg:steering_wheel_torque_cmd-val is deprecated.  Use dbw-msg:steering_wheel_torque_cmd instead.")
  (steering_wheel_torque_cmd m))

(cl:ensure-generic-function 'cmd_type-val :lambda-list '(m))
(cl:defmethod cmd_type-val ((m <SteerCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dbw-msg:cmd_type-val is deprecated.  Use dbw-msg:cmd_type instead.")
  (cmd_type m))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <SteerCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dbw-msg:enable-val is deprecated.  Use dbw-msg:enable instead.")
  (enable m))

(cl:ensure-generic-function 'clear-val :lambda-list '(m))
(cl:defmethod clear-val ((m <SteerCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dbw-msg:clear-val is deprecated.  Use dbw-msg:clear instead.")
  (clear m))

(cl:ensure-generic-function 'ignore-val :lambda-list '(m))
(cl:defmethod ignore-val ((m <SteerCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dbw-msg:ignore-val is deprecated.  Use dbw-msg:ignore instead.")
  (ignore m))

(cl:ensure-generic-function 'quiet-val :lambda-list '(m))
(cl:defmethod quiet-val ((m <SteerCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dbw-msg:quiet-val is deprecated.  Use dbw-msg:quiet instead.")
  (quiet m))

(cl:ensure-generic-function 'count-val :lambda-list '(m))
(cl:defmethod count-val ((m <SteerCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dbw-msg:count-val is deprecated.  Use dbw-msg:count instead.")
  (count m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SteerCmd>)))
    "Constants for message type '<SteerCmd>"
  '((:CMD_ANGLE . 0)
    (:CMD_TORQUE . 1)
    (:ANGLE_MAX . 9.6)
    (:VELOCITY_MAX . 17.5)
    (:TORQUE_MAX . 8.0))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SteerCmd)))
    "Constants for message type 'SteerCmd"
  '((:CMD_ANGLE . 0)
    (:CMD_TORQUE . 1)
    (:ANGLE_MAX . 9.6)
    (:VELOCITY_MAX . 17.5)
    (:TORQUE_MAX . 8.0))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SteerCmd>) ostream)
  "Serializes a message object of type '<SteerCmd>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steering_wheel_angle_cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steering_wheel_angle_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steering_wheel_torque_cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clear) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ignore) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'quiet) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'count)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SteerCmd>) istream)
  "Deserializes a message object of type '<SteerCmd>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering_wheel_angle_cmd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering_wheel_angle_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering_wheel_torque_cmd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd_type)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'enable) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'clear) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ignore) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'quiet) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'count)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SteerCmd>)))
  "Returns string type for a message object of type '<SteerCmd>"
  "dbw/SteerCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SteerCmd)))
  "Returns string type for a message object of type 'SteerCmd"
  "dbw/SteerCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SteerCmd>)))
  "Returns md5sum for a message object of type '<SteerCmd>"
  "076be04e947ae855b3617e6190e078b9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SteerCmd)))
  "Returns md5sum for a message object of type 'SteerCmd"
  "076be04e947ae855b3617e6190e078b9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SteerCmd>)))
  "Returns full string definition for message of type '<SteerCmd>"
  (cl:format cl:nil "# Steering Wheel~%float32 steering_wheel_angle_cmd        # rad, range -9.6 to 9.6~%float32 steering_wheel_angle_velocity   # rad/s, range 0 to 17.5, 0 = maximum~%float32 steering_wheel_torque_cmd       # Nm, range -8.0 to 8.0~%uint8 cmd_type~%~%# Enable~%bool enable~%~%# Clear driver overrides~%bool clear~%~%# Ignore driver overrides~%bool ignore~%~%# Disable the driver override audible warning~%bool quiet~%~%# Watchdog counter (optional)~%uint8 count~%~%# Command types~%uint8 CMD_ANGLE=0~%uint8 CMD_TORQUE=1~%~%# Maximum values~%float32 ANGLE_MAX=9.6     # rad, maximum angle~%float32 VELOCITY_MAX=17.5 # rad/s, maximum velocity~%float32 TORQUE_MAX=8.0    # Nm, maximum torque~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SteerCmd)))
  "Returns full string definition for message of type 'SteerCmd"
  (cl:format cl:nil "# Steering Wheel~%float32 steering_wheel_angle_cmd        # rad, range -9.6 to 9.6~%float32 steering_wheel_angle_velocity   # rad/s, range 0 to 17.5, 0 = maximum~%float32 steering_wheel_torque_cmd       # Nm, range -8.0 to 8.0~%uint8 cmd_type~%~%# Enable~%bool enable~%~%# Clear driver overrides~%bool clear~%~%# Ignore driver overrides~%bool ignore~%~%# Disable the driver override audible warning~%bool quiet~%~%# Watchdog counter (optional)~%uint8 count~%~%# Command types~%uint8 CMD_ANGLE=0~%uint8 CMD_TORQUE=1~%~%# Maximum values~%float32 ANGLE_MAX=9.6     # rad, maximum angle~%float32 VELOCITY_MAX=17.5 # rad/s, maximum velocity~%float32 TORQUE_MAX=8.0    # Nm, maximum torque~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SteerCmd>))
  (cl:+ 0
     4
     4
     4
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SteerCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'SteerCmd
    (cl:cons ':steering_wheel_angle_cmd (steering_wheel_angle_cmd msg))
    (cl:cons ':steering_wheel_angle_velocity (steering_wheel_angle_velocity msg))
    (cl:cons ':steering_wheel_torque_cmd (steering_wheel_torque_cmd msg))
    (cl:cons ':cmd_type (cmd_type msg))
    (cl:cons ':enable (enable msg))
    (cl:cons ':clear (clear msg))
    (cl:cons ':ignore (ignore msg))
    (cl:cons ':quiet (quiet msg))
    (cl:cons ':count (count msg))
))
