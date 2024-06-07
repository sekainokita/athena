; Auto-generated. Do not edit!


(cl:in-package xsens_mti_driver-msg)


;//! \htmlinclude XsStatusWord.msg.html

(cl:defclass <XsStatusWord> (roslisp-msg-protocol:ros-message)
  ((selftest
    :reader selftest
    :initarg :selftest
    :type cl:boolean
    :initform cl:nil)
   (filter_valid
    :reader filter_valid
    :initarg :filter_valid
    :type cl:boolean
    :initform cl:nil)
   (gnss_fix
    :reader gnss_fix
    :initarg :gnss_fix
    :type cl:boolean
    :initform cl:nil)
   (no_rotation_update_status
    :reader no_rotation_update_status
    :initarg :no_rotation_update_status
    :type cl:fixnum
    :initform 0)
   (representative_motion
    :reader representative_motion
    :initarg :representative_motion
    :type cl:boolean
    :initform cl:nil)
   (clock_bias_estimation
    :reader clock_bias_estimation
    :initarg :clock_bias_estimation
    :type cl:boolean
    :initform cl:nil)
   (clipflag_acc_x
    :reader clipflag_acc_x
    :initarg :clipflag_acc_x
    :type cl:boolean
    :initform cl:nil)
   (clipflag_acc_y
    :reader clipflag_acc_y
    :initarg :clipflag_acc_y
    :type cl:boolean
    :initform cl:nil)
   (clipflag_acc_z
    :reader clipflag_acc_z
    :initarg :clipflag_acc_z
    :type cl:boolean
    :initform cl:nil)
   (clipflag_gyr_x
    :reader clipflag_gyr_x
    :initarg :clipflag_gyr_x
    :type cl:boolean
    :initform cl:nil)
   (clipflag_gyr_y
    :reader clipflag_gyr_y
    :initarg :clipflag_gyr_y
    :type cl:boolean
    :initform cl:nil)
   (clipflag_gyr_z
    :reader clipflag_gyr_z
    :initarg :clipflag_gyr_z
    :type cl:boolean
    :initform cl:nil)
   (clipflag_mag_x
    :reader clipflag_mag_x
    :initarg :clipflag_mag_x
    :type cl:boolean
    :initform cl:nil)
   (clipflag_mag_y
    :reader clipflag_mag_y
    :initarg :clipflag_mag_y
    :type cl:boolean
    :initform cl:nil)
   (clipflag_mag_z
    :reader clipflag_mag_z
    :initarg :clipflag_mag_z
    :type cl:boolean
    :initform cl:nil)
   (clipping_indication
    :reader clipping_indication
    :initarg :clipping_indication
    :type cl:boolean
    :initform cl:nil)
   (syncin_marker
    :reader syncin_marker
    :initarg :syncin_marker
    :type cl:boolean
    :initform cl:nil)
   (syncout_marker
    :reader syncout_marker
    :initarg :syncout_marker
    :type cl:boolean
    :initform cl:nil)
   (filter_mode
    :reader filter_mode
    :initarg :filter_mode
    :type cl:fixnum
    :initform 0)
   (have_gnss_time_pulse
    :reader have_gnss_time_pulse
    :initarg :have_gnss_time_pulse
    :type cl:boolean
    :initform cl:nil)
   (rtk_status
    :reader rtk_status
    :initarg :rtk_status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass XsStatusWord (<XsStatusWord>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <XsStatusWord>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'XsStatusWord)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xsens_mti_driver-msg:<XsStatusWord> is deprecated: use xsens_mti_driver-msg:XsStatusWord instead.")))

(cl:ensure-generic-function 'selftest-val :lambda-list '(m))
(cl:defmethod selftest-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:selftest-val is deprecated.  Use xsens_mti_driver-msg:selftest instead.")
  (selftest m))

(cl:ensure-generic-function 'filter_valid-val :lambda-list '(m))
(cl:defmethod filter_valid-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:filter_valid-val is deprecated.  Use xsens_mti_driver-msg:filter_valid instead.")
  (filter_valid m))

(cl:ensure-generic-function 'gnss_fix-val :lambda-list '(m))
(cl:defmethod gnss_fix-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:gnss_fix-val is deprecated.  Use xsens_mti_driver-msg:gnss_fix instead.")
  (gnss_fix m))

(cl:ensure-generic-function 'no_rotation_update_status-val :lambda-list '(m))
(cl:defmethod no_rotation_update_status-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:no_rotation_update_status-val is deprecated.  Use xsens_mti_driver-msg:no_rotation_update_status instead.")
  (no_rotation_update_status m))

(cl:ensure-generic-function 'representative_motion-val :lambda-list '(m))
(cl:defmethod representative_motion-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:representative_motion-val is deprecated.  Use xsens_mti_driver-msg:representative_motion instead.")
  (representative_motion m))

(cl:ensure-generic-function 'clock_bias_estimation-val :lambda-list '(m))
(cl:defmethod clock_bias_estimation-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:clock_bias_estimation-val is deprecated.  Use xsens_mti_driver-msg:clock_bias_estimation instead.")
  (clock_bias_estimation m))

(cl:ensure-generic-function 'clipflag_acc_x-val :lambda-list '(m))
(cl:defmethod clipflag_acc_x-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:clipflag_acc_x-val is deprecated.  Use xsens_mti_driver-msg:clipflag_acc_x instead.")
  (clipflag_acc_x m))

(cl:ensure-generic-function 'clipflag_acc_y-val :lambda-list '(m))
(cl:defmethod clipflag_acc_y-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:clipflag_acc_y-val is deprecated.  Use xsens_mti_driver-msg:clipflag_acc_y instead.")
  (clipflag_acc_y m))

(cl:ensure-generic-function 'clipflag_acc_z-val :lambda-list '(m))
(cl:defmethod clipflag_acc_z-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:clipflag_acc_z-val is deprecated.  Use xsens_mti_driver-msg:clipflag_acc_z instead.")
  (clipflag_acc_z m))

(cl:ensure-generic-function 'clipflag_gyr_x-val :lambda-list '(m))
(cl:defmethod clipflag_gyr_x-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:clipflag_gyr_x-val is deprecated.  Use xsens_mti_driver-msg:clipflag_gyr_x instead.")
  (clipflag_gyr_x m))

(cl:ensure-generic-function 'clipflag_gyr_y-val :lambda-list '(m))
(cl:defmethod clipflag_gyr_y-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:clipflag_gyr_y-val is deprecated.  Use xsens_mti_driver-msg:clipflag_gyr_y instead.")
  (clipflag_gyr_y m))

(cl:ensure-generic-function 'clipflag_gyr_z-val :lambda-list '(m))
(cl:defmethod clipflag_gyr_z-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:clipflag_gyr_z-val is deprecated.  Use xsens_mti_driver-msg:clipflag_gyr_z instead.")
  (clipflag_gyr_z m))

(cl:ensure-generic-function 'clipflag_mag_x-val :lambda-list '(m))
(cl:defmethod clipflag_mag_x-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:clipflag_mag_x-val is deprecated.  Use xsens_mti_driver-msg:clipflag_mag_x instead.")
  (clipflag_mag_x m))

(cl:ensure-generic-function 'clipflag_mag_y-val :lambda-list '(m))
(cl:defmethod clipflag_mag_y-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:clipflag_mag_y-val is deprecated.  Use xsens_mti_driver-msg:clipflag_mag_y instead.")
  (clipflag_mag_y m))

(cl:ensure-generic-function 'clipflag_mag_z-val :lambda-list '(m))
(cl:defmethod clipflag_mag_z-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:clipflag_mag_z-val is deprecated.  Use xsens_mti_driver-msg:clipflag_mag_z instead.")
  (clipflag_mag_z m))

(cl:ensure-generic-function 'clipping_indication-val :lambda-list '(m))
(cl:defmethod clipping_indication-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:clipping_indication-val is deprecated.  Use xsens_mti_driver-msg:clipping_indication instead.")
  (clipping_indication m))

(cl:ensure-generic-function 'syncin_marker-val :lambda-list '(m))
(cl:defmethod syncin_marker-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:syncin_marker-val is deprecated.  Use xsens_mti_driver-msg:syncin_marker instead.")
  (syncin_marker m))

(cl:ensure-generic-function 'syncout_marker-val :lambda-list '(m))
(cl:defmethod syncout_marker-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:syncout_marker-val is deprecated.  Use xsens_mti_driver-msg:syncout_marker instead.")
  (syncout_marker m))

(cl:ensure-generic-function 'filter_mode-val :lambda-list '(m))
(cl:defmethod filter_mode-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:filter_mode-val is deprecated.  Use xsens_mti_driver-msg:filter_mode instead.")
  (filter_mode m))

(cl:ensure-generic-function 'have_gnss_time_pulse-val :lambda-list '(m))
(cl:defmethod have_gnss_time_pulse-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:have_gnss_time_pulse-val is deprecated.  Use xsens_mti_driver-msg:have_gnss_time_pulse instead.")
  (have_gnss_time_pulse m))

(cl:ensure-generic-function 'rtk_status-val :lambda-list '(m))
(cl:defmethod rtk_status-val ((m <XsStatusWord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens_mti_driver-msg:rtk_status-val is deprecated.  Use xsens_mti_driver-msg:rtk_status instead.")
  (rtk_status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <XsStatusWord>) ostream)
  "Serializes a message object of type '<XsStatusWord>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'selftest) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'filter_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'gnss_fix) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'no_rotation_update_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'representative_motion) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clock_bias_estimation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clipflag_acc_x) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clipflag_acc_y) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clipflag_acc_z) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clipflag_gyr_x) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clipflag_gyr_y) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clipflag_gyr_z) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clipflag_mag_x) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clipflag_mag_y) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clipflag_mag_z) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clipping_indication) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'syncin_marker) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'syncout_marker) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'filter_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'have_gnss_time_pulse) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rtk_status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <XsStatusWord>) istream)
  "Deserializes a message object of type '<XsStatusWord>"
    (cl:setf (cl:slot-value msg 'selftest) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'filter_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'gnss_fix) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'no_rotation_update_status)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'representative_motion) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'clock_bias_estimation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'clipflag_acc_x) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'clipflag_acc_y) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'clipflag_acc_z) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'clipflag_gyr_x) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'clipflag_gyr_y) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'clipflag_gyr_z) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'clipflag_mag_x) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'clipflag_mag_y) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'clipflag_mag_z) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'clipping_indication) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'syncin_marker) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'syncout_marker) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'filter_mode)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'have_gnss_time_pulse) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rtk_status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<XsStatusWord>)))
  "Returns string type for a message object of type '<XsStatusWord>"
  "xsens_mti_driver/XsStatusWord")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'XsStatusWord)))
  "Returns string type for a message object of type 'XsStatusWord"
  "xsens_mti_driver/XsStatusWord")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<XsStatusWord>)))
  "Returns md5sum for a message object of type '<XsStatusWord>"
  "dad684e003fb0f5d7e08711072d64f83")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'XsStatusWord)))
  "Returns md5sum for a message object of type 'XsStatusWord"
  "dad684e003fb0f5d7e08711072d64f83")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<XsStatusWord>)))
  "Returns full string definition for message of type '<XsStatusWord>"
  (cl:format cl:nil "# Define the custom XsStatusWord message~%bool selftest~%bool filter_valid~%bool gnss_fix~%uint8 no_rotation_update_status ##0: not running, 2: aborted, 3:running~%bool representative_motion~%bool clock_bias_estimation~%#bool reserved1~%bool clipflag_acc_x~%bool clipflag_acc_y~%bool clipflag_acc_z~%bool clipflag_gyr_x~%bool clipflag_gyr_y~%bool clipflag_gyr_z~%bool clipflag_mag_x~%bool clipflag_mag_y~%bool clipflag_mag_z~%#uint8 reserved2~%bool clipping_indication~%#bool reserved3~%bool syncin_marker~%bool syncout_marker~%uint8 filter_mode ##0: Without GNSS, 1: Coasting, 3: With GNSS~%bool have_gnss_time_pulse~%uint8 rtk_status ##0: No RTK, 1: RTK Floating, 2: RTK Fix~%#uint8 reserved4~%~%########################~%####ref to MT Low Level Protocol Communication: https://mtidocs.xsens.com/messages~%####1) Bit 0: Selftest, This flag indicates if the MT passed the self-test according to eMTS(electronic Motion Tracker Specification).~%####2) Bit 1: Filter Valid, This flag indicates if input into the orientation filter is reliable and / or complete.~%####3) Bit 2: GNSS fix, This flag indicates if the GNSS unit has a proper fix. ~%####4) Bit 3:4: NoRotationUpdate Status, This flag indicates the status of the no rotation update procedure in the filter after the SetNoRotation message has been sent. 11: Running with no rotation assumption; 10: Rotation detected, no gyro bias estimation (sticky); 00: Estimation complete, no errors.~%####5) Bit 5: Representative Motion (RepMo), Indicates if the MTi is in In-run Compass Calibration Representative Mode~%####6) Bit 6: Clock Bias Estimation (ClockSync), Indicates that the Clock Bias Estimation synchronization feature is active~%####7) Bit 7: Reserved, Reserved for future use~%####8) Bit 8: Clipflag Acc X, If set, an out of range acceleration on the X axis is detected~%####9) Bit 9: Clipflag Acc Y, If set, an out of range acceleration on the Y axis is detected~%####10) Bit 10: Clipflag Acc Z, If set, an out of range acceleration on the Z axis is detected~%####11) Bit 11: Clipflag Gyr X, If set, an out of range angular velocity on the X axis is detected~%####12) Bit 12: Clipflag Gyr Y, If set, an out of range angular velocity on the Y axis is detected~%####13) Bit 13: Clipflag Gyr Z, If set, an out of range angular velocity on the Z axis is detected~%####14) Bit 14: Clipflag Mag X, If set, an out of range magnetic field on the X axis is detected~%####15) Bit 15: Clipflag Mag Y, If set, an out of range magnetic field on the Y axis is detected~%####16) Bit 16: Clipflag Mag Z, If set, an out of range magnetic field on the Z axis is detected~%####17) Bit 17:18, Reserved, Reserved for future use~%####18) Bit 19, Clipping Indication, This flag indicates going out of range of one of the sensors (it is set when one or more bits from 8:16 are set)~%####19) Bit 20,  Reserved, Reserved for future use~%####20) Bit 21, SyncIn Marker, When a SyncIn is detected, this bit will rise to 1. ~%####21) Bit 22, SyncOut Marker, When SyncOut is active, this bit will rise to 1.~%####22) Bit 23:25, Filter Mode, Indicates Filter Mode, currently only available for GNSS/INS devices:000: Without GNSS (filter profile is in VRU mode); 001: Coasting mode (GNSS has been lost <60 sec ago);011: With GNSS (default mode)~%####23) Bit 26, HaveGnssTimePulse, Indicates that the 1PPS GNSS time pulse is present~%####24) 27:28, RtkStatus, Indicates the availability and status of RTK: 00: No RTK; 01: RTK floating; 10: RTK fixed~%####25) 29:31, Reserved, Reserved for future use~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'XsStatusWord)))
  "Returns full string definition for message of type 'XsStatusWord"
  (cl:format cl:nil "# Define the custom XsStatusWord message~%bool selftest~%bool filter_valid~%bool gnss_fix~%uint8 no_rotation_update_status ##0: not running, 2: aborted, 3:running~%bool representative_motion~%bool clock_bias_estimation~%#bool reserved1~%bool clipflag_acc_x~%bool clipflag_acc_y~%bool clipflag_acc_z~%bool clipflag_gyr_x~%bool clipflag_gyr_y~%bool clipflag_gyr_z~%bool clipflag_mag_x~%bool clipflag_mag_y~%bool clipflag_mag_z~%#uint8 reserved2~%bool clipping_indication~%#bool reserved3~%bool syncin_marker~%bool syncout_marker~%uint8 filter_mode ##0: Without GNSS, 1: Coasting, 3: With GNSS~%bool have_gnss_time_pulse~%uint8 rtk_status ##0: No RTK, 1: RTK Floating, 2: RTK Fix~%#uint8 reserved4~%~%########################~%####ref to MT Low Level Protocol Communication: https://mtidocs.xsens.com/messages~%####1) Bit 0: Selftest, This flag indicates if the MT passed the self-test according to eMTS(electronic Motion Tracker Specification).~%####2) Bit 1: Filter Valid, This flag indicates if input into the orientation filter is reliable and / or complete.~%####3) Bit 2: GNSS fix, This flag indicates if the GNSS unit has a proper fix. ~%####4) Bit 3:4: NoRotationUpdate Status, This flag indicates the status of the no rotation update procedure in the filter after the SetNoRotation message has been sent. 11: Running with no rotation assumption; 10: Rotation detected, no gyro bias estimation (sticky); 00: Estimation complete, no errors.~%####5) Bit 5: Representative Motion (RepMo), Indicates if the MTi is in In-run Compass Calibration Representative Mode~%####6) Bit 6: Clock Bias Estimation (ClockSync), Indicates that the Clock Bias Estimation synchronization feature is active~%####7) Bit 7: Reserved, Reserved for future use~%####8) Bit 8: Clipflag Acc X, If set, an out of range acceleration on the X axis is detected~%####9) Bit 9: Clipflag Acc Y, If set, an out of range acceleration on the Y axis is detected~%####10) Bit 10: Clipflag Acc Z, If set, an out of range acceleration on the Z axis is detected~%####11) Bit 11: Clipflag Gyr X, If set, an out of range angular velocity on the X axis is detected~%####12) Bit 12: Clipflag Gyr Y, If set, an out of range angular velocity on the Y axis is detected~%####13) Bit 13: Clipflag Gyr Z, If set, an out of range angular velocity on the Z axis is detected~%####14) Bit 14: Clipflag Mag X, If set, an out of range magnetic field on the X axis is detected~%####15) Bit 15: Clipflag Mag Y, If set, an out of range magnetic field on the Y axis is detected~%####16) Bit 16: Clipflag Mag Z, If set, an out of range magnetic field on the Z axis is detected~%####17) Bit 17:18, Reserved, Reserved for future use~%####18) Bit 19, Clipping Indication, This flag indicates going out of range of one of the sensors (it is set when one or more bits from 8:16 are set)~%####19) Bit 20,  Reserved, Reserved for future use~%####20) Bit 21, SyncIn Marker, When a SyncIn is detected, this bit will rise to 1. ~%####21) Bit 22, SyncOut Marker, When SyncOut is active, this bit will rise to 1.~%####22) Bit 23:25, Filter Mode, Indicates Filter Mode, currently only available for GNSS/INS devices:000: Without GNSS (filter profile is in VRU mode); 001: Coasting mode (GNSS has been lost <60 sec ago);011: With GNSS (default mode)~%####23) Bit 26, HaveGnssTimePulse, Indicates that the 1PPS GNSS time pulse is present~%####24) 27:28, RtkStatus, Indicates the availability and status of RTK: 00: No RTK; 01: RTK floating; 10: RTK fixed~%####25) 29:31, Reserved, Reserved for future use~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <XsStatusWord>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <XsStatusWord>))
  "Converts a ROS message object to a list"
  (cl:list 'XsStatusWord
    (cl:cons ':selftest (selftest msg))
    (cl:cons ':filter_valid (filter_valid msg))
    (cl:cons ':gnss_fix (gnss_fix msg))
    (cl:cons ':no_rotation_update_status (no_rotation_update_status msg))
    (cl:cons ':representative_motion (representative_motion msg))
    (cl:cons ':clock_bias_estimation (clock_bias_estimation msg))
    (cl:cons ':clipflag_acc_x (clipflag_acc_x msg))
    (cl:cons ':clipflag_acc_y (clipflag_acc_y msg))
    (cl:cons ':clipflag_acc_z (clipflag_acc_z msg))
    (cl:cons ':clipflag_gyr_x (clipflag_gyr_x msg))
    (cl:cons ':clipflag_gyr_y (clipflag_gyr_y msg))
    (cl:cons ':clipflag_gyr_z (clipflag_gyr_z msg))
    (cl:cons ':clipflag_mag_x (clipflag_mag_x msg))
    (cl:cons ':clipflag_mag_y (clipflag_mag_y msg))
    (cl:cons ':clipflag_mag_z (clipflag_mag_z msg))
    (cl:cons ':clipping_indication (clipping_indication msg))
    (cl:cons ':syncin_marker (syncin_marker msg))
    (cl:cons ':syncout_marker (syncout_marker msg))
    (cl:cons ':filter_mode (filter_mode msg))
    (cl:cons ':have_gnss_time_pulse (have_gnss_time_pulse msg))
    (cl:cons ':rtk_status (rtk_status msg))
))
