;; Auto-generated. Do not edit!


(when (boundp 'xsens_mti_driver::XsStatusWord)
  (if (not (find-package "XSENS_MTI_DRIVER"))
    (make-package "XSENS_MTI_DRIVER"))
  (shadow 'XsStatusWord (find-package "XSENS_MTI_DRIVER")))
(unless (find-package "XSENS_MTI_DRIVER::XSSTATUSWORD")
  (make-package "XSENS_MTI_DRIVER::XSSTATUSWORD"))

(in-package "ROS")
;;//! \htmlinclude XsStatusWord.msg.html


(defclass xsens_mti_driver::XsStatusWord
  :super ros::object
  :slots (_selftest _filter_valid _gnss_fix _no_rotation_update_status _representative_motion _clock_bias_estimation _clipflag_acc_x _clipflag_acc_y _clipflag_acc_z _clipflag_gyr_x _clipflag_gyr_y _clipflag_gyr_z _clipflag_mag_x _clipflag_mag_y _clipflag_mag_z _clipping_indication _syncin_marker _syncout_marker _filter_mode _have_gnss_time_pulse _rtk_status ))

(defmethod xsens_mti_driver::XsStatusWord
  (:init
   (&key
    ((:selftest __selftest) nil)
    ((:filter_valid __filter_valid) nil)
    ((:gnss_fix __gnss_fix) nil)
    ((:no_rotation_update_status __no_rotation_update_status) 0)
    ((:representative_motion __representative_motion) nil)
    ((:clock_bias_estimation __clock_bias_estimation) nil)
    ((:clipflag_acc_x __clipflag_acc_x) nil)
    ((:clipflag_acc_y __clipflag_acc_y) nil)
    ((:clipflag_acc_z __clipflag_acc_z) nil)
    ((:clipflag_gyr_x __clipflag_gyr_x) nil)
    ((:clipflag_gyr_y __clipflag_gyr_y) nil)
    ((:clipflag_gyr_z __clipflag_gyr_z) nil)
    ((:clipflag_mag_x __clipflag_mag_x) nil)
    ((:clipflag_mag_y __clipflag_mag_y) nil)
    ((:clipflag_mag_z __clipflag_mag_z) nil)
    ((:clipping_indication __clipping_indication) nil)
    ((:syncin_marker __syncin_marker) nil)
    ((:syncout_marker __syncout_marker) nil)
    ((:filter_mode __filter_mode) 0)
    ((:have_gnss_time_pulse __have_gnss_time_pulse) nil)
    ((:rtk_status __rtk_status) 0)
    )
   (send-super :init)
   (setq _selftest __selftest)
   (setq _filter_valid __filter_valid)
   (setq _gnss_fix __gnss_fix)
   (setq _no_rotation_update_status (round __no_rotation_update_status))
   (setq _representative_motion __representative_motion)
   (setq _clock_bias_estimation __clock_bias_estimation)
   (setq _clipflag_acc_x __clipflag_acc_x)
   (setq _clipflag_acc_y __clipflag_acc_y)
   (setq _clipflag_acc_z __clipflag_acc_z)
   (setq _clipflag_gyr_x __clipflag_gyr_x)
   (setq _clipflag_gyr_y __clipflag_gyr_y)
   (setq _clipflag_gyr_z __clipflag_gyr_z)
   (setq _clipflag_mag_x __clipflag_mag_x)
   (setq _clipflag_mag_y __clipflag_mag_y)
   (setq _clipflag_mag_z __clipflag_mag_z)
   (setq _clipping_indication __clipping_indication)
   (setq _syncin_marker __syncin_marker)
   (setq _syncout_marker __syncout_marker)
   (setq _filter_mode (round __filter_mode))
   (setq _have_gnss_time_pulse __have_gnss_time_pulse)
   (setq _rtk_status (round __rtk_status))
   self)
  (:selftest
   (&optional (__selftest :null))
   (if (not (eq __selftest :null)) (setq _selftest __selftest)) _selftest)
  (:filter_valid
   (&optional (__filter_valid :null))
   (if (not (eq __filter_valid :null)) (setq _filter_valid __filter_valid)) _filter_valid)
  (:gnss_fix
   (&optional (__gnss_fix :null))
   (if (not (eq __gnss_fix :null)) (setq _gnss_fix __gnss_fix)) _gnss_fix)
  (:no_rotation_update_status
   (&optional __no_rotation_update_status)
   (if __no_rotation_update_status (setq _no_rotation_update_status __no_rotation_update_status)) _no_rotation_update_status)
  (:representative_motion
   (&optional (__representative_motion :null))
   (if (not (eq __representative_motion :null)) (setq _representative_motion __representative_motion)) _representative_motion)
  (:clock_bias_estimation
   (&optional (__clock_bias_estimation :null))
   (if (not (eq __clock_bias_estimation :null)) (setq _clock_bias_estimation __clock_bias_estimation)) _clock_bias_estimation)
  (:clipflag_acc_x
   (&optional (__clipflag_acc_x :null))
   (if (not (eq __clipflag_acc_x :null)) (setq _clipflag_acc_x __clipflag_acc_x)) _clipflag_acc_x)
  (:clipflag_acc_y
   (&optional (__clipflag_acc_y :null))
   (if (not (eq __clipflag_acc_y :null)) (setq _clipflag_acc_y __clipflag_acc_y)) _clipflag_acc_y)
  (:clipflag_acc_z
   (&optional (__clipflag_acc_z :null))
   (if (not (eq __clipflag_acc_z :null)) (setq _clipflag_acc_z __clipflag_acc_z)) _clipflag_acc_z)
  (:clipflag_gyr_x
   (&optional (__clipflag_gyr_x :null))
   (if (not (eq __clipflag_gyr_x :null)) (setq _clipflag_gyr_x __clipflag_gyr_x)) _clipflag_gyr_x)
  (:clipflag_gyr_y
   (&optional (__clipflag_gyr_y :null))
   (if (not (eq __clipflag_gyr_y :null)) (setq _clipflag_gyr_y __clipflag_gyr_y)) _clipflag_gyr_y)
  (:clipflag_gyr_z
   (&optional (__clipflag_gyr_z :null))
   (if (not (eq __clipflag_gyr_z :null)) (setq _clipflag_gyr_z __clipflag_gyr_z)) _clipflag_gyr_z)
  (:clipflag_mag_x
   (&optional (__clipflag_mag_x :null))
   (if (not (eq __clipflag_mag_x :null)) (setq _clipflag_mag_x __clipflag_mag_x)) _clipflag_mag_x)
  (:clipflag_mag_y
   (&optional (__clipflag_mag_y :null))
   (if (not (eq __clipflag_mag_y :null)) (setq _clipflag_mag_y __clipflag_mag_y)) _clipflag_mag_y)
  (:clipflag_mag_z
   (&optional (__clipflag_mag_z :null))
   (if (not (eq __clipflag_mag_z :null)) (setq _clipflag_mag_z __clipflag_mag_z)) _clipflag_mag_z)
  (:clipping_indication
   (&optional (__clipping_indication :null))
   (if (not (eq __clipping_indication :null)) (setq _clipping_indication __clipping_indication)) _clipping_indication)
  (:syncin_marker
   (&optional (__syncin_marker :null))
   (if (not (eq __syncin_marker :null)) (setq _syncin_marker __syncin_marker)) _syncin_marker)
  (:syncout_marker
   (&optional (__syncout_marker :null))
   (if (not (eq __syncout_marker :null)) (setq _syncout_marker __syncout_marker)) _syncout_marker)
  (:filter_mode
   (&optional __filter_mode)
   (if __filter_mode (setq _filter_mode __filter_mode)) _filter_mode)
  (:have_gnss_time_pulse
   (&optional (__have_gnss_time_pulse :null))
   (if (not (eq __have_gnss_time_pulse :null)) (setq _have_gnss_time_pulse __have_gnss_time_pulse)) _have_gnss_time_pulse)
  (:rtk_status
   (&optional __rtk_status)
   (if __rtk_status (setq _rtk_status __rtk_status)) _rtk_status)
  (:serialization-length
   ()
   (+
    ;; bool _selftest
    1
    ;; bool _filter_valid
    1
    ;; bool _gnss_fix
    1
    ;; uint8 _no_rotation_update_status
    1
    ;; bool _representative_motion
    1
    ;; bool _clock_bias_estimation
    1
    ;; bool _clipflag_acc_x
    1
    ;; bool _clipflag_acc_y
    1
    ;; bool _clipflag_acc_z
    1
    ;; bool _clipflag_gyr_x
    1
    ;; bool _clipflag_gyr_y
    1
    ;; bool _clipflag_gyr_z
    1
    ;; bool _clipflag_mag_x
    1
    ;; bool _clipflag_mag_y
    1
    ;; bool _clipflag_mag_z
    1
    ;; bool _clipping_indication
    1
    ;; bool _syncin_marker
    1
    ;; bool _syncout_marker
    1
    ;; uint8 _filter_mode
    1
    ;; bool _have_gnss_time_pulse
    1
    ;; uint8 _rtk_status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _selftest
       (if _selftest (write-byte -1 s) (write-byte 0 s))
     ;; bool _filter_valid
       (if _filter_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _gnss_fix
       (if _gnss_fix (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _no_rotation_update_status
       (write-byte _no_rotation_update_status s)
     ;; bool _representative_motion
       (if _representative_motion (write-byte -1 s) (write-byte 0 s))
     ;; bool _clock_bias_estimation
       (if _clock_bias_estimation (write-byte -1 s) (write-byte 0 s))
     ;; bool _clipflag_acc_x
       (if _clipflag_acc_x (write-byte -1 s) (write-byte 0 s))
     ;; bool _clipflag_acc_y
       (if _clipflag_acc_y (write-byte -1 s) (write-byte 0 s))
     ;; bool _clipflag_acc_z
       (if _clipflag_acc_z (write-byte -1 s) (write-byte 0 s))
     ;; bool _clipflag_gyr_x
       (if _clipflag_gyr_x (write-byte -1 s) (write-byte 0 s))
     ;; bool _clipflag_gyr_y
       (if _clipflag_gyr_y (write-byte -1 s) (write-byte 0 s))
     ;; bool _clipflag_gyr_z
       (if _clipflag_gyr_z (write-byte -1 s) (write-byte 0 s))
     ;; bool _clipflag_mag_x
       (if _clipflag_mag_x (write-byte -1 s) (write-byte 0 s))
     ;; bool _clipflag_mag_y
       (if _clipflag_mag_y (write-byte -1 s) (write-byte 0 s))
     ;; bool _clipflag_mag_z
       (if _clipflag_mag_z (write-byte -1 s) (write-byte 0 s))
     ;; bool _clipping_indication
       (if _clipping_indication (write-byte -1 s) (write-byte 0 s))
     ;; bool _syncin_marker
       (if _syncin_marker (write-byte -1 s) (write-byte 0 s))
     ;; bool _syncout_marker
       (if _syncout_marker (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _filter_mode
       (write-byte _filter_mode s)
     ;; bool _have_gnss_time_pulse
       (if _have_gnss_time_pulse (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _rtk_status
       (write-byte _rtk_status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _selftest
     (setq _selftest (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _filter_valid
     (setq _filter_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _gnss_fix
     (setq _gnss_fix (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _no_rotation_update_status
     (setq _no_rotation_update_status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _representative_motion
     (setq _representative_motion (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _clock_bias_estimation
     (setq _clock_bias_estimation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _clipflag_acc_x
     (setq _clipflag_acc_x (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _clipflag_acc_y
     (setq _clipflag_acc_y (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _clipflag_acc_z
     (setq _clipflag_acc_z (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _clipflag_gyr_x
     (setq _clipflag_gyr_x (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _clipflag_gyr_y
     (setq _clipflag_gyr_y (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _clipflag_gyr_z
     (setq _clipflag_gyr_z (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _clipflag_mag_x
     (setq _clipflag_mag_x (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _clipflag_mag_y
     (setq _clipflag_mag_y (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _clipflag_mag_z
     (setq _clipflag_mag_z (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _clipping_indication
     (setq _clipping_indication (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _syncin_marker
     (setq _syncin_marker (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _syncout_marker
     (setq _syncout_marker (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _filter_mode
     (setq _filter_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _have_gnss_time_pulse
     (setq _have_gnss_time_pulse (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _rtk_status
     (setq _rtk_status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get xsens_mti_driver::XsStatusWord :md5sum-) "dad684e003fb0f5d7e08711072d64f83")
(setf (get xsens_mti_driver::XsStatusWord :datatype-) "xsens_mti_driver/XsStatusWord")
(setf (get xsens_mti_driver::XsStatusWord :definition-)
      "# Define the custom XsStatusWord message
bool selftest
bool filter_valid
bool gnss_fix
uint8 no_rotation_update_status ##0: not running, 2: aborted, 3:running
bool representative_motion
bool clock_bias_estimation
#bool reserved1
bool clipflag_acc_x
bool clipflag_acc_y
bool clipflag_acc_z
bool clipflag_gyr_x
bool clipflag_gyr_y
bool clipflag_gyr_z
bool clipflag_mag_x
bool clipflag_mag_y
bool clipflag_mag_z
#uint8 reserved2
bool clipping_indication
#bool reserved3
bool syncin_marker
bool syncout_marker
uint8 filter_mode ##0: Without GNSS, 1: Coasting, 3: With GNSS
bool have_gnss_time_pulse
uint8 rtk_status ##0: No RTK, 1: RTK Floating, 2: RTK Fix
#uint8 reserved4

########################
####ref to MT Low Level Protocol Communication: https://mtidocs.xsens.com/messages
####1) Bit 0: Selftest, This flag indicates if the MT passed the self-test according to eMTS(electronic Motion Tracker Specification).
####2) Bit 1: Filter Valid, This flag indicates if input into the orientation filter is reliable and / or complete.
####3) Bit 2: GNSS fix, This flag indicates if the GNSS unit has a proper fix. 
####4) Bit 3:4: NoRotationUpdate Status, This flag indicates the status of the no rotation update procedure in the filter after the SetNoRotation message has been sent. 11: Running with no rotation assumption; 10: Rotation detected, no gyro bias estimation (sticky); 00: Estimation complete, no errors.
####5) Bit 5: Representative Motion (RepMo), Indicates if the MTi is in In-run Compass Calibration Representative Mode
####6) Bit 6: Clock Bias Estimation (ClockSync), Indicates that the Clock Bias Estimation synchronization feature is active
####7) Bit 7: Reserved, Reserved for future use
####8) Bit 8: Clipflag Acc X, If set, an out of range acceleration on the X axis is detected
####9) Bit 9: Clipflag Acc Y, If set, an out of range acceleration on the Y axis is detected
####10) Bit 10: Clipflag Acc Z, If set, an out of range acceleration on the Z axis is detected
####11) Bit 11: Clipflag Gyr X, If set, an out of range angular velocity on the X axis is detected
####12) Bit 12: Clipflag Gyr Y, If set, an out of range angular velocity on the Y axis is detected
####13) Bit 13: Clipflag Gyr Z, If set, an out of range angular velocity on the Z axis is detected
####14) Bit 14: Clipflag Mag X, If set, an out of range magnetic field on the X axis is detected
####15) Bit 15: Clipflag Mag Y, If set, an out of range magnetic field on the Y axis is detected
####16) Bit 16: Clipflag Mag Z, If set, an out of range magnetic field on the Z axis is detected
####17) Bit 17:18, Reserved, Reserved for future use
####18) Bit 19, Clipping Indication, This flag indicates going out of range of one of the sensors (it is set when one or more bits from 8:16 are set)
####19) Bit 20,  Reserved, Reserved for future use
####20) Bit 21, SyncIn Marker, When a SyncIn is detected, this bit will rise to 1. 
####21) Bit 22, SyncOut Marker, When SyncOut is active, this bit will rise to 1.
####22) Bit 23:25, Filter Mode, Indicates Filter Mode, currently only available for GNSS/INS devices:000: Without GNSS (filter profile is in VRU mode); 001: Coasting mode (GNSS has been lost <60 sec ago);011: With GNSS (default mode)
####23) Bit 26, HaveGnssTimePulse, Indicates that the 1PPS GNSS time pulse is present
####24) 27:28, RtkStatus, Indicates the availability and status of RTK: 00: No RTK; 01: RTK floating; 10: RTK fixed
####25) 29:31, Reserved, Reserved for future use

")



(provide :xsens_mti_driver/XsStatusWord "dad684e003fb0f5d7e08711072d64f83")


