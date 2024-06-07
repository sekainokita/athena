// Auto-generated. Do not edit!

// (in-package xsens_mti_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class XsStatusWord {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.selftest = null;
      this.filter_valid = null;
      this.gnss_fix = null;
      this.no_rotation_update_status = null;
      this.representative_motion = null;
      this.clock_bias_estimation = null;
      this.clipflag_acc_x = null;
      this.clipflag_acc_y = null;
      this.clipflag_acc_z = null;
      this.clipflag_gyr_x = null;
      this.clipflag_gyr_y = null;
      this.clipflag_gyr_z = null;
      this.clipflag_mag_x = null;
      this.clipflag_mag_y = null;
      this.clipflag_mag_z = null;
      this.clipping_indication = null;
      this.syncin_marker = null;
      this.syncout_marker = null;
      this.filter_mode = null;
      this.have_gnss_time_pulse = null;
      this.rtk_status = null;
    }
    else {
      if (initObj.hasOwnProperty('selftest')) {
        this.selftest = initObj.selftest
      }
      else {
        this.selftest = false;
      }
      if (initObj.hasOwnProperty('filter_valid')) {
        this.filter_valid = initObj.filter_valid
      }
      else {
        this.filter_valid = false;
      }
      if (initObj.hasOwnProperty('gnss_fix')) {
        this.gnss_fix = initObj.gnss_fix
      }
      else {
        this.gnss_fix = false;
      }
      if (initObj.hasOwnProperty('no_rotation_update_status')) {
        this.no_rotation_update_status = initObj.no_rotation_update_status
      }
      else {
        this.no_rotation_update_status = 0;
      }
      if (initObj.hasOwnProperty('representative_motion')) {
        this.representative_motion = initObj.representative_motion
      }
      else {
        this.representative_motion = false;
      }
      if (initObj.hasOwnProperty('clock_bias_estimation')) {
        this.clock_bias_estimation = initObj.clock_bias_estimation
      }
      else {
        this.clock_bias_estimation = false;
      }
      if (initObj.hasOwnProperty('clipflag_acc_x')) {
        this.clipflag_acc_x = initObj.clipflag_acc_x
      }
      else {
        this.clipflag_acc_x = false;
      }
      if (initObj.hasOwnProperty('clipflag_acc_y')) {
        this.clipflag_acc_y = initObj.clipflag_acc_y
      }
      else {
        this.clipflag_acc_y = false;
      }
      if (initObj.hasOwnProperty('clipflag_acc_z')) {
        this.clipflag_acc_z = initObj.clipflag_acc_z
      }
      else {
        this.clipflag_acc_z = false;
      }
      if (initObj.hasOwnProperty('clipflag_gyr_x')) {
        this.clipflag_gyr_x = initObj.clipflag_gyr_x
      }
      else {
        this.clipflag_gyr_x = false;
      }
      if (initObj.hasOwnProperty('clipflag_gyr_y')) {
        this.clipflag_gyr_y = initObj.clipflag_gyr_y
      }
      else {
        this.clipflag_gyr_y = false;
      }
      if (initObj.hasOwnProperty('clipflag_gyr_z')) {
        this.clipflag_gyr_z = initObj.clipflag_gyr_z
      }
      else {
        this.clipflag_gyr_z = false;
      }
      if (initObj.hasOwnProperty('clipflag_mag_x')) {
        this.clipflag_mag_x = initObj.clipflag_mag_x
      }
      else {
        this.clipflag_mag_x = false;
      }
      if (initObj.hasOwnProperty('clipflag_mag_y')) {
        this.clipflag_mag_y = initObj.clipflag_mag_y
      }
      else {
        this.clipflag_mag_y = false;
      }
      if (initObj.hasOwnProperty('clipflag_mag_z')) {
        this.clipflag_mag_z = initObj.clipflag_mag_z
      }
      else {
        this.clipflag_mag_z = false;
      }
      if (initObj.hasOwnProperty('clipping_indication')) {
        this.clipping_indication = initObj.clipping_indication
      }
      else {
        this.clipping_indication = false;
      }
      if (initObj.hasOwnProperty('syncin_marker')) {
        this.syncin_marker = initObj.syncin_marker
      }
      else {
        this.syncin_marker = false;
      }
      if (initObj.hasOwnProperty('syncout_marker')) {
        this.syncout_marker = initObj.syncout_marker
      }
      else {
        this.syncout_marker = false;
      }
      if (initObj.hasOwnProperty('filter_mode')) {
        this.filter_mode = initObj.filter_mode
      }
      else {
        this.filter_mode = 0;
      }
      if (initObj.hasOwnProperty('have_gnss_time_pulse')) {
        this.have_gnss_time_pulse = initObj.have_gnss_time_pulse
      }
      else {
        this.have_gnss_time_pulse = false;
      }
      if (initObj.hasOwnProperty('rtk_status')) {
        this.rtk_status = initObj.rtk_status
      }
      else {
        this.rtk_status = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type XsStatusWord
    // Serialize message field [selftest]
    bufferOffset = _serializer.bool(obj.selftest, buffer, bufferOffset);
    // Serialize message field [filter_valid]
    bufferOffset = _serializer.bool(obj.filter_valid, buffer, bufferOffset);
    // Serialize message field [gnss_fix]
    bufferOffset = _serializer.bool(obj.gnss_fix, buffer, bufferOffset);
    // Serialize message field [no_rotation_update_status]
    bufferOffset = _serializer.uint8(obj.no_rotation_update_status, buffer, bufferOffset);
    // Serialize message field [representative_motion]
    bufferOffset = _serializer.bool(obj.representative_motion, buffer, bufferOffset);
    // Serialize message field [clock_bias_estimation]
    bufferOffset = _serializer.bool(obj.clock_bias_estimation, buffer, bufferOffset);
    // Serialize message field [clipflag_acc_x]
    bufferOffset = _serializer.bool(obj.clipflag_acc_x, buffer, bufferOffset);
    // Serialize message field [clipflag_acc_y]
    bufferOffset = _serializer.bool(obj.clipflag_acc_y, buffer, bufferOffset);
    // Serialize message field [clipflag_acc_z]
    bufferOffset = _serializer.bool(obj.clipflag_acc_z, buffer, bufferOffset);
    // Serialize message field [clipflag_gyr_x]
    bufferOffset = _serializer.bool(obj.clipflag_gyr_x, buffer, bufferOffset);
    // Serialize message field [clipflag_gyr_y]
    bufferOffset = _serializer.bool(obj.clipflag_gyr_y, buffer, bufferOffset);
    // Serialize message field [clipflag_gyr_z]
    bufferOffset = _serializer.bool(obj.clipflag_gyr_z, buffer, bufferOffset);
    // Serialize message field [clipflag_mag_x]
    bufferOffset = _serializer.bool(obj.clipflag_mag_x, buffer, bufferOffset);
    // Serialize message field [clipflag_mag_y]
    bufferOffset = _serializer.bool(obj.clipflag_mag_y, buffer, bufferOffset);
    // Serialize message field [clipflag_mag_z]
    bufferOffset = _serializer.bool(obj.clipflag_mag_z, buffer, bufferOffset);
    // Serialize message field [clipping_indication]
    bufferOffset = _serializer.bool(obj.clipping_indication, buffer, bufferOffset);
    // Serialize message field [syncin_marker]
    bufferOffset = _serializer.bool(obj.syncin_marker, buffer, bufferOffset);
    // Serialize message field [syncout_marker]
    bufferOffset = _serializer.bool(obj.syncout_marker, buffer, bufferOffset);
    // Serialize message field [filter_mode]
    bufferOffset = _serializer.uint8(obj.filter_mode, buffer, bufferOffset);
    // Serialize message field [have_gnss_time_pulse]
    bufferOffset = _serializer.bool(obj.have_gnss_time_pulse, buffer, bufferOffset);
    // Serialize message field [rtk_status]
    bufferOffset = _serializer.uint8(obj.rtk_status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type XsStatusWord
    let len;
    let data = new XsStatusWord(null);
    // Deserialize message field [selftest]
    data.selftest = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [filter_valid]
    data.filter_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gnss_fix]
    data.gnss_fix = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [no_rotation_update_status]
    data.no_rotation_update_status = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [representative_motion]
    data.representative_motion = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [clock_bias_estimation]
    data.clock_bias_estimation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [clipflag_acc_x]
    data.clipflag_acc_x = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [clipflag_acc_y]
    data.clipflag_acc_y = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [clipflag_acc_z]
    data.clipflag_acc_z = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [clipflag_gyr_x]
    data.clipflag_gyr_x = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [clipflag_gyr_y]
    data.clipflag_gyr_y = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [clipflag_gyr_z]
    data.clipflag_gyr_z = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [clipflag_mag_x]
    data.clipflag_mag_x = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [clipflag_mag_y]
    data.clipflag_mag_y = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [clipflag_mag_z]
    data.clipflag_mag_z = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [clipping_indication]
    data.clipping_indication = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [syncin_marker]
    data.syncin_marker = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [syncout_marker]
    data.syncout_marker = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [filter_mode]
    data.filter_mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [have_gnss_time_pulse]
    data.have_gnss_time_pulse = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [rtk_status]
    data.rtk_status = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'xsens_mti_driver/XsStatusWord';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dad684e003fb0f5d7e08711072d64f83';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Define the custom XsStatusWord message
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new XsStatusWord(null);
    if (msg.selftest !== undefined) {
      resolved.selftest = msg.selftest;
    }
    else {
      resolved.selftest = false
    }

    if (msg.filter_valid !== undefined) {
      resolved.filter_valid = msg.filter_valid;
    }
    else {
      resolved.filter_valid = false
    }

    if (msg.gnss_fix !== undefined) {
      resolved.gnss_fix = msg.gnss_fix;
    }
    else {
      resolved.gnss_fix = false
    }

    if (msg.no_rotation_update_status !== undefined) {
      resolved.no_rotation_update_status = msg.no_rotation_update_status;
    }
    else {
      resolved.no_rotation_update_status = 0
    }

    if (msg.representative_motion !== undefined) {
      resolved.representative_motion = msg.representative_motion;
    }
    else {
      resolved.representative_motion = false
    }

    if (msg.clock_bias_estimation !== undefined) {
      resolved.clock_bias_estimation = msg.clock_bias_estimation;
    }
    else {
      resolved.clock_bias_estimation = false
    }

    if (msg.clipflag_acc_x !== undefined) {
      resolved.clipflag_acc_x = msg.clipflag_acc_x;
    }
    else {
      resolved.clipflag_acc_x = false
    }

    if (msg.clipflag_acc_y !== undefined) {
      resolved.clipflag_acc_y = msg.clipflag_acc_y;
    }
    else {
      resolved.clipflag_acc_y = false
    }

    if (msg.clipflag_acc_z !== undefined) {
      resolved.clipflag_acc_z = msg.clipflag_acc_z;
    }
    else {
      resolved.clipflag_acc_z = false
    }

    if (msg.clipflag_gyr_x !== undefined) {
      resolved.clipflag_gyr_x = msg.clipflag_gyr_x;
    }
    else {
      resolved.clipflag_gyr_x = false
    }

    if (msg.clipflag_gyr_y !== undefined) {
      resolved.clipflag_gyr_y = msg.clipflag_gyr_y;
    }
    else {
      resolved.clipflag_gyr_y = false
    }

    if (msg.clipflag_gyr_z !== undefined) {
      resolved.clipflag_gyr_z = msg.clipflag_gyr_z;
    }
    else {
      resolved.clipflag_gyr_z = false
    }

    if (msg.clipflag_mag_x !== undefined) {
      resolved.clipflag_mag_x = msg.clipflag_mag_x;
    }
    else {
      resolved.clipflag_mag_x = false
    }

    if (msg.clipflag_mag_y !== undefined) {
      resolved.clipflag_mag_y = msg.clipflag_mag_y;
    }
    else {
      resolved.clipflag_mag_y = false
    }

    if (msg.clipflag_mag_z !== undefined) {
      resolved.clipflag_mag_z = msg.clipflag_mag_z;
    }
    else {
      resolved.clipflag_mag_z = false
    }

    if (msg.clipping_indication !== undefined) {
      resolved.clipping_indication = msg.clipping_indication;
    }
    else {
      resolved.clipping_indication = false
    }

    if (msg.syncin_marker !== undefined) {
      resolved.syncin_marker = msg.syncin_marker;
    }
    else {
      resolved.syncin_marker = false
    }

    if (msg.syncout_marker !== undefined) {
      resolved.syncout_marker = msg.syncout_marker;
    }
    else {
      resolved.syncout_marker = false
    }

    if (msg.filter_mode !== undefined) {
      resolved.filter_mode = msg.filter_mode;
    }
    else {
      resolved.filter_mode = 0
    }

    if (msg.have_gnss_time_pulse !== undefined) {
      resolved.have_gnss_time_pulse = msg.have_gnss_time_pulse;
    }
    else {
      resolved.have_gnss_time_pulse = false
    }

    if (msg.rtk_status !== undefined) {
      resolved.rtk_status = msg.rtk_status;
    }
    else {
      resolved.rtk_status = 0
    }

    return resolved;
    }
};

module.exports = XsStatusWord;
