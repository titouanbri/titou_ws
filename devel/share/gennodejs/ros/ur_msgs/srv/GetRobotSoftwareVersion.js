// Auto-generated. Do not edit!

// (in-package ur_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetRobotSoftwareVersionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetRobotSoftwareVersionRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetRobotSoftwareVersionRequest
    let len;
    let data = new GetRobotSoftwareVersionRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur_msgs/GetRobotSoftwareVersionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetRobotSoftwareVersionRequest(null);
    return resolved;
    }
};

class GetRobotSoftwareVersionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.major = null;
      this.minor = null;
      this.bugfix = null;
      this.build = null;
    }
    else {
      if (initObj.hasOwnProperty('major')) {
        this.major = initObj.major
      }
      else {
        this.major = 0;
      }
      if (initObj.hasOwnProperty('minor')) {
        this.minor = initObj.minor
      }
      else {
        this.minor = 0;
      }
      if (initObj.hasOwnProperty('bugfix')) {
        this.bugfix = initObj.bugfix
      }
      else {
        this.bugfix = 0;
      }
      if (initObj.hasOwnProperty('build')) {
        this.build = initObj.build
      }
      else {
        this.build = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetRobotSoftwareVersionResponse
    // Serialize message field [major]
    bufferOffset = _serializer.uint32(obj.major, buffer, bufferOffset);
    // Serialize message field [minor]
    bufferOffset = _serializer.uint32(obj.minor, buffer, bufferOffset);
    // Serialize message field [bugfix]
    bufferOffset = _serializer.uint32(obj.bugfix, buffer, bufferOffset);
    // Serialize message field [build]
    bufferOffset = _serializer.uint32(obj.build, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetRobotSoftwareVersionResponse
    let len;
    let data = new GetRobotSoftwareVersionResponse(null);
    // Deserialize message field [major]
    data.major = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [minor]
    data.minor = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [bugfix]
    data.bugfix = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [build]
    data.build = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur_msgs/GetRobotSoftwareVersionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'eea0f1664f7955042558cb2bf2c766ad';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 major    # Major version number
    uint32 minor    # Minor version number
    uint32 bugfix   # Bugfix version number
    uint32 build    # Build number
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetRobotSoftwareVersionResponse(null);
    if (msg.major !== undefined) {
      resolved.major = msg.major;
    }
    else {
      resolved.major = 0
    }

    if (msg.minor !== undefined) {
      resolved.minor = msg.minor;
    }
    else {
      resolved.minor = 0
    }

    if (msg.bugfix !== undefined) {
      resolved.bugfix = msg.bugfix;
    }
    else {
      resolved.bugfix = 0
    }

    if (msg.build !== undefined) {
      resolved.build = msg.build;
    }
    else {
      resolved.build = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: GetRobotSoftwareVersionRequest,
  Response: GetRobotSoftwareVersionResponse,
  md5sum() { return 'eea0f1664f7955042558cb2bf2c766ad'; },
  datatype() { return 'ur_msgs/GetRobotSoftwareVersion'; }
};
