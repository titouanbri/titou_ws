// Auto-generated. Do not edit!

// (in-package rokubimini_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class ResetWrenchRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.desired_wrench = null;
    }
    else {
      if (initObj.hasOwnProperty('desired_wrench')) {
        this.desired_wrench = initObj.desired_wrench
      }
      else {
        this.desired_wrench = new geometry_msgs.msg.Wrench();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResetWrenchRequest
    // Serialize message field [desired_wrench]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.desired_wrench, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResetWrenchRequest
    let len;
    let data = new ResetWrenchRequest(null);
    // Deserialize message field [desired_wrench]
    data.desired_wrench = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rokubimini_msgs/ResetWrenchRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c17e2a03e9bfbf4880fb44b7cbb9269b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Wrench desired_wrench
    
    ================================================================================
    MSG: geometry_msgs/Wrench
    # This represents force in free space, separated into
    # its linear and angular parts.
    Vector3  force
    Vector3  torque
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ResetWrenchRequest(null);
    if (msg.desired_wrench !== undefined) {
      resolved.desired_wrench = geometry_msgs.msg.Wrench.Resolve(msg.desired_wrench)
    }
    else {
      resolved.desired_wrench = new geometry_msgs.msg.Wrench()
    }

    return resolved;
    }
};

class ResetWrenchResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResetWrenchResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResetWrenchResponse
    let len;
    let data = new ResetWrenchResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rokubimini_msgs/ResetWrenchResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ResetWrenchResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: ResetWrenchRequest,
  Response: ResetWrenchResponse,
  md5sum() { return 'a01e1ece25b40c645838e2e60d7f441c'; },
  datatype() { return 'rokubimini_msgs/ResetWrench'; }
};
