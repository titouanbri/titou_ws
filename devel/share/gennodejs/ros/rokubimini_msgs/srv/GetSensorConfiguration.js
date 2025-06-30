// Auto-generated. Do not edit!

// (in-package rokubimini_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetSensorConfigurationRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.a = null;
    }
    else {
      if (initObj.hasOwnProperty('a')) {
        this.a = initObj.a
      }
      else {
        this.a = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetSensorConfigurationRequest
    // Serialize message field [a]
    bufferOffset = _serializer.bool(obj.a, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetSensorConfigurationRequest
    let len;
    let data = new GetSensorConfigurationRequest(null);
    // Deserialize message field [a]
    data.a = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rokubimini_msgs/GetSensorConfigurationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '685b546c53fb0b7de5dfef48cf30fe1f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool a
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetSensorConfigurationRequest(null);
    if (msg.a !== undefined) {
      resolved.a = msg.a;
    }
    else {
      resolved.a = false
    }

    return resolved;
    }
};

class GetSensorConfigurationResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.b = null;
    }
    else {
      if (initObj.hasOwnProperty('b')) {
        this.b = initObj.b
      }
      else {
        this.b = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetSensorConfigurationResponse
    // Serialize message field [b]
    bufferOffset = _serializer.bool(obj.b, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetSensorConfigurationResponse
    let len;
    let data = new GetSensorConfigurationResponse(null);
    // Deserialize message field [b]
    data.b = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rokubimini_msgs/GetSensorConfigurationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '88c93a4e354c9b18b18fde29f72f94c2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    bool b
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetSensorConfigurationResponse(null);
    if (msg.b !== undefined) {
      resolved.b = msg.b;
    }
    else {
      resolved.b = false
    }

    return resolved;
    }
};

module.exports = {
  Request: GetSensorConfigurationRequest,
  Response: GetSensorConfigurationResponse,
  md5sum() { return '81f01bfd9a951b1adf9102125874ff5b'; },
  datatype() { return 'rokubimini_msgs/GetSensorConfiguration'; }
};
