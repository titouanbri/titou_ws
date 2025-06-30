// Auto-generated. Do not edit!

// (in-package ur_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Analog = require('../msg/Analog.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetAnalogOutputRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = new Analog();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetAnalogOutputRequest
    // Serialize message field [data]
    bufferOffset = Analog.serialize(obj.data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetAnalogOutputRequest
    let len;
    let data = new SetAnalogOutputRequest(null);
    // Deserialize message field [data]
    data.data = Analog.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 6;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur_msgs/SetAnalogOutputRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0ab594f795bd6245c7dfa69b09b68bca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Analog data
    
    ================================================================================
    MSG: ur_msgs/Analog
    uint8 CURRENT=0
    uint8 VOLTAGE=1
    
    uint8 pin
    uint8 domain # can be VOLTAGE or CURRENT
    float32 state
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetAnalogOutputRequest(null);
    if (msg.data !== undefined) {
      resolved.data = Analog.Resolve(msg.data)
    }
    else {
      resolved.data = new Analog()
    }

    return resolved;
    }
};

class SetAnalogOutputResponse {
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
    // Serializes a message object of type SetAnalogOutputResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetAnalogOutputResponse
    let len;
    let data = new SetAnalogOutputResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur_msgs/SetAnalogOutputResponse';
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
    const resolved = new SetAnalogOutputResponse(null);
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
  Request: SetAnalogOutputRequest,
  Response: SetAnalogOutputResponse,
  md5sum() { return 'eb3840e5f3632fc236409b92a9250f5b'; },
  datatype() { return 'ur_msgs/SetAnalogOutput'; }
};
