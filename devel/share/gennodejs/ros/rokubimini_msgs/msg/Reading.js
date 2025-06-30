// Auto-generated. Do not edit!

// (in-package rokubimini_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Reading {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.statusword = null;
      this.imu = null;
      this.wrench = null;
      this.externalImu = null;
      this.isForceTorqueSaturated = null;
      this.temperature = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('statusword')) {
        this.statusword = initObj.statusword
      }
      else {
        this.statusword = 0;
      }
      if (initObj.hasOwnProperty('imu')) {
        this.imu = initObj.imu
      }
      else {
        this.imu = new sensor_msgs.msg.Imu();
      }
      if (initObj.hasOwnProperty('wrench')) {
        this.wrench = initObj.wrench
      }
      else {
        this.wrench = new geometry_msgs.msg.WrenchStamped();
      }
      if (initObj.hasOwnProperty('externalImu')) {
        this.externalImu = initObj.externalImu
      }
      else {
        this.externalImu = new sensor_msgs.msg.Imu();
      }
      if (initObj.hasOwnProperty('isForceTorqueSaturated')) {
        this.isForceTorqueSaturated = initObj.isForceTorqueSaturated
      }
      else {
        this.isForceTorqueSaturated = false;
      }
      if (initObj.hasOwnProperty('temperature')) {
        this.temperature = initObj.temperature
      }
      else {
        this.temperature = new sensor_msgs.msg.Temperature();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Reading
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [statusword]
    bufferOffset = _serializer.uint32(obj.statusword, buffer, bufferOffset);
    // Serialize message field [imu]
    bufferOffset = sensor_msgs.msg.Imu.serialize(obj.imu, buffer, bufferOffset);
    // Serialize message field [wrench]
    bufferOffset = geometry_msgs.msg.WrenchStamped.serialize(obj.wrench, buffer, bufferOffset);
    // Serialize message field [externalImu]
    bufferOffset = sensor_msgs.msg.Imu.serialize(obj.externalImu, buffer, bufferOffset);
    // Serialize message field [isForceTorqueSaturated]
    bufferOffset = _serializer.bool(obj.isForceTorqueSaturated, buffer, bufferOffset);
    // Serialize message field [temperature]
    bufferOffset = sensor_msgs.msg.Temperature.serialize(obj.temperature, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Reading
    let len;
    let data = new Reading(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [statusword]
    data.statusword = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [imu]
    data.imu = sensor_msgs.msg.Imu.deserialize(buffer, bufferOffset);
    // Deserialize message field [wrench]
    data.wrench = geometry_msgs.msg.WrenchStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [externalImu]
    data.externalImu = sensor_msgs.msg.Imu.deserialize(buffer, bufferOffset);
    // Deserialize message field [isForceTorqueSaturated]
    data.isForceTorqueSaturated = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [temperature]
    data.temperature = sensor_msgs.msg.Temperature.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += sensor_msgs.msg.Imu.getMessageSize(object.imu);
    length += geometry_msgs.msg.WrenchStamped.getMessageSize(object.wrench);
    length += sensor_msgs.msg.Imu.getMessageSize(object.externalImu);
    length += sensor_msgs.msg.Temperature.getMessageSize(object.temperature);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rokubimini_msgs/Reading';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f632d7286fac45ac13e8083c2bf7f237';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Reading
    
    # Message header.
    Header header
    # Statusword.
    uint32 statusword
    sensor_msgs/Imu imu
    geometry_msgs/WrenchStamped wrench
    sensor_msgs/Imu externalImu
    bool isForceTorqueSaturated
    sensor_msgs/Temperature temperature
    
    
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
    
    ================================================================================
    MSG: sensor_msgs/Imu
    # This is a message to hold data from an IMU (Inertial Measurement Unit)
    #
    # Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
    #
    # If the covariance of the measurement is known, it should be filled in (if all you know is the 
    # variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
    # A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
    # data a covariance will have to be assumed or gotten from some other source
    #
    # If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
    # estimate), please set element 0 of the associated covariance matrix to -1
    # If you are interpreting this message, please check for a value of -1 in the first element of each 
    # covariance matrix, and disregard the associated estimate.
    
    Header header
    
    geometry_msgs/Quaternion orientation
    float64[9] orientation_covariance # Row major about x, y, z axes
    
    geometry_msgs/Vector3 angular_velocity
    float64[9] angular_velocity_covariance # Row major about x, y, z axes
    
    geometry_msgs/Vector3 linear_acceleration
    float64[9] linear_acceleration_covariance # Row major x, y z 
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
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
    ================================================================================
    MSG: geometry_msgs/WrenchStamped
    # A wrench with reference coordinate frame and timestamp
    Header header
    Wrench wrench
    
    ================================================================================
    MSG: geometry_msgs/Wrench
    # This represents force in free space, separated into
    # its linear and angular parts.
    Vector3  force
    Vector3  torque
    
    ================================================================================
    MSG: sensor_msgs/Temperature
    # Single temperature reading.
    
    Header header           # timestamp is the time the temperature was measured
                             # frame_id is the location of the temperature reading
    
    float64 temperature     # Measurement of the Temperature in Degrees Celsius
    
    float64 variance        # 0 is interpreted as variance unknown
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Reading(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.statusword !== undefined) {
      resolved.statusword = msg.statusword;
    }
    else {
      resolved.statusword = 0
    }

    if (msg.imu !== undefined) {
      resolved.imu = sensor_msgs.msg.Imu.Resolve(msg.imu)
    }
    else {
      resolved.imu = new sensor_msgs.msg.Imu()
    }

    if (msg.wrench !== undefined) {
      resolved.wrench = geometry_msgs.msg.WrenchStamped.Resolve(msg.wrench)
    }
    else {
      resolved.wrench = new geometry_msgs.msg.WrenchStamped()
    }

    if (msg.externalImu !== undefined) {
      resolved.externalImu = sensor_msgs.msg.Imu.Resolve(msg.externalImu)
    }
    else {
      resolved.externalImu = new sensor_msgs.msg.Imu()
    }

    if (msg.isForceTorqueSaturated !== undefined) {
      resolved.isForceTorqueSaturated = msg.isForceTorqueSaturated;
    }
    else {
      resolved.isForceTorqueSaturated = false
    }

    if (msg.temperature !== undefined) {
      resolved.temperature = sensor_msgs.msg.Temperature.Resolve(msg.temperature)
    }
    else {
      resolved.temperature = new sensor_msgs.msg.Temperature()
    }

    return resolved;
    }
};

module.exports = Reading;
