// Auto-generated. Do not edit!

// (in-package ur_msgs.srv)


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

class SetForceModeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.task_frame = null;
      this.selection_vector_x = null;
      this.selection_vector_y = null;
      this.selection_vector_z = null;
      this.selection_vector_rx = null;
      this.selection_vector_ry = null;
      this.selection_vector_rz = null;
      this.wrench = null;
      this.type = null;
      this.speed_limits = null;
      this.deviation_limits = null;
      this.damping_factor = null;
      this.gain_scaling = null;
    }
    else {
      if (initObj.hasOwnProperty('task_frame')) {
        this.task_frame = initObj.task_frame
      }
      else {
        this.task_frame = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('selection_vector_x')) {
        this.selection_vector_x = initObj.selection_vector_x
      }
      else {
        this.selection_vector_x = false;
      }
      if (initObj.hasOwnProperty('selection_vector_y')) {
        this.selection_vector_y = initObj.selection_vector_y
      }
      else {
        this.selection_vector_y = false;
      }
      if (initObj.hasOwnProperty('selection_vector_z')) {
        this.selection_vector_z = initObj.selection_vector_z
      }
      else {
        this.selection_vector_z = false;
      }
      if (initObj.hasOwnProperty('selection_vector_rx')) {
        this.selection_vector_rx = initObj.selection_vector_rx
      }
      else {
        this.selection_vector_rx = false;
      }
      if (initObj.hasOwnProperty('selection_vector_ry')) {
        this.selection_vector_ry = initObj.selection_vector_ry
      }
      else {
        this.selection_vector_ry = false;
      }
      if (initObj.hasOwnProperty('selection_vector_rz')) {
        this.selection_vector_rz = initObj.selection_vector_rz
      }
      else {
        this.selection_vector_rz = false;
      }
      if (initObj.hasOwnProperty('wrench')) {
        this.wrench = initObj.wrench
      }
      else {
        this.wrench = new geometry_msgs.msg.Wrench();
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('speed_limits')) {
        this.speed_limits = initObj.speed_limits
      }
      else {
        this.speed_limits = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('deviation_limits')) {
        this.deviation_limits = initObj.deviation_limits
      }
      else {
        this.deviation_limits = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('damping_factor')) {
        this.damping_factor = initObj.damping_factor
      }
      else {
        this.damping_factor = 0.0;
      }
      if (initObj.hasOwnProperty('gain_scaling')) {
        this.gain_scaling = initObj.gain_scaling
      }
      else {
        this.gain_scaling = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetForceModeRequest
    // Serialize message field [task_frame]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.task_frame, buffer, bufferOffset);
    // Serialize message field [selection_vector_x]
    bufferOffset = _serializer.bool(obj.selection_vector_x, buffer, bufferOffset);
    // Serialize message field [selection_vector_y]
    bufferOffset = _serializer.bool(obj.selection_vector_y, buffer, bufferOffset);
    // Serialize message field [selection_vector_z]
    bufferOffset = _serializer.bool(obj.selection_vector_z, buffer, bufferOffset);
    // Serialize message field [selection_vector_rx]
    bufferOffset = _serializer.bool(obj.selection_vector_rx, buffer, bufferOffset);
    // Serialize message field [selection_vector_ry]
    bufferOffset = _serializer.bool(obj.selection_vector_ry, buffer, bufferOffset);
    // Serialize message field [selection_vector_rz]
    bufferOffset = _serializer.bool(obj.selection_vector_rz, buffer, bufferOffset);
    // Serialize message field [wrench]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.wrench, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.uint8(obj.type, buffer, bufferOffset);
    // Serialize message field [speed_limits]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.speed_limits, buffer, bufferOffset);
    // Check that the constant length array field [deviation_limits] has the right length
    if (obj.deviation_limits.length !== 6) {
      throw new Error('Unable to serialize array field deviation_limits - length must be 6')
    }
    // Serialize message field [deviation_limits]
    bufferOffset = _arraySerializer.float32(obj.deviation_limits, buffer, bufferOffset, 6);
    // Serialize message field [damping_factor]
    bufferOffset = _serializer.float32(obj.damping_factor, buffer, bufferOffset);
    // Serialize message field [gain_scaling]
    bufferOffset = _serializer.float32(obj.gain_scaling, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetForceModeRequest
    let len;
    let data = new SetForceModeRequest(null);
    // Deserialize message field [task_frame]
    data.task_frame = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [selection_vector_x]
    data.selection_vector_x = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [selection_vector_y]
    data.selection_vector_y = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [selection_vector_z]
    data.selection_vector_z = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [selection_vector_rx]
    data.selection_vector_rx = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [selection_vector_ry]
    data.selection_vector_ry = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [selection_vector_rz]
    data.selection_vector_rz = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [wrench]
    data.wrench = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [speed_limits]
    data.speed_limits = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [deviation_limits]
    data.deviation_limits = _arrayDeserializer.float32(buffer, bufferOffset, 6)
    // Deserialize message field [damping_factor]
    data.damping_factor = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gain_scaling]
    data.gain_scaling = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.task_frame);
    return length + 135;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur_msgs/SetForceModeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '04cfdc672416a6eb3ccbbcc81eeaa8a5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # A 6d pose vector that defines the force frame relative to the base frame
    geometry_msgs/PoseStamped task_frame        
    
    # A 6d vector of 0s and 1s. 1 means that the robot will be compliant in the corresponding axis of the task frame
    bool selection_vector_x
    bool selection_vector_y
    bool selection_vector_z
    bool selection_vector_rx
    bool selection_vector_ry
    bool selection_vector_rz  
    
    # The forces/torques the robot will apply to its environment. For geometric interpretation, please
    # see parameter `type`
    geometry_msgs/Wrench wrench
    
    # An integer [1;3] specifying how the robot interprets the force frame
    # 1: The force frame is transformed in a way such that its y-axis is aligned with a vector pointing
    #    from the robot tcp towards the origin of the force frame.
    # 2: The force frame is not transformed.
    # 3: The force frame is transformed in a way such that its x-axis is the projection of the robot tcp
    #     velocity vector onto the x-y plane of the force frame.
    uint8 type
    # Type constants:
    uint8 TCP_TO_ORIGIN=1
    uint8 NO_TRANSFORM=2
    uint8 TCP_VELOCITY_TO_X_Y=3
    
    # Maximum allowed tcp speed (relative to the task frame).
    # PLEASE NOTE: This is only relevant for axes marked as compliant in the selection_vector
    geometry_msgs/Twist speed_limits
    
    # For non-compliant axes, these values are the maximum allowed deviation along/about an axis
    # between the actual tcp position and the one set by the program.
    float32[6] deviation_limits
    
    # Force mode damping factor. Sets the damping parameter in force mode. In range [0;1], default value is 0.025
    # A value of 1 is full damping, so the robot will decelerate quickly if no force is present. A value of 0
    # is no damping, here the robot will maintain the speed.
    float32 damping_factor
    
    # Force mode gain scaling factor. Scales the gain in force mode. scaling parameter is in range [0;2], default is 0.5. 
    # A value larger than 1 can make force mode unstable, e.g. in case of collisions or pushing against hard surfaces.
    float32 gain_scaling
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
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
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
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
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3 linear
    Vector3 angular
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetForceModeRequest(null);
    if (msg.task_frame !== undefined) {
      resolved.task_frame = geometry_msgs.msg.PoseStamped.Resolve(msg.task_frame)
    }
    else {
      resolved.task_frame = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.selection_vector_x !== undefined) {
      resolved.selection_vector_x = msg.selection_vector_x;
    }
    else {
      resolved.selection_vector_x = false
    }

    if (msg.selection_vector_y !== undefined) {
      resolved.selection_vector_y = msg.selection_vector_y;
    }
    else {
      resolved.selection_vector_y = false
    }

    if (msg.selection_vector_z !== undefined) {
      resolved.selection_vector_z = msg.selection_vector_z;
    }
    else {
      resolved.selection_vector_z = false
    }

    if (msg.selection_vector_rx !== undefined) {
      resolved.selection_vector_rx = msg.selection_vector_rx;
    }
    else {
      resolved.selection_vector_rx = false
    }

    if (msg.selection_vector_ry !== undefined) {
      resolved.selection_vector_ry = msg.selection_vector_ry;
    }
    else {
      resolved.selection_vector_ry = false
    }

    if (msg.selection_vector_rz !== undefined) {
      resolved.selection_vector_rz = msg.selection_vector_rz;
    }
    else {
      resolved.selection_vector_rz = false
    }

    if (msg.wrench !== undefined) {
      resolved.wrench = geometry_msgs.msg.Wrench.Resolve(msg.wrench)
    }
    else {
      resolved.wrench = new geometry_msgs.msg.Wrench()
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.speed_limits !== undefined) {
      resolved.speed_limits = geometry_msgs.msg.Twist.Resolve(msg.speed_limits)
    }
    else {
      resolved.speed_limits = new geometry_msgs.msg.Twist()
    }

    if (msg.deviation_limits !== undefined) {
      resolved.deviation_limits = msg.deviation_limits;
    }
    else {
      resolved.deviation_limits = new Array(6).fill(0)
    }

    if (msg.damping_factor !== undefined) {
      resolved.damping_factor = msg.damping_factor;
    }
    else {
      resolved.damping_factor = 0.0
    }

    if (msg.gain_scaling !== undefined) {
      resolved.gain_scaling = msg.gain_scaling;
    }
    else {
      resolved.gain_scaling = 0.0
    }

    return resolved;
    }
};

// Constants for message
SetForceModeRequest.Constants = {
  TCP_TO_ORIGIN: 1,
  NO_TRANSFORM: 2,
  TCP_VELOCITY_TO_X_Y: 3,
}

class SetForceModeResponse {
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
    // Serializes a message object of type SetForceModeResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetForceModeResponse
    let len;
    let data = new SetForceModeResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur_msgs/SetForceModeResponse';
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
    const resolved = new SetForceModeResponse(null);
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
  Request: SetForceModeRequest,
  Response: SetForceModeResponse,
  md5sum() { return 'ad1bcbfccfedd053ec6bb7eb365913d8'; },
  datatype() { return 'ur_msgs/SetForceMode'; }
};
