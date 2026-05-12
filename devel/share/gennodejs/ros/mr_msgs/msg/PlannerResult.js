// Auto-generated. Do not edit!

// (in-package mr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PlannerResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.algorithm = null;
      this.path_length = null;
      this.planning_time = null;
      this.num_waypoints = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('algorithm')) {
        this.algorithm = initObj.algorithm
      }
      else {
        this.algorithm = '';
      }
      if (initObj.hasOwnProperty('path_length')) {
        this.path_length = initObj.path_length
      }
      else {
        this.path_length = 0.0;
      }
      if (initObj.hasOwnProperty('planning_time')) {
        this.planning_time = initObj.planning_time
      }
      else {
        this.planning_time = 0.0;
      }
      if (initObj.hasOwnProperty('num_waypoints')) {
        this.num_waypoints = initObj.num_waypoints
      }
      else {
        this.num_waypoints = 0;
      }
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlannerResult
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [algorithm]
    bufferOffset = _serializer.string(obj.algorithm, buffer, bufferOffset);
    // Serialize message field [path_length]
    bufferOffset = _serializer.float64(obj.path_length, buffer, bufferOffset);
    // Serialize message field [planning_time]
    bufferOffset = _serializer.float64(obj.planning_time, buffer, bufferOffset);
    // Serialize message field [num_waypoints]
    bufferOffset = _serializer.int32(obj.num_waypoints, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlannerResult
    let len;
    let data = new PlannerResult(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [algorithm]
    data.algorithm = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [path_length]
    data.path_length = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [planning_time]
    data.planning_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [num_waypoints]
    data.num_waypoints = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.algorithm);
    return length + 25;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mr_msgs/PlannerResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ef19d2ad7038d14cfe1f591d32dcfcbf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Benchmark result for a single planning run.
    Header header
    
    # Algorithm identifier, e.g. "astar", "bcd".
    string algorithm
    
    # Total path length in meters.
    float64 path_length
    
    # Wall-clock planning time in seconds.
    float64 planning_time
    
    # Number of waypoints in the published path.
    int32 num_waypoints
    
    # True if the planner found a feasible path.
    bool success
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlannerResult(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.algorithm !== undefined) {
      resolved.algorithm = msg.algorithm;
    }
    else {
      resolved.algorithm = ''
    }

    if (msg.path_length !== undefined) {
      resolved.path_length = msg.path_length;
    }
    else {
      resolved.path_length = 0.0
    }

    if (msg.planning_time !== undefined) {
      resolved.planning_time = msg.planning_time;
    }
    else {
      resolved.planning_time = 0.0
    }

    if (msg.num_waypoints !== undefined) {
      resolved.num_waypoints = msg.num_waypoints;
    }
    else {
      resolved.num_waypoints = 0
    }

    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = PlannerResult;
