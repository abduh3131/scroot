// Auto-generated. Do not edit!

// (in-package scooter_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let YoloDetection = require('./YoloDetection.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class YoloDetectionArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.detections = null;
      this.source_model = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('detections')) {
        this.detections = initObj.detections
      }
      else {
        this.detections = [];
      }
      if (initObj.hasOwnProperty('source_model')) {
        this.source_model = initObj.source_model
      }
      else {
        this.source_model = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type YoloDetectionArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [detections]
    // Serialize the length for message field [detections]
    bufferOffset = _serializer.uint32(obj.detections.length, buffer, bufferOffset);
    obj.detections.forEach((val) => {
      bufferOffset = YoloDetection.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [source_model]
    bufferOffset = _serializer.string(obj.source_model, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type YoloDetectionArray
    let len;
    let data = new YoloDetectionArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [detections]
    // Deserialize array length for message field [detections]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.detections = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.detections[i] = YoloDetection.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [source_model]
    data.source_model = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.detections.forEach((val) => {
      length += YoloDetection.getMessageSize(val);
    });
    length += _getByteLength(object.source_model);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'scooter_control/YoloDetectionArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '478a9916436565eb349ccaef92e54ba5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # An array of YOLO detections from the dual_yolo_node
    
    std_msgs/Header header
    
    # Array of individual detections
    YoloDetection[] detections
    
    # Source of the detection (e.g., "yolov8n", "yolov8s", "fused")
    string source_model
    
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
    MSG: scooter_control/YoloDetection
    # Defines a single YOLO object detection
    
    # Class label of the detected object
    string class_label
    
    # Confidence score of the detection
    float32 confidence
    
    # Bounding box coordinates (in pixel space)
    # top-left corner
    int32 x_min
    int32 y_min
    # bottom-right corner
    int32 x_max
    int32 y_max
    float32 distance   # estimated distance to object in meters, -1 if unknown
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new YoloDetectionArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.detections !== undefined) {
      resolved.detections = new Array(msg.detections.length);
      for (let i = 0; i < resolved.detections.length; ++i) {
        resolved.detections[i] = YoloDetection.Resolve(msg.detections[i]);
      }
    }
    else {
      resolved.detections = []
    }

    if (msg.source_model !== undefined) {
      resolved.source_model = msg.source_model;
    }
    else {
      resolved.source_model = ''
    }

    return resolved;
    }
};

module.exports = YoloDetectionArray;
