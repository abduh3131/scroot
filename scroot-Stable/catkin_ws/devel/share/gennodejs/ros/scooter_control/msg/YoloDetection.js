// Auto-generated. Do not edit!

// (in-package scooter_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class YoloDetection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.class_label = null;
      this.confidence = null;
      this.x_min = null;
      this.y_min = null;
      this.x_max = null;
      this.y_max = null;
      this.distance = null;
    }
    else {
      if (initObj.hasOwnProperty('class_label')) {
        this.class_label = initObj.class_label
      }
      else {
        this.class_label = '';
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
      if (initObj.hasOwnProperty('x_min')) {
        this.x_min = initObj.x_min
      }
      else {
        this.x_min = 0;
      }
      if (initObj.hasOwnProperty('y_min')) {
        this.y_min = initObj.y_min
      }
      else {
        this.y_min = 0;
      }
      if (initObj.hasOwnProperty('x_max')) {
        this.x_max = initObj.x_max
      }
      else {
        this.x_max = 0;
      }
      if (initObj.hasOwnProperty('y_max')) {
        this.y_max = initObj.y_max
      }
      else {
        this.y_max = 0;
      }
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type YoloDetection
    // Serialize message field [class_label]
    bufferOffset = _serializer.string(obj.class_label, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float32(obj.confidence, buffer, bufferOffset);
    // Serialize message field [x_min]
    bufferOffset = _serializer.int32(obj.x_min, buffer, bufferOffset);
    // Serialize message field [y_min]
    bufferOffset = _serializer.int32(obj.y_min, buffer, bufferOffset);
    // Serialize message field [x_max]
    bufferOffset = _serializer.int32(obj.x_max, buffer, bufferOffset);
    // Serialize message field [y_max]
    bufferOffset = _serializer.int32(obj.y_max, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.float32(obj.distance, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type YoloDetection
    let len;
    let data = new YoloDetection(null);
    // Deserialize message field [class_label]
    data.class_label = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [x_min]
    data.x_min = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [y_min]
    data.y_min = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [x_max]
    data.x_max = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [y_max]
    data.y_max = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.class_label);
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'scooter_control/YoloDetection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '83c40f205e7d2dabf77dfc8fb221a703';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new YoloDetection(null);
    if (msg.class_label !== undefined) {
      resolved.class_label = msg.class_label;
    }
    else {
      resolved.class_label = ''
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    if (msg.x_min !== undefined) {
      resolved.x_min = msg.x_min;
    }
    else {
      resolved.x_min = 0
    }

    if (msg.y_min !== undefined) {
      resolved.y_min = msg.y_min;
    }
    else {
      resolved.y_min = 0
    }

    if (msg.x_max !== undefined) {
      resolved.x_max = msg.x_max;
    }
    else {
      resolved.x_max = 0
    }

    if (msg.y_max !== undefined) {
      resolved.y_max = msg.y_max;
    }
    else {
      resolved.y_max = 0
    }

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0.0
    }

    return resolved;
    }
};

module.exports = YoloDetection;
