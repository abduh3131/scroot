// Auto-generated. Do not edit!

// (in-package scooter_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let sensor_msgs = _finder('sensor_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class SensorHub {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.lidar_ranges = null;
      this.angle_min = null;
      this.angle_increment = null;
      this.angle_max = null;
      this.ultrasonic_distance = null;
      this.imu_vector = null;
      this.camera_frame = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('lidar_ranges')) {
        this.lidar_ranges = initObj.lidar_ranges
      }
      else {
        this.lidar_ranges = [];
      }
      if (initObj.hasOwnProperty('angle_min')) {
        this.angle_min = initObj.angle_min
      }
      else {
        this.angle_min = 0.0;
      }
      if (initObj.hasOwnProperty('angle_increment')) {
        this.angle_increment = initObj.angle_increment
      }
      else {
        this.angle_increment = 0.0;
      }
      if (initObj.hasOwnProperty('angle_max')) {
        this.angle_max = initObj.angle_max
      }
      else {
        this.angle_max = 0.0;
      }
      if (initObj.hasOwnProperty('ultrasonic_distance')) {
        this.ultrasonic_distance = initObj.ultrasonic_distance
      }
      else {
        this.ultrasonic_distance = 0.0;
      }
      if (initObj.hasOwnProperty('imu_vector')) {
        this.imu_vector = initObj.imu_vector
      }
      else {
        this.imu_vector = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('camera_frame')) {
        this.camera_frame = initObj.camera_frame
      }
      else {
        this.camera_frame = new sensor_msgs.msg.Image();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SensorHub
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [lidar_ranges]
    bufferOffset = _arraySerializer.float32(obj.lidar_ranges, buffer, bufferOffset, null);
    // Serialize message field [angle_min]
    bufferOffset = _serializer.float32(obj.angle_min, buffer, bufferOffset);
    // Serialize message field [angle_increment]
    bufferOffset = _serializer.float32(obj.angle_increment, buffer, bufferOffset);
    // Serialize message field [angle_max]
    bufferOffset = _serializer.float32(obj.angle_max, buffer, bufferOffset);
    // Serialize message field [ultrasonic_distance]
    bufferOffset = _serializer.float32(obj.ultrasonic_distance, buffer, bufferOffset);
    // Serialize message field [imu_vector]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.imu_vector, buffer, bufferOffset);
    // Serialize message field [camera_frame]
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.camera_frame, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SensorHub
    let len;
    let data = new SensorHub(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [lidar_ranges]
    data.lidar_ranges = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [angle_min]
    data.angle_min = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angle_increment]
    data.angle_increment = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angle_max]
    data.angle_max = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ultrasonic_distance]
    data.ultrasonic_distance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [imu_vector]
    data.imu_vector = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [camera_frame]
    data.camera_frame = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.lidar_ranges.length;
    length += sensor_msgs.msg.Image.getMessageSize(object.camera_frame);
    return length + 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'scooter_control/SensorHub';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8a7ff4c96a4d7dde852a8ffd7c0319b9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Unified perception message from the sensor fusion node
    
    std_msgs/Header header
    
    # LiDAR data
    float32[] lidar_ranges
    float32 angle_min
    float32 angle_increment
    float32 angle_max
    
    # Ultrasonic sensor data
    float32 ultrasonic_distance
    
    # IMU data
    geometry_msgs/Vector3 imu_vector
    
    # Raw camera frame
    sensor_msgs/Image camera_frame
    
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
    MSG: sensor_msgs/Image
    # This message contains an uncompressed image
    # (0, 0) is at top-left corner of image
    #
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of camera
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
                         # If the frame_id here and the frame_id of the CameraInfo
                         # message associated with the image conflict
                         # the behavior is undefined
    
    uint32 height         # image height, that is, number of rows
    uint32 width          # image width, that is, number of columns
    
    # The legal values for encoding are in file src/image_encodings.cpp
    # If you want to standardize a new string format, join
    # ros-users@lists.sourceforge.net and send an email proposing a new encoding.
    
    string encoding       # Encoding of pixels -- channel meaning, ordering, size
                          # taken from the list of strings in include/sensor_msgs/image_encodings.h
    
    uint8 is_bigendian    # is this data bigendian?
    uint32 step           # Full row length in bytes
    uint8[] data          # actual matrix data, size is (step * rows)
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SensorHub(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.lidar_ranges !== undefined) {
      resolved.lidar_ranges = msg.lidar_ranges;
    }
    else {
      resolved.lidar_ranges = []
    }

    if (msg.angle_min !== undefined) {
      resolved.angle_min = msg.angle_min;
    }
    else {
      resolved.angle_min = 0.0
    }

    if (msg.angle_increment !== undefined) {
      resolved.angle_increment = msg.angle_increment;
    }
    else {
      resolved.angle_increment = 0.0
    }

    if (msg.angle_max !== undefined) {
      resolved.angle_max = msg.angle_max;
    }
    else {
      resolved.angle_max = 0.0
    }

    if (msg.ultrasonic_distance !== undefined) {
      resolved.ultrasonic_distance = msg.ultrasonic_distance;
    }
    else {
      resolved.ultrasonic_distance = 0.0
    }

    if (msg.imu_vector !== undefined) {
      resolved.imu_vector = geometry_msgs.msg.Vector3.Resolve(msg.imu_vector)
    }
    else {
      resolved.imu_vector = new geometry_msgs.msg.Vector3()
    }

    if (msg.camera_frame !== undefined) {
      resolved.camera_frame = sensor_msgs.msg.Image.Resolve(msg.camera_frame)
    }
    else {
      resolved.camera_frame = new sensor_msgs.msg.Image()
    }

    return resolved;
    }
};

module.exports = SensorHub;
