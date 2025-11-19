// Auto-generated. Do not edit!

// (in-package sensor_interface.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

class SensorHub {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.lidar_ranges = null;
      this.lidar_angle_min = null;
      this.lidar_angle_max = null;
      this.lidar_angle_increment = null;
      this.camera_frame = null;
      this.imu_vector = null;
      this.ultrasonic_distance = null;
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
      if (initObj.hasOwnProperty('lidar_angle_min')) {
        this.lidar_angle_min = initObj.lidar_angle_min
      }
      else {
        this.lidar_angle_min = 0.0;
      }
      if (initObj.hasOwnProperty('lidar_angle_max')) {
        this.lidar_angle_max = initObj.lidar_angle_max
      }
      else {
        this.lidar_angle_max = 0.0;
      }
      if (initObj.hasOwnProperty('lidar_angle_increment')) {
        this.lidar_angle_increment = initObj.lidar_angle_increment
      }
      else {
        this.lidar_angle_increment = 0.0;
      }
      if (initObj.hasOwnProperty('camera_frame')) {
        this.camera_frame = initObj.camera_frame
      }
      else {
        this.camera_frame = new sensor_msgs.msg.Image();
      }
      if (initObj.hasOwnProperty('imu_vector')) {
        this.imu_vector = initObj.imu_vector
      }
      else {
        this.imu_vector = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('ultrasonic_distance')) {
        this.ultrasonic_distance = initObj.ultrasonic_distance
      }
      else {
        this.ultrasonic_distance = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SensorHub
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [lidar_ranges]
    bufferOffset = _arraySerializer.float32(obj.lidar_ranges, buffer, bufferOffset, null);
    // Serialize message field [lidar_angle_min]
    bufferOffset = _serializer.float32(obj.lidar_angle_min, buffer, bufferOffset);
    // Serialize message field [lidar_angle_max]
    bufferOffset = _serializer.float32(obj.lidar_angle_max, buffer, bufferOffset);
    // Serialize message field [lidar_angle_increment]
    bufferOffset = _serializer.float32(obj.lidar_angle_increment, buffer, bufferOffset);
    // Serialize message field [camera_frame]
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.camera_frame, buffer, bufferOffset);
    // Serialize message field [imu_vector]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.imu_vector, buffer, bufferOffset);
    // Serialize message field [ultrasonic_distance]
    bufferOffset = _serializer.float32(obj.ultrasonic_distance, buffer, bufferOffset);
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
    // Deserialize message field [lidar_angle_min]
    data.lidar_angle_min = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lidar_angle_max]
    data.lidar_angle_max = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lidar_angle_increment]
    data.lidar_angle_increment = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [camera_frame]
    data.camera_frame = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    // Deserialize message field [imu_vector]
    data.imu_vector = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [ultrasonic_distance]
    data.ultrasonic_distance = _deserializer.float32(buffer, bufferOffset);
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
    return 'sensor_interface/SensorHub';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b0a1fb0ce2812b325de845d2687d9680';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    float32[] lidar_ranges
    float32 lidar_angle_min
    float32 lidar_angle_max
    float32 lidar_angle_increment
    sensor_msgs/Image camera_frame
    geometry_msgs/Vector3 imu_vector
    float32 ultrasonic_distance
    
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

    if (msg.lidar_angle_min !== undefined) {
      resolved.lidar_angle_min = msg.lidar_angle_min;
    }
    else {
      resolved.lidar_angle_min = 0.0
    }

    if (msg.lidar_angle_max !== undefined) {
      resolved.lidar_angle_max = msg.lidar_angle_max;
    }
    else {
      resolved.lidar_angle_max = 0.0
    }

    if (msg.lidar_angle_increment !== undefined) {
      resolved.lidar_angle_increment = msg.lidar_angle_increment;
    }
    else {
      resolved.lidar_angle_increment = 0.0
    }

    if (msg.camera_frame !== undefined) {
      resolved.camera_frame = sensor_msgs.msg.Image.Resolve(msg.camera_frame)
    }
    else {
      resolved.camera_frame = new sensor_msgs.msg.Image()
    }

    if (msg.imu_vector !== undefined) {
      resolved.imu_vector = geometry_msgs.msg.Vector3.Resolve(msg.imu_vector)
    }
    else {
      resolved.imu_vector = new geometry_msgs.msg.Vector3()
    }

    if (msg.ultrasonic_distance !== undefined) {
      resolved.ultrasonic_distance = msg.ultrasonic_distance;
    }
    else {
      resolved.ultrasonic_distance = 0.0
    }

    return resolved;
    }
};

module.exports = SensorHub;
