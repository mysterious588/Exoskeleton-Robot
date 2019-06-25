// Auto-generated. Do not edit!

// (in-package mpu.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class angles {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angleX = null;
      this.angleY = null;
      this.angleZ = null;
    }
    else {
      if (initObj.hasOwnProperty('angleX')) {
        this.angleX = initObj.angleX
      }
      else {
        this.angleX = 0;
      }
      if (initObj.hasOwnProperty('angleY')) {
        this.angleY = initObj.angleY
      }
      else {
        this.angleY = 0;
      }
      if (initObj.hasOwnProperty('angleZ')) {
        this.angleZ = initObj.angleZ
      }
      else {
        this.angleZ = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type angles
    // Serialize message field [angleX]
    bufferOffset = _serializer.int16(obj.angleX, buffer, bufferOffset);
    // Serialize message field [angleY]
    bufferOffset = _serializer.int16(obj.angleY, buffer, bufferOffset);
    // Serialize message field [angleZ]
    bufferOffset = _serializer.int16(obj.angleZ, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type angles
    let len;
    let data = new angles(null);
    // Deserialize message field [angleX]
    data.angleX = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [angleY]
    data.angleY = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [angleZ]
    data.angleZ = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mpu/angles';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '76513374ad214381430fd6261a078518';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 angleX
    int16 angleY
    int16 angleZ
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new angles(null);
    if (msg.angleX !== undefined) {
      resolved.angleX = msg.angleX;
    }
    else {
      resolved.angleX = 0
    }

    if (msg.angleY !== undefined) {
      resolved.angleY = msg.angleY;
    }
    else {
      resolved.angleY = 0
    }

    if (msg.angleZ !== undefined) {
      resolved.angleZ = msg.angleZ;
    }
    else {
      resolved.angleZ = 0
    }

    return resolved;
    }
};

module.exports = angles;
