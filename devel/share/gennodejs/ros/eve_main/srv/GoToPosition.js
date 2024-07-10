// Auto-generated. Do not edit!

// (in-package eve_main.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GoToPositionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.desiredXPosition = null;
      this.desiredZPosition = null;
      this.speed = null;
    }
    else {
      if (initObj.hasOwnProperty('desiredXPosition')) {
        this.desiredXPosition = initObj.desiredXPosition
      }
      else {
        this.desiredXPosition = 0.0;
      }
      if (initObj.hasOwnProperty('desiredZPosition')) {
        this.desiredZPosition = initObj.desiredZPosition
      }
      else {
        this.desiredZPosition = 0.0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GoToPositionRequest
    // Serialize message field [desiredXPosition]
    bufferOffset = _serializer.float32(obj.desiredXPosition, buffer, bufferOffset);
    // Serialize message field [desiredZPosition]
    bufferOffset = _serializer.float32(obj.desiredZPosition, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float32(obj.speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GoToPositionRequest
    let len;
    let data = new GoToPositionRequest(null);
    // Deserialize message field [desiredXPosition]
    data.desiredXPosition = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [desiredZPosition]
    data.desiredZPosition = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'eve_main/GoToPositionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c8d13069b95fb9c6b68d27613e61705e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 desiredXPosition
    float32 desiredZPosition
    float32 speed
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GoToPositionRequest(null);
    if (msg.desiredXPosition !== undefined) {
      resolved.desiredXPosition = msg.desiredXPosition;
    }
    else {
      resolved.desiredXPosition = 0.0
    }

    if (msg.desiredZPosition !== undefined) {
      resolved.desiredZPosition = msg.desiredZPosition;
    }
    else {
      resolved.desiredZPosition = 0.0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    return resolved;
    }
};

class GoToPositionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GoToPositionResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GoToPositionResponse
    let len;
    let data = new GoToPositionResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'eve_main/GoToPositionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GoToPositionResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: GoToPositionRequest,
  Response: GoToPositionResponse,
  md5sum() { return 'c8d13069b95fb9c6b68d27613e61705e'; },
  datatype() { return 'eve_main/GoToPosition'; }
};
