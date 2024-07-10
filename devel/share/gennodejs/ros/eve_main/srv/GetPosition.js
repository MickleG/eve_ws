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

class GetPositionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPositionRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPositionRequest
    let len;
    let data = new GetPositionRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'eve_main/GetPositionRequest';
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
    const resolved = new GetPositionRequest(null);
    return resolved;
    }
};

class GetPositionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.xPosition = null;
      this.yPosition = null;
      this.zPosition = null;
    }
    else {
      if (initObj.hasOwnProperty('xPosition')) {
        this.xPosition = initObj.xPosition
      }
      else {
        this.xPosition = 0.0;
      }
      if (initObj.hasOwnProperty('yPosition')) {
        this.yPosition = initObj.yPosition
      }
      else {
        this.yPosition = 0.0;
      }
      if (initObj.hasOwnProperty('zPosition')) {
        this.zPosition = initObj.zPosition
      }
      else {
        this.zPosition = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPositionResponse
    // Serialize message field [xPosition]
    bufferOffset = _serializer.float32(obj.xPosition, buffer, bufferOffset);
    // Serialize message field [yPosition]
    bufferOffset = _serializer.float32(obj.yPosition, buffer, bufferOffset);
    // Serialize message field [zPosition]
    bufferOffset = _serializer.float32(obj.zPosition, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPositionResponse
    let len;
    let data = new GetPositionResponse(null);
    // Deserialize message field [xPosition]
    data.xPosition = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yPosition]
    data.yPosition = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [zPosition]
    data.zPosition = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'eve_main/GetPositionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f98d79514754ade5731789b7227e61f3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 xPosition
    float32 yPosition
    float32 zPosition
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPositionResponse(null);
    if (msg.xPosition !== undefined) {
      resolved.xPosition = msg.xPosition;
    }
    else {
      resolved.xPosition = 0.0
    }

    if (msg.yPosition !== undefined) {
      resolved.yPosition = msg.yPosition;
    }
    else {
      resolved.yPosition = 0.0
    }

    if (msg.zPosition !== undefined) {
      resolved.zPosition = msg.zPosition;
    }
    else {
      resolved.zPosition = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: GetPositionRequest,
  Response: GetPositionResponse,
  md5sum() { return 'f98d79514754ade5731789b7227e61f3'; },
  datatype() { return 'eve_main/GetPosition'; }
};
