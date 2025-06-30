
"use strict";

let SetIO = require('./SetIO.js')
let GetRobotSoftwareVersion = require('./GetRobotSoftwareVersion.js')
let SetAnalogOutput = require('./SetAnalogOutput.js')
let SetForceMode = require('./SetForceMode.js')
let SetSpeedSliderFraction = require('./SetSpeedSliderFraction.js')
let SetPayload = require('./SetPayload.js')

module.exports = {
  SetIO: SetIO,
  GetRobotSoftwareVersion: GetRobotSoftwareVersion,
  SetAnalogOutput: SetAnalogOutput,
  SetForceMode: SetForceMode,
  SetSpeedSliderFraction: SetSpeedSliderFraction,
  SetPayload: SetPayload,
};
