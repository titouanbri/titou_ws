
"use strict";

let IsProgramSaved = require('./IsProgramSaved.js')
let Load = require('./Load.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let GetProgramState = require('./GetProgramState.js')
let GetRobotMode = require('./GetRobotMode.js')
let Popup = require('./Popup.js')
let RawRequest = require('./RawRequest.js')
let IsProgramRunning = require('./IsProgramRunning.js')
let AddToLog = require('./AddToLog.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')

module.exports = {
  IsProgramSaved: IsProgramSaved,
  Load: Load,
  IsInRemoteControl: IsInRemoteControl,
  GetProgramState: GetProgramState,
  GetRobotMode: GetRobotMode,
  Popup: Popup,
  RawRequest: RawRequest,
  IsProgramRunning: IsProgramRunning,
  AddToLog: AddToLog,
  GetSafetyMode: GetSafetyMode,
  GetLoadedProgram: GetLoadedProgram,
};
