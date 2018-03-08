
"use strict";

let RestartController = require('./RestartController.js')
let SetSpeed = require('./SetSpeed.js')
let SetCompliancePunch = require('./SetCompliancePunch.js')
let SetTorqueLimit = require('./SetTorqueLimit.js')
let SetComplianceMargin = require('./SetComplianceMargin.js')
let TorqueEnable = require('./TorqueEnable.js')
let SetComplianceSlope = require('./SetComplianceSlope.js')
let StartController = require('./StartController.js')
let StopController = require('./StopController.js')

module.exports = {
  RestartController: RestartController,
  SetSpeed: SetSpeed,
  SetCompliancePunch: SetCompliancePunch,
  SetTorqueLimit: SetTorqueLimit,
  SetComplianceMargin: SetComplianceMargin,
  TorqueEnable: TorqueEnable,
  SetComplianceSlope: SetComplianceSlope,
  StartController: StartController,
  StopController: StopController,
};
