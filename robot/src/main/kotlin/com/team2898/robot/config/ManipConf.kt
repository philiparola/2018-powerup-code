package com.team2898.robot.config.ManipConf

import com.team2898.engine.kinematics.Rotation2d


const val MANIP_CURRENT_LIMIT = true
const val MANIP_CONT_MAX_AMPS = 15
const val MANIP_PEAK_MAX_AMPS = 17
const val MANIP_PEAK_MAX_AMPS_DUR_MS = 500

const val MANIP_Kp = 0.5
const val MANIP_Ki = 0.001
const val MANIP_Kd = 1.0
const val MANIP_Kf = 0.83

const val MANIP_MAX_VEL = 1100
const val MANIP_MAX_ACC = 1000

const val ABSO_OFFSET = 0.0 // TODO

///////////// COMMAND ////////////

// ASSUME [1 0] POSITION IS FLAT AND LEVEL
// View manip coordinate system with front of robot to +x direction

val START_POS = Rotation2d.createFromDegrees(90.0) // TODO

// Intake
val INTAKE_POS = Rotation2d.createFromDegrees(0.0) // TODO

// deploy
val HOLD_POS = Rotation2d.createFromDegrees(30.0)// TODO

// light throw
val LIGHT_THROW_FINAL_POS = Rotation2d.createFromDegrees(-90.0) // TODO

// heavy throw
val HEAVY_THROW_FINAL_POS = Rotation2d.createFromDegrees(-90.0) // TODO
