package com.team2898.robot.config.ManipConf

import com.team2898.engine.kinematics.Rotation2d


const val MANIP_CURRENT_LIMIT = true
const val MANIP_CONT_MAX_AMPS = 15
const val MANIP_PEAK_MAX_AMPS = 17
const val MANIP_PEAK_MAX_AMPS_DUR_MS = 500

const val MANIP_Kp = 0.0
const val MANIP_Ki = 0.0
const val MANIP_Kd = 0.0

const val MANIP_MAX_VEL = 1100
const val MANIP_MAX_ACC = 1000

const val ABSO_OFFSET = 0.0 // TODO

///////////// COMMAND ////////////

// Intake
val INTAKE_POS = Rotation2d(0.0, 0.0) // TODO

// deploy
val DEPLOY_POS = Rotation2d(0.0, 0.0) // TODO

// light throw
val LIGHT_THROW_FINAL_POS = Rotation2d(0.0, 0.0) // TODO

// heavy throw
val HEAVY_THROW_FINAL_POS = Rotation2d(0.0, 0.0) // TODO
