package com.team2898.robot.config.IntakeConf

const val SPARK_LEFT = 0
const val SPARK_RIGHT = 1

const val INTAKE_CURRENT_LIMIT = true
const val INTAKE_CONT_MAX_AMPS = 15
const val INTAKE_PEAK_MAX_AMPS = 17
const val INTAKE_PEAK_MAX_AMPS_DUR_MS = 500

const val INTAKE_Kp = 0.043
const val INTAKE_Ki = 0.01
const val INTAKE_Kd = 0.0

const val LEFT_INTAKE_SOLENOID_FORWARD_ID = 0
const val LEFT_INTAKE_SOLENOID_REVERSE_ID = 0

const val RIGHT_INTAKE_SOLENOID_FORWARD_ID = 0
const val RIGHT_INTAKE_SOLENOID_REVERSE_ID = 0

const val INTAKE_MAX_VEL: Int = 1000 // TODO in FSTU
const val INTAKE_MAX_ACC: Int = 1000 // TODO

const val MAX_POS = 90.0 // TODO in angle
const val MIN_POS = 0.0 // TODO

const val ABSO_OFFSET_LEFT = 2212.0 // TODO
const val ABSO_OFFSET_RIGHT = 0.0 // TODO