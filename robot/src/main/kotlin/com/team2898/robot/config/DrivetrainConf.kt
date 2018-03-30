package com.team2898.robot.config.DrivetrainConf

import com.sun.xml.internal.bind.v2.runtime.reflect.opt.TransducedAccessor_method_Boolean
import kotlin.math.round
import kotlin.math.roundToInt


const val Kp = 0.0 // Constants for bunnybot
const val Ki = 0.0
const val Kd = 0.0
const val Kf = 1023 / 9493.94

const val LOCK_KP = 0.050
const val LOCK_KI = 0.001
const val LOCK_KD = 0.000

const val CURRENT_LIMIT = true
const val CONT_MAX_AMPS = 15
const val PEAK_MAX_AMPS = 17
const val PEAK_MAX_AMPS_DUR_MS = 500

const val LEFT_M = 1.26
const val LEFT_B = -1.55

const val RIGHT_M = 1.21
const val RIGHT_B = -1.53


const val L_MM_ACC_FTS2 = 3.0
const val L_MM_CRUISE_FTS = 3.0
const val R_MM_ACC_FTS2 = L_MM_ACC_FTS2 * (RIGHT_M / LEFT_M)
const val R_MM_CRUISE_FTS = L_MM_CRUISE_FTS * (RIGHT_M / LEFT_M)

val L_MM_ACC_STU2 = (L_MM_ACC_FTS2 * (1 / (.5 * Math.PI)) * 1 / 10.0 * 4096.0).roundToInt()
val R_MM_ACC_STU2 = (R_MM_ACC_FTS2 * (1 / (.5 * Math.PI)) * 1 / 10.0 * 4096.0).roundToInt()
val L_MM_CRUISE_STU = (L_MM_CRUISE_FTS * (1 / (.5 * Math.PI)) * 1 / 10.0 * 4096.0).roundToInt()
val R_MM_CRUISE_STU = (R_MM_CRUISE_FTS * (1 / (.5 * Math.PI)) * 1 / 10.0 * 4096.0).roundToInt()

// y = velocity, x = voltage

// left motor
// y = 1.26x - 1.55

// (y - b) / a = x

// right motor
// y = 1.21x - 1.53

