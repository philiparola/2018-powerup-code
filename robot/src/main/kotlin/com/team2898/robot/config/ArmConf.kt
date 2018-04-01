package com.team2898.robot.config.ArmConf

import com.team2898.robot.subsystems.Arm.ticksToDegrees
import kotlin.math.roundToInt

const val ARM_PWM_OFFSET: Int = -2065

const val CRUISE_SPEED: Double = 120.0 // Deg/s
const val MAX_ACC: Double = CRUISE_SPEED/2 // Deg/s^2

val MM_CRUISE_SPEED = (340.0).roundToInt() // STU
val MM_MAX_ACC = (MM_CRUISE_SPEED/1.5).roundToInt() // STU/s

const val ARM_CONT_MAX_AMPS = 20
const val ARM_PEAK_MAX_AMPS = 35
const val ARM_PEAK_MAX_AMPS_DUR_MS = 250

const val ARM_REDUCTION = (1 / 10.0) * (18.0 / 66.0) * (24.0 / 60.0) //= 0.0109090909 = 1:96.1

const val ARM_Kpp = 0.045 // Full output at 120 deg error
const val ARM_Kpi = 0.0
const val ARM_HORIZ_VOLTAGE = -2.00
const val ARM_Kvp = 0.0

const val TALON_Kp = 512.0/((45.0/360.0)*4096)// Half output at 180 deg error -> 1.5
//const val TALON_Ki = 0.002
//const val TALON_Kd = 10.0
//const val TALON_Kf = 0.769
const val TALON_Ki = 0.0
const val TALON_Kd = 30.0
const val TALON_Kf = 2.933

const val TALON_Kf_ADD = 2.0/12.0

// 11v output = 16839 rpm. Commanded deg/s of 16839 * ARM_REDUCTION * 1/60 * 360 should result in 11v output
// 16839 * ARM_REDUCTION * 1/60 * 360 * kvf = 11
// 11/(16839 * ARM_REDUCTION* (1/60) * 360) = kvf

const val ARM_Kvf: Double = 11.0 / (16839.0 * ARM_REDUCTION * (1 / 60.0) * 360.0)
const val ARM_Kaf = 0.0

const val SETPOINT_1_DEG = 90.0
const val SETPOINT_2_DEG = 45.0
const val SETPOINT_3_DEG = 0.0
const val SETPOINT_4_DEG = 115.0

const val ARM_FORWARD_LIMIT = 1400
const val ARM_REVERSE_LIMIT = -126
val ARM_FORWARD_LIMIT_DEG = ticksToDegrees(ARM_FORWARD_LIMIT)
val ARM_REVERSE_LIMIT_DEG = ticksToDegrees(ARM_REVERSE_LIMIT)
