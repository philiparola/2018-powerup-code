package com.team2898.robot.config.DrivetrainConf


const val Kp = 0.0 // Constants for bunnybot
const val Ki = 0.0
const val Kd = 0.0
const val Kf = 1023/9493.94

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

// y = velocity, x = voltage

// left motor
// y = 1.26x - 1.55

// (y - b) / a = x

// right motor
// y = 1.21x - 1.53