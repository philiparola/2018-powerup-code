package com.team2898.robot.config.IntakeConf

import com.team2898.engine.controlLoops.modernControl.StateSpaceGains
import com.team2898.engine.math.linear.Matrix

const val INTAKE_CONT_MAX_AMPS = 20
const val INTAKE_PEAK_MAX_AMPS = 100
const val INTAKE_PEAK_MAX_AMPS_DUR_MS = 50

// Maximum percent of the baseline surface speed we can add/remove to align the cube
const val MAX_ORIENTATION_ADJ_PERCENT = 50.0
const val INTAKEN_CUBE_THRESHOLD = 2 // inches

const val MAX_SURFACE_SPEED = 30.0 // ft/s

const val INTAKE_GEARING = 1 / 10.0


const val INTAKE_Kp = 0.0
const val INTAKE_Ki = 0.0
const val INTAKE_Kd = 0.0
const val INTAKE_Kf = 12/(18370* INTAKE_GEARING)

val INTAKE_K_CUBE = StateSpaceGains(
        A = Matrix(),
        B = Matrix(),
        C = Matrix(),
        D = Matrix(),
        K = Matrix(),
        Umax = Matrix(),
        Umin = Matrix()
)
val INTAKE_K_NO_CUBE = StateSpaceGains(
        A = Matrix(),
        B = Matrix(),
        C = Matrix(),
        D = Matrix(),
        K = Matrix(),
        Umax = Matrix(),
        Umin = Matrix()
)
