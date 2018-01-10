package com.team2898.engine.controlLoops.classicControl

import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod
import com.team2898.engine.math.clamp

/** 3DOF PID Controller for simultanious control of position, velocity, and acceleration
 * @param Kp
 *  Position: Proportional gain
 * @param Ki
 *  Position: Integral gain
 * @param Kvp
 *  Velocity: Proportional gain
 * @param Kvf
 *  Velocity: Feed-forward gain
 * @param Kaf
 *  Acceleration: Feed-forward gain
 *
 */
class PVAPID(val Kp: Double, val Ki: Double,
             val Kvp: Double, val Kvf: Double,
             val Kaf: Double,
             val minOutput: Double = -1.0,
             val maxOutput: Double = 1.0) {

    var lastPos: Double = Double.NaN
    var lastVel: Double = Double.NaN
    var integrator: Double = 0.0

    fun update(position: Double, velocity: Double, targetPos: Double, targetVel: Double, targetAcc: Double, dt: Double): Double {
        if (lastPos == Double.NaN) lastPos = position
        if (velocity == Double.NaN) lastVel = velocity

        val posError = position - targetPos
        val velError = velocity - targetVel

        integrator += posError


        var output = Kp * posError + Kvp * velError + Kvf * targetVel + Kaf * targetAcc

        if (minOutput <= output && output <= maxOutput) {
            integrator += posError * dt
            output += Ki * integrator
        } else integrator = 0.0

        output = clamp(output, minOutput, maxOutput)
        return output
    }

    fun reset() {

    }
}