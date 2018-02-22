package com.team2898.engine.motion

import edu.wpi.first.wpilibj.Timer
import kotlin.math.max
import kotlin.math.sign

class TrapezoidProfile(
        var maxAcc: Double,
        var maxVel: Double,
        val currentPos: () -> Double,
        val time: () -> Double = { Timer.getFPGATimestamp() }
) {
    var lastTime = 0.0
    var timeToMaxVel = maxVel / maxAcc
        get() = maxVel / maxAcc
        set(timeToMax) {
            maxAcc = maxVel / field
        }

    var timeFromMaxVel = 0.0
    val timeTotal
        get() = timeFromMaxVel + timeToMaxVel

    var sign = 0.0

    var timeOffset: Double

    init {
        timeOffset = time()
    }

    var goalPos: Double = 0.0
        set(goal) { // set to set trapezoidal goal
            timeOffset = time()
            val setpoint = goal - currentPos()
            sign = sign(setpoint)
            val deltaPosMaxVel = (sign * setpoint) - (timeToMaxVel * maxVel)
            val timeAtMaxVel = deltaPosMaxVel / maxVel
            timeFromMaxVel = timeToMaxVel + timeAtMaxVel
            lastTime = time()
            field = goal
        }

    var velSetpoint = 0.0
        get() {
            posSetpoint // trigger get() lul
            return field
        }

    var posSetpoint: Double = 0.0
        get() { // get to get position controller input
            val time = time()-timeOffset
            val dt = time - lastTime
            if (time < timeToMaxVel) {
                field += (maxAcc * time * dt * sign)
                velSetpoint = (maxAcc * time * sign)
            } else if (time < timeFromMaxVel) {
                field += maxVel * dt * sign
                velSetpoint = maxVel * sign
            } else if (time < timeTotal) {
                val decelTime = time - timeFromMaxVel // Time we've been deaccelerating
                val vel = maxVel - maxAcc * decelTime
                field += vel * dt * sign
                velSetpoint = vel * sign
            }
            lastTime = time
            return field
        }

}