package com.team2898.engine.motion

import edu.wpi.first.wpilibj.DigitalOutput
import edu.wpi.first.wpilibj.Timer
import kotlin.math.max
import kotlin.math.sign

class TrapezoidProfile(
        var maxAcc: Double, // in FSTU
        var maxVel: Double, // in FSTU
        var currentVel: () -> Double,
        var currentPos: () -> Double
) {
    var startTime = 0.0

    var timeToZero = maxVel / maxAcc
        get() = maxVel / maxAcc

    var targetPos = 0.0
        set(value) { field = value }
        get() = field

    var timeToMax: () -> Double = {
        if (currentVel() == maxVel) {
            0.0
        } else {
            (maxVel - currentVel()) / maxAcc
        }
    }

    val hasTodecelerate = {  // assuming it's max vel, and distance it takes to zero it
        var offset = targetPos - currentPos()
        val time = (offset * 2) / currentVel()
//        offset = t * currentVel() + (maxAcc/2) * Math.pow(t, 2.0)
    }

    // d = tv + 1/2 a t^2
    // t =

    // max vel, max acc

    // d = (v1 + v2) / 2 * t
    // d / (v1 + v2) = 1 / 2 * t

    fun updateTarget(target: Double) {
        startTime = Timer.getFPGATimestamp()
        targetPos = target
    }

    fun updateProfile(currentTime: Double = Timer.getFPGATimestamp(), currentVel: Double, currentPos: Double) {
        val deltaTime = currentTime - startTime
    }
}