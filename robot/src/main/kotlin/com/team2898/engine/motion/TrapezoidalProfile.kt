package com.team2898.engine.motion

import edu.wpi.first.wpilibj.Timer

//todo when im not dead
class TrapezoidalProfile(
        val startTime: Double = Timer.getFPGATimestamp(),
        var targetPosition: Double,
        val maxAcceleration: Double, val maxDeacceleration: Double = maxAcceleration,
        val cruiseSpeed: Double,
        val startSpeed: Double, private val startTime: Double) {

    private val adjTargetPos = targetPosition-startPosition
    private val lastTime = 0.0
    private val isFinished=false

    // x is time
    // c is cruise vel
    // a is max acceleration
    //d is max deacceleration
    // m is start time
    // n is end time

    //{x <c/a +m < -c/d +n: ax-am, c/a +m < x < -c/d +n:c,x> -c/d+n:-dx+nd}
    init {
    }


    fun update(time: Double=Timer.getFPGATimestamp()) {

    }
}