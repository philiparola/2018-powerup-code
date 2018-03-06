package com.team2898.engine.motion

import edu.wpi.first.wpilibj.Timer
import kotlin.math.abs


class TrapezoidProfile(
        var maxAcc: Double, // in FSTU
        var maxVel: Double, // in FSTU
        var currentVel: () -> Double,
        var currentPos: () -> Double
) {
    data class PVAData(val currentPos: () -> Double, val currentVel:() -> Double, val targetPos: Double, val targetVel: Double, val targetAcc: Double)

    /**
     * REST: resting
     * ACC: Accelerating
     * CONST: Const velocity
     * DEC: Decelerating
     */
    enum class CurrentState { REST, ACC, CONST, DEC }

    val POS_OFFSET = 100
    val VEL_OFFSET = 100

    var currentState = CurrentState.REST
    var lastTime = 0.0
    var startTime = 0.0

    var targetPos = 0.0
        set(value) {
            field = value
        }
        get() = field

    init {
        lastTime = Timer.getFPGATimestamp()
    }

    val hasTodecelerate = {
        // returns true if it has to decelerate
        val time = currentVel() / maxAcc
        val offset = targetPos - currentPos()
        offset >= time * currentVel() + (maxAcc / 2) * Math.pow(time, 2.0)
    }


    fun updateTarget(target: Double) {
        startTime = Timer.getFPGATimestamp()
        targetPos = target
        currentState = CurrentState.ACC
    }

    /**
     * @return CurrentState class depending on current velocity, and current position
     */
    fun updateCurrentStatus() {
        currentState = if (hasTodecelerate()) CurrentState.DEC
        else if (abs(currentVel()) == maxVel - VEL_OFFSET) CurrentState.CONST
        else if (abs(currentPos()) == targetPos - POS_OFFSET) CurrentState.REST
        else CurrentState.ACC
    }

    /**
     * @param currentTime current time, default = Timer.getFPGATimestamp
     * @return PVAData class which has
     *      current position
     *      current velocity
     *      target position (d = Vi * t + a/2t^2)
     *      target velocity (Vf = Vi + ta)
     *      target acc (max theoretical acceleration, or -mac acceleration)
     */
    fun updateProfile(currentTime: Double = Timer.getFPGATimestamp()): PVAData  {
        val deltaTime = currentTime - lastTime
        lastTime = currentTime
        updateCurrentStatus()
        return when (currentState) {
            CurrentState.ACC -> {
                PVAData(
                        currentPos,
                        currentVel,
                        deltaTime * currentVel() + (maxAcc/2) * Math.pow(deltaTime, 2.0),
                        currentVel() + maxAcc * deltaTime,
                        maxAcc
                )
            }
            CurrentState.CONST -> {
                PVAData(
                        currentPos,
                        currentVel,
                        deltaTime * currentVel(),
                        maxVel,
                        0.0
                )
            }
            CurrentState.DEC -> {
                PVAData(
                        currentPos,
                        currentVel,
                        deltaTime * currentVel() + (-maxAcc/2) * Math.pow(deltaTime, 2.0),
                       currentVel() + -maxAcc * deltaTime,
                        -maxAcc
                )
            }
            CurrentState.REST -> {
                PVAData(
                        currentPos,
                        currentVel,
                        currentPos(),
                        currentVel(),
                        0.0
                )
            }
        }
    }
}