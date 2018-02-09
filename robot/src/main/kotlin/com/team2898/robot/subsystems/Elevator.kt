package com.team2898.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlFrame
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced
import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.ILooper
import com.team2898.engine.logic.ISelfCheck
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.motion.DriveSignal
import com.team2898.engine.motion.TalonWrapper
import com.team2898.robot.config.ElevatorConf.*
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D

object Elevator: Subsystem(name ="Elevator", loopHz = 100.0), ISelfCheck, ILooper {
    val leftMaser = TalonWrapper(ELEV_LEFT_MASTER)
    val leftSlave = TalonWrapper(ELEV_LEFT_SLAVE)
    val rightMaster = TalonWrapper(ELEV_RIGHT_MASTER)
    val rightSlave = TalonWrapper(ELEV_RIGHT_SLAVE)

    val masters = listOf(leftMaser, rightMaster)

    init {
        leftSlave slaveTo leftMaser
        rightSlave slaveTo rightMaster

        masters.forEach {
            it.apply {
                setMagEncoder()
                configPeakCurrentLimit(ELEV_PEAK_MAX_AMPS, 0)
                configContinuousCurrentLimit(ELEV_CONT_MAX_AMPS, 0)
                configPeakCurrentDuration(ELEV_PEAK_MAX_AMPS_DUR_MS, 0)
                enableCurrentLimit(ELEV_CURRENT_LIMIT)

                setPID(ELEV_Kp, ELEV_Ki, ELEV_Kd, ELEV_Kf)
            }
        }
    }

    val encRawVel
        get() = Vector2D(
                leftMaser.sensorCollection.quadratureVelocity.toDouble(),
                rightMaster.sensorCollection.quadratureVelocity.toDouble()
        )

    val encRawPos
        get() = Vector2D(
                leftMaser.sensorCollection.quadraturePosition.toDouble(),
                rightMaster.sensorCollection.quadraturePosition.toDouble()
        )

    override val loop: AsyncLooper
        get() = super.loop

    override val enableTimes: List<GamePeriods> = listOf(GamePeriods.TELEOP, GamePeriods.AUTO)

    override fun onStart() {
    }
    override fun onLoop() {
    }

    override fun onStop() {
    }

    override fun selfCheckup(): Boolean {
        return true
    }

    override fun selfTest(): Boolean {
        return true
    }

}

