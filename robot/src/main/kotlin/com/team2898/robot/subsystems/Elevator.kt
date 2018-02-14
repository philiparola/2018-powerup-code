package com.team2898.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.ILooper
import com.team2898.engine.logic.ISelfCheck
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.motion.TalonWrapper
import com.team2898.robot.config.ElevatorConf.*
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D
import kotlin.math.PI

object Elevator: Subsystem(name ="Elevator", loopHz = 100.0), ISelfCheck, ILooper {
    val leftMaser = TalonWrapper(ELEV_LEFT_MASTER)
    val leftSlave = TalonWrapper(ELEV_LEFT_SLAVE)
    val rightMaster = TalonWrapper(ELEV_RIGHT_MASTER)
    val rightSlave = TalonWrapper(ELEV_RIGHT_SLAVE)

    val masters = listOf(leftMaser, rightMaster)

    val encRawVel
        get() = Vector2D(
                leftMaser.sensorCollection.quadratureVelocity.toDouble(),
                rightMaster.sensorCollection.quadratureVelocity.toDouble()
        )

    val encVelFtSec
        get() = Vector2D(
                (leftMaser.sensorCollection.quadratureVelocity.toDouble() / 409.6), // TODO * circ of the gear
                (rightMaster.sensorCollection.quadratureVelocity.toDouble() / 409.6)
        )

    val encRawPos
        get() = Vector2D(
                leftMaser.sensorCollection.quadraturePosition.toDouble(),
                rightMaster.sensorCollection.quadraturePosition.toDouble()
        )

    var targetPos: Double = 0.0
        set(value) {
            field = value
            masters {

            }
        }

    var currentPos: Vector2D
        get() = Vector2D(
                (leftMaser.sensorCollection.quadraturePosition / 4096.0) * (2.55 / 2) * PI / 12,
                (rightMaster.sensorCollection.quadraturePosition / 4096.0) * (2.55 / 2) * PI / 12
        )

    val ftSecToEnc: (Double) -> Double = { vel : Double ->
        vel * 12 / PI / (2.55 / 2) * 4096.0 / 10
    }

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
                changeControlMode(ControlMode.MotionMagic)

                configMotionCruiseVelocity(ftSecToEnc(2.0).toInt(), 0)
                configMotionAcceleration(ftSecToEnc(2.0).toInt(), 0)

                setPID(ELEV_Kp, ELEV_Ki, ELEV_Kd, ELEV_Kf)
                sensorCollection.setQuadraturePosition(0, 0)
            }
        }
        currentPos = Vector2D(0.0, 0.0)
    }

    fun masters(block: TalonWrapper.()-> Unit) {
        masters.forEach {
            it.apply(block)
        }
    }

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

