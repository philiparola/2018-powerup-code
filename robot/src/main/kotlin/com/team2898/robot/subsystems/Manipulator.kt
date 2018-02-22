package com.team2898.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.ILooper
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.motion.TalonWrapper
import com.team2898.robot.config.ElevatorConf.SPROCKET_PITCH_DIA
import com.team2898.robot.config.ManipConf.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.math.*


object Manipulator : Subsystem(50.0, "manipulator"), ILooper {
    override val enableTimes: List<GamePeriods> = listOf(GamePeriods.AUTO, GamePeriods.TELEOP)

    override val loop: AsyncLooper = AsyncLooper(100.0) {
        SmartDashboard.putNumber("Manipulator sin", currentPos().sin)
        SmartDashboard.putNumber("Manipulator cos", currentPos().cos)
    }

    val talon = TalonWrapper(MANIP_TALON)

    val currentPos: () -> Rotation2d
        get() = {
            encPosToRotation2d(talon.sensorCollection.quadraturePosition.toDouble())
        }

    var targetPos = Rotation2d(0.0, 0.0)
        set(value) {
            if (value != field) {
                talon.changeControlMode(ControlMode.MotionMagic)
                talon.set(rotation2dToEncPos(value))
                field = value
            }
        }

    val encPosToRotation2d = { encPos: Double ->
        Rotation2d.createFromRadians(encPos / 4096 * 2 * PI)
    }

    val rotation2dToEncPos = { rotation2d: Rotation2d ->
        rotation2d.radians / PI / 2 * 4096
    }

    val velToSTU =  { vel: Double ->
        vel.roundToInt() // TODO fix the shit
    }

    val accToSTU = { acc: Double ->
        acc.roundToInt() // TODO fix the shit
    }


    init {
        talon.apply {
            setMagEncoder()
            configPeakCurrentLimit(MANIP_PEAK_MAX_AMPS, 0)
            configContinuousCurrentLimit(MANIP_CONT_MAX_AMPS, 0)
            configPeakCurrentDuration(MANIP_PEAK_MAX_AMPS_DUR_MS, 0)
            enableCurrentLimit(MANIP_CURRENT_LIMIT)

            configMotionAcceleration(accToSTU(MANIP_MAX_ACC), 0)
            configMotionCruiseVelocity(velToSTU(MANIP_MAX_VEL), 0)

            setPID(MANIP_Kp, MANIP_Ki, MANIP_Kd)
            zeroEnc()
        }
    }

    fun zeroEnc() {
        talon.sensorCollection.setQuadraturePosition(0, 0)
    }

    override fun onStart() {
    }

    override fun selfCheckup(): Boolean {
        return true
    }

    override fun selfTest(): Boolean {
        return true
    }
}