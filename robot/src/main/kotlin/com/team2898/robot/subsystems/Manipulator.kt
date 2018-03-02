package com.team2898.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.logging.LogLevel
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.SelfCheckFailException
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.ILooper
import com.team2898.engine.logic.SelfCheckManager
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.motion.TalonWrapper
import com.team2898.robot.config.ManipConf.*
import com.team2898.robot.config.RobotMap.MANIP_TALON
import edu.wpi.first.wpilibj.DriverStation
import org.apache.commons.math3.geometry.euclidean.threed.Rotation
import kotlin.math.*

object Manipulator : Subsystem(50.0, "manipulator"), ILooper {
    override val enableTimes: List<GamePeriods> = listOf(GamePeriods.AUTO, GamePeriods.TELEOP)

    val talon = TalonWrapper(MANIP_TALON)

    val currentPos: Rotation2d
        get() = encPosToRotation2d(talon.sensorCollection.quadraturePosition.toDouble())


    var targetPos = Rotation2d(0.0, 0.0)
        set(value) {
            if (value != field) {
                talon.changeControlMode(ControlMode.MotionMagic)
                talon.set(rotation2dToEncPos(value))
                field = value
            }
        }

    fun encPosToRotation2d(encPos: Double): Rotation2d {
        return Rotation2d.createFromRadians(encPos / 4096 * 2 * PI)
    }

    fun rotation2dToEncPos(rotation2d: Rotation2d): Double {
        return rotation2d.radians / PI / 2 * 4096
    }

    init {
        talon.apply {
            setMagEncoder()
            configPeakCurrentLimit(MANIP_PEAK_MAX_AMPS, 0)
            configContinuousCurrentLimit(MANIP_CONT_MAX_AMPS, 0)
            configPeakCurrentDuration(MANIP_PEAK_MAX_AMPS_DUR_MS, 0)
            enableCurrentLimit(MANIP_CURRENT_LIMIT)

            configMotionAcceleration(MANIP_MAX_ACC, 0)
            configMotionCruiseVelocity(MANIP_MAX_VEL, 0)

            setPID(MANIP_Kp, MANIP_Ki, MANIP_Kd, MANIP_Kf,0, 102)
        }
    }


    override fun onStart() {
        this.targetPos = START_POS
    }

    override fun selfCheckup(): Boolean {
        if (talon.pwmPos == 0) {
            try {
                throw SelfCheckFailException("Manipulator talon mag encoder not found", LogLevel.WARNING)
            } catch (e: SelfCheckFailException) {
                DriverStation.reportError(e.reason, true)
            }
            return false
        }
        Logger.logInfo("Maniplator selfCheckup", LogLevel.INFO, "selfCheckup Passed!")
        return true
    }

    override fun selfTest(): Boolean {
        return true
    }

    fun rehome() {
        talon.sensorCollection.setQuadraturePosition(((talon.sensorCollection.pulseWidthPosition and 0xFFF) - ABSO_OFFSET).roundToInt(), 0)
    }
}