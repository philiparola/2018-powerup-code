package com.team2898.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.team2898.engine.extensions.Vector2D.get
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.logging.LogLevel
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.SelfCheckFailException
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.ILooper
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.motion.TalonWrapper
import com.team2898.robot.config.IntakeConf.*
import com.team2898.robot.config.RobotMap.INTAKE_LEFT
import com.team2898.robot.config.RobotMap.INTAKE_RIGHT
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Spark
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D
import kotlin.math.PI
import kotlin.math.roundToInt

object Intake : ILooper, Subsystem(50.0, "Intake") {
    override val enableTimes: List<GamePeriods> = listOf(GamePeriods.AUTO, GamePeriods.TELEOP)

    val leftDeployTalon = TalonWrapper(INTAKE_LEFT) // deployy
    val rightDeployTalon = TalonWrapper(INTAKE_RIGHT)

    val leftSpark = Spark(SPARK_LEFT) // spinning thingyyyy
    val rightSpark = Spark(SPARK_RIGHT)
    val leftPiston = DoubleSolenoid(LEFT_INTAKE_SOLENOID_FORWARD_ID, LEFT_INTAKE_SOLENOID_REVERSE_ID)
    val rightPiston = DoubleSolenoid(RIGHT_INTAKE_SOLENOID_FORWARD_ID, RIGHT_INTAKE_SOLENOID_REVERSE_ID)
    val pistons = listOf(leftPiston, rightPiston)


    enum class PistonState {
        OPEN,
        CLOSED
    }

    var pistonState = PistonState.CLOSED
        set(value) {
            if (field != value) {
                field = value
                when (field) {
                    PistonState.CLOSED -> pistons.forEach { it.set(DoubleSolenoid.Value.kForward) }
                    PistonState.OPEN -> pistons.forEach { it.set(DoubleSolenoid.Value.kReverse) }
                }
            }
        }


    val rotation2dToEncPos = { rotation2d: Rotation2d ->
        rotation2d.radians / 2 / PI * 4096
    }

    val encPosToRotation2d = { enc: Int ->
        Rotation2d.createFromRadians(enc / 4096 * 2 * PI)
    }

    var sparkTargetSpeed = Vector2D(0.0, 0.0)
        set(value) {
            field = value
            leftSpark.set(-field[0])
            rightSpark.set(field[1])
        }

    var talonTargetPos = Rotation2d(0.0, 1.0)
        set(value) {
            field = if (value.degrees > MAX_POS) Rotation2d.createFromDegrees(MAX_POS)
            else if (value.degrees < MIN_POS) Rotation2d.createFromDegrees(MIN_POS)
            else value
            listOf(leftDeployTalon, rightDeployTalon).forEach {
                it.set(ControlMode.Position, rotation2dToEncPos(field))
            }
        }

    val currentPos: Pair<Rotation2d, Rotation2d>
        get() = Pair(
                encPosToRotation2d(leftDeployTalon.sensorCollection.quadraturePosition),
                encPosToRotation2d(rightDeployTalon.sensorCollection.quadraturePosition)
        )


    init {
        listOf(leftDeployTalon, rightDeployTalon).forEach {
            it.apply {
                setMagEncoder()
                configPeakCurrentLimit(INTAKE_PEAK_MAX_AMPS, 0)
                configContinuousCurrentLimit(INTAKE_CONT_MAX_AMPS, 0)
                configPeakCurrentDuration(INTAKE_PEAK_MAX_AMPS_DUR_MS, 0)
                enableCurrentLimit(INTAKE_CURRENT_LIMIT)

                setPID(INTAKE_Kp, INTAKE_Ki, INTAKE_Kd)
                configMotionAcceleration(INTAKE_MAX_ACC, 0)
                configMotionCruiseVelocity(INTAKE_MAX_VEL, 0)
            }
        }
    }

    override fun onStart() {
        this.talonTargetPos = Rotation2d(1.0, 0.0)
    }

    fun rehome() {
        leftDeployTalon.sensorCollection.setQuadraturePosition(((leftDeployTalon.sensorCollection.pulseWidthPosition and 0xFFF) - ABSO_OFFSET_LEFT).roundToInt(), 0)
        rightDeployTalon.sensorCollection.setQuadraturePosition(((rightDeployTalon.sensorCollection.pulseWidthPosition and 0xFFF) - ABSO_OFFSET_RIGHT).roundToInt(), 0)
    }

    override fun selfCheckup(): Boolean {
        var fail = false
        if (leftDeployTalon.pwmPos == 0) {
            fail = true
            try {
                throw SelfCheckFailException("Left intake talon mag encoder now found", LogLevel.WARNING)
            } catch (e: SelfCheckFailException) {
                DriverStation.reportError(e.reason, true)
            }
        }
        if (rightDeployTalon.pwmPos == 0) {
            fail = true
            try {
                throw SelfCheckFailException("Right intake talon mag encoder now found", LogLevel.WARNING)
            } catch (e: SelfCheckFailException) {
                DriverStation.reportError(e.reason, true)
            }
        }
        if (!fail) Logger.logInfo("Intake selfCheckup", LogLevel.INFO, "selfCheckup Passed!")
        return !fail
    }

}

