package com.team2898.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.team2898.engine.extensions.Vector2D.get
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.ILooper
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.motion.TalonWrapper
import com.team2898.robot.config.IntakeConf.*
import edu.wpi.first.wpilibj.DoubleSolenoid
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

    //val deployEncVelFtSec: Vector2D
    //    get() = Vector2D(
    //            (leftDeployTalon.sensorCollection.quadratureVelocity.toDouble() / 409.6), // TODO
    //            (rightDeployTalon.sensorCollection.quadratureVelocity.toDouble() / 409.6)
    //    )

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

                ((sensorCollection.pulseWidthPosition and 0xFFF) - ABSO_OFFSET).roundToInt()

            }
        }
    }

    override fun onStart() {
        listOf(leftDeployTalon, rightDeployTalon).forEach {
            it.apply {
                ((sensorCollection.pulseWidthPosition and 0xFFF) - ABSO_OFFSET).roundToInt()
            }
        }
    }

}

