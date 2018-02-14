package com.team2898.robot.subsystems

import com.team2898.engine.extensions.Vector2D.get
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.ILooper
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.motion.TalonWrapper
import com.team2898.robot.config.IntakeConf.*
import edu.wpi.first.wpilibj.Spark
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D

object Intake : ILooper, Subsystem(100.0, "Intake") {
    override val enableTimes: List<GamePeriods> = listOf(GamePeriods.AUTO, GamePeriods.TELEOP)
    val i = Rotation2d
    val leftDeployTalon = TalonWrapper(INTAKE_MASTER) // deployy
    val rightDeployTalon = TalonWrapper(INTAKE_SLAVE)
    val leftSpark = Spark(LEFT_SPARK) // spinning thingyyyy
    val rightSpark = Spark(RIGHT_SPARK)

    val deployEncVelRaw: Vector2D
        get() = Vector2D(
                leftDeployTalon.sensorCollection.quadratureVelocity.toDouble(),
                rightDeployTalon.sensorCollection.quadratureVelocity.toDouble()
        )

    val deployEncVelFtSec: Vector2D
        get() = Vector2D(
                (leftDeployTalon.sensorCollection.quadratureVelocity.toDouble() / 409.6), // TODO * circ of the gear or something
                (rightDeployTalon.sensorCollection.quadratureVelocity.toDouble() / 409.6)
        )

    var talonTargetSpeed = Vector2D(0.0, 0.0)
        set(value) {
            field = value
            leftDeployTalon.set(field[0])
            rightDeployTalon.set(field[1])
        }

    var sparkTargetSpeed = Vector2D(0.0, 0.0)
        set(value) {
            field = value
            leftSpark.set(field[0])
            rightSpark.set(field[1])
        }

    val talons = listOf(leftDeployTalon, rightDeployTalon)

    init {
        talons.forEach {
            it.apply {
                setMagEncoder()
                configPeakCurrentLimit(INTAKE_PEAK_MAX_AMPS, 0)
                configContinuousCurrentLimit(INTAKE_CONT_MAX_AMPS, 0)
                configPeakCurrentDuration(INTAKE_PEAK_MAX_AMPS_DUR_MS, 0)
                enableCurrentLimit(INTAKE_CURRENT_LIMIT)

                setPID(INTAKE_Kp, INTAKE_Ki, INTAKE_Kd)
            }
        }
    }

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