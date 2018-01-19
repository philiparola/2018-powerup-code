package com.team2898.robot.subsystems

import com.ctre.phoenix.ErrorCode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod
import com.team2898.engine.logic.*
import com.team2898.robot.config.RobotMap.*
import com.team2898.engine.motion.DriveSignal
import com.team2898.engine.motion.TalonWrapper
import com.team2898.robot.config.DrivetrainConf.*
import edu.wpi.first.wpilibj.DoubleSolenoid
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D

object Drivetrain : Subsystem(50.0, "Drivetrain") {

    enum class ControlModes { OPEN_LOOP, BASE_LOCK, VELOCITY_DRIVE }

    val leftMaster = TalonWrapper(LEFT_MASTER_CANID)
    val rightMaster = TalonWrapper(RIGHT_MASTER_CANID)
    val leftSlave = TalonWrapper(LEFT_SLAVE_CANID)
    val rightSlave = TalonWrapper(RIGHT_SLAVE_CANID)
    val masterList = listOf(leftMaster, rightMaster)

    val encVelInSec
        get() = Vector2D(
                (leftMaster.sensorCollection.quadratureVelocity.toDouble()/409)*6*Math.PI, // 1 rot/sec is 41 enc units
                (rightMaster.sensorCollection.quadratureVelocity.toDouble()/409)*6*Math.PI // 6" wheels,
        )
    val encPosIn
        get() = Vector2D(
                ((leftMaster.sensorCollection.quadraturePosition.toDouble()) / 4096) * 6 * Math.PI,
                ((rightMaster.sensorCollection.quadraturePosition.toDouble()) / 4096) * 6 * Math.PI
        )

    val encVelRaw
        get() = Vector2D(
                leftMaster.sensorCollection.quadratureVelocity.toDouble(),
                rightMaster.sensorCollection.quadratureVelocity.toDouble()
        )
    val encPosRaw
        get() = Vector2D(
                leftMaster.sensorCollection.quadraturePosition.toDouble(),
                rightMaster.sensorCollection.quadraturePosition.toDouble()
        )

    override val enableTimes = listOf(GamePeriods.TELEOP, GamePeriods.AUTO)

    var baseLockPos = Vector2D(0.0, 0.0)

    // -1 to 1, vbat
    var openLoopPower = DriveSignal.BRAKE
    var closedLoopVelTarget = DriveSignal.NEUTRAL

    // Manages control modes
    val driveStateMachine = StateMachine()

    var controlMode = ControlModes.OPEN_LOOP
        set(value: ControlModes) {
            if (field != value) {
                driveStateMachine changeStateTo value
                field = value
            }
        }

    override fun onStart() {}

    override fun onLoop() =
            driveStateMachine.update()

    override fun onStop() {}

    init {
        // Set slaves
        leftSlave slaveTo leftMaster
        rightSlave slaveTo rightMaster

        // Set master characteristics
        masters {
            enableVoltageCompensation(true)
            configVoltageCompSaturation(12.0, 0)

            setNeutralMode(NeutralMode.Coast)

            setMagEncoder()

            configContinuousCurrentLimit(CONT_MAX_AMPS, 0)
            configPeakCurrentLimit(PEAK_MAX_AMPS, 0)
            configPeakCurrentDuration(PEAK_MAX_AMPS_DUR_MS, 0)
            enableCurrentLimit(CURRENT_LIMIT)

            configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 0)
            configVelocityMeasurementWindow(32, 0)

            setPID(Kp, Ki, Kd, Kf)
        }

        driveStateMachine.apply {
            registerWhile(ControlModes.OPEN_LOOP) {
                masters {
                }
                leftMaster.setOpenLoop(openLoopPower.left)
                rightMaster.setOpenLoop(-openLoopPower.right)
            }
            registerTo(ControlModes.OPEN_LOOP) {
                masters { setOpenLoop() }
            }
            registerFrom(ControlModes.OPEN_LOOP) { openLoopPower = DriveSignal.BRAKE }
        }

        driveStateMachine.changeStateTo(controlMode)
    }

    fun masters(block: TalonWrapper.() -> Unit) {
        masterList.forEach { srx ->
            srx.block()
        }
    }

    override fun selfCheckup(): Boolean {
        return false
    }

    override fun selfTest(): Boolean {
        return false
    }
}