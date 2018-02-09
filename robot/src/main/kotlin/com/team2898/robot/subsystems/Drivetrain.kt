package com.team2898.robot.subsystems

import com.ctre.phoenix.motorcontrol.*
import com.team2898.engine.async.util.go
import com.team2898.engine.extensions.Vector2D.get
import com.team2898.engine.extensions.Vector2D.times
import com.team2898.engine.logic.*
import com.team2898.robot.config.RobotMap.*
import com.team2898.engine.motion.DriveSignal
import com.team2898.engine.motion.TalonWrapper
import com.team2898.robot.config.DrivetrainConf.*
import kotlinx.coroutines.experimental.delay
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
                (leftMaster.getSelectedSensorVelocity(0).toDouble() / 409.6) * 6 * Math.PI, // 1 rot/sec is 41 enc units
                (rightMaster.getSelectedSensorVelocity(0).toDouble() / 409.6) * 6 * Math.PI // 6" wheels,
        )

    init {
    }

    val encPosIn
        get() = Vector2D(
                ((leftMaster.getSelectedSensorPosition(0).toDouble()) / 4096) * 6 * Math.PI,
                ((rightMaster.getSelectedSensorPosition(0).toDouble()) / 4096) * 6 * Math.PI
        )

    val encVelRaw
        get() = Vector2D(
                leftMaster.getSelectedSensorVelocity(0).toDouble(),
                rightMaster.getSelectedSensorVelocity(0).toDouble()
        )

    val encPosFt
        get() = Vector2D(
                encPosIn[0] / 12.toDouble(), // inches/sec/12 -> feet/sec
                encPosIn[1] / 12.toDouble() // inches/sec/12 -> feet/sec
        )

    val encPosRaw
        get() = Vector2D(
                leftMaster.getSelectedSensorPosition(0).toDouble(),
                rightMaster.getSelectedSensorPosition(0).toDouble()
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

//            configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 0)
//            configVelocityMeasurementWindow(32, 0)
            configReverseSoftLimitEnable(false, 10)
            configForwardSoftLimitEnable(false, 10)
            configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 10)

            setPID(Kp, Ki, Kd, Kf)

            setControlFramePeriod(ControlFrame.Control_3_General, 10)

            setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 0)
            setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, 0)
        }
        rightMaster.setSensorPhase(true)
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

    fun zeroEncoders() {
        masters {
            sensorCollection.setQuadraturePosition(0, 0)
        }
    }
}