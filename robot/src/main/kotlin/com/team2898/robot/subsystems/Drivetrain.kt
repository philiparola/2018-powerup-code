package com.team2898.robot.subsystems

import com.ctre.phoenix.motorcontrol.*
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.StateMachine
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.motion.DriveSignal
import com.team2898.engine.motion.TalonWrapper
import com.team2898.robot.config.DrivetrainConf.*
import com.team2898.robot.config.RobotMap.*
import kotlin.math.abs
import kotlin.math.sign

object Drivetrain : Subsystem(50.0, "Drivetrain") {

    enum class ControlModes { OPEN_LOOP, BASE_LOCK, VELOCITY_DRIVE,MOTION_MAGIC }

    val leftMaster = TalonWrapper(LEFT_MASTER_CANID)
    val rightMaster = TalonWrapper(RIGHT_MASTER_CANID)
    val leftSlave = TalonWrapper(LEFT_SLAVE_CANID)
    val rightSlave = TalonWrapper(RIGHT_SLAVE_CANID)
    val masterList = listOf(leftMaster, rightMaster)

    val encVelInSec
        get() = Pair<Double, Double>(
                (leftMaster.getSelectedSensorVelocity(0).toDouble() / 409.6) * 6 * Math.PI,
                (rightMaster.getSelectedSensorVelocity(0).toDouble() / 409.6) * 6 * Math.PI
        )

    val encPosIn
        get() = Pair<Double, Double>(
                ((leftMaster.getSelectedSensorPosition(0).toDouble()) / 4096) * 6 * Math.PI,
                ((rightMaster.getSelectedSensorPosition(0).toDouble()) / 4096) * 6 * Math.PI
        )

    val encPosFt
        get() = Pair<Double, Double>(
                encPosIn.first / 12, // inches/sec/12 -> feet/sec
                encPosIn.second / 12 // inches/sec/12 -> feet/sec
        )

    val encVelRaw
        get() = Pair<Int, Int>(
                leftMaster.getSelectedSensorVelocity(0),
                rightMaster.getSelectedSensorVelocity(0)
        )

    val encPosRaw
        get() = Pair<Int, Int>(
                leftMaster.getSelectedSensorPosition(0),
                rightMaster.getSelectedSensorPosition(0)
        )

    override val enableTimes = listOf(GamePeriods.TELEOP, GamePeriods.AUTO)


    var openLoopPower = DriveSignal.BRAKE
    var uncorrectedOpenLoopPower = openLoopPower
        get() = openLoopPower
        set(uncor) {
            val leftOpen = uncor.left
            val rightOpen = uncor.right
            val leftCorr =
                    ((if (abs(leftOpen * 14.6) > 0.1) abs(leftOpen * 14.6) - LEFT_B else 0.0) / LEFT_M) / 12.0 * sign(leftOpen)
            val rightCorr =
                    ((if (abs(rightOpen * 14.6) > 0.1) abs(rightOpen * 14.6) - RIGHT_B else 0.0) / RIGHT_M) / 12.0 * sign(rightOpen)
            field = uncor
            openLoopPower = uncor.copy(left = leftCorr, right = rightCorr)
        }

    var mmDistance = DriveSignal.BRAKE

    var closedLoopVelTarget = DriveSignal.NEUTRAL

    var baseLockTarget = Pair(0, 0)

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

            setControlFramePeriod(ControlFrame.Control_3_General, 10)

            setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 0)
            setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, 0)

        }
        leftMaster.configMotionCruiseVelocity(L_MM_CRUISE_STU,10)
        rightMaster.configMotionCruiseVelocity(R_MM_CRUISE_STU,10)
        leftMaster.configMotionAcceleration(L_MM_ACC_STU2,10)
        rightMaster.configMotionAcceleration(R_MM_ACC_STU2,10)

        rightMaster.setSensorPhase(true) // test bot

        driveStateMachine.apply {
            registerWhile(ControlModes.OPEN_LOOP) {
                leftMaster.setOpenLoop(openLoopPower.left)
                rightMaster.setOpenLoop(-openLoopPower.right)
            }
            registerFrom(ControlModes.OPEN_LOOP) { openLoopPower = DriveSignal.BRAKE }

            registerTo(ControlModes.BASE_LOCK) {
                baseLockTarget = encPosRaw
                masters {
                    setPID(LOCK_KP, LOCK_KI, LOCK_KD, 0.0, 1, 128)
                }
            }
            registerWhile(ControlModes.BASE_LOCK) {
                leftMaster.setPositionControl(baseLockTarget.first.toDouble())
                rightMaster.setPositionControl(baseLockTarget.second.toDouble())
            }
            registerFrom(ControlModes.BASE_LOCK) {
                masters {
                    setPID(Kp, Ki, Kd, Kf, 1, 128)
                }
            }
            registerWhile(ControlModes.MOTION_MAGIC) {
                leftMaster.set(ControlMode.MotionMagic, mmDistance.left)
                rightMaster.set(ControlMode.MotionMagic, mmDistance.right)
            }
        }

        driveStateMachine.changeStateTo(controlMode)
    }

    fun masters(block: TalonWrapper.() -> Unit) {
        masterList.forEach { srx ->
            srx.block()
        }
    }

    override fun selfCheckup(): Boolean {
        return true
    }

    override fun selfTest(): Boolean {
        return true
    }

    fun zeroEncoders() {
        masters {
            sensorCollection.setQuadraturePosition(0, 0)
        }
    }
}