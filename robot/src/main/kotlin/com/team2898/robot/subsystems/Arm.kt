package com.team2898.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import com.team2898.engine.async.util.go
import com.team2898.engine.controlLoops.classicControl.PVAPID
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.motion.TalonWrapper
import com.team2898.engine.motion.TrapezoidProfile
import com.team2898.robot.config.ArmConf.*
import com.team2898.robot.config.RobotMap.ARM_MASTER_CANID
import com.team2898.robot.config.RobotMap.ARM_SLAVE1_CANID

object Arm : Subsystem(50.0, "Arm") {
    override val enableTimes = listOf(GamePeriods.AUTO, GamePeriods.TELEOP)

    val masterTalon = TalonWrapper(ARM_MASTER_CANID)
    val slave1 = TalonWrapper(ARM_SLAVE1_CANID)


    val profiler = TrapezoidProfile(maxVel = CRUISE_SPEED, maxAcc = MAX_ACC)

    val rotation: Rotation2d
        get() = Rotation2d.createFromDegrees(armDegrees)

    val armDegrees // 0 degrees is horizontal
        get() = (masterTalon.getSelectedSensorPosition(0) / 4096.0) * 360.0

    val armVel // deg/s
        get() = stuToDegS(masterTalon.getSelectedSensorVelocity(0))

    var targetRotation: Rotation2d = Rotation2d(0.0, 1.0)
        set(new) {
            field = new
            profiler.updateTarget(field.degrees)
        }

    // Input deg, deg/s, deg/s^2, output volts
    val armPVA = PVAPID(
            Kpp = ARM_Kpp, Kpi = ARM_Kpi, Kpf = { ARM_HORIZ_VOLTAGE * rotation.cos },
            Kvp = 0.0, Kvf = { vel -> vel * ARM_Kvf },
            Kaf = 0.0,
            minOutput = -12.0, maxOutput = 12.0)


    init {
        slave1.apply {
            enableCurrentLimit(true)
            configContinuousCurrentLimit(ARM_CONT_MAX_AMPS, 10)
            configPeakCurrentDuration(ARM_PEAK_MAX_AMPS, 10)
            configPeakCurrentLimit(ARM_PEAK_MAX_AMPS_DUR_MS, 10)
            configNeutralDeadband(0.001, 10)
            this slaveTo masterTalon
            setMagEncoder()
        }
        masterTalon.apply {
            configNeutralDeadband(0.001, 10)
            enableCurrentLimit(true)
            configContinuousCurrentLimit(ARM_CONT_MAX_AMPS, 10)
            configPeakCurrentDuration(ARM_PEAK_MAX_AMPS, 10)
            configPeakCurrentLimit(ARM_PEAK_MAX_AMPS_DUR_MS, 10)
            enableVoltageCompensation(true)
            configVoltageCompSaturation(12.0, 10)

            setMagEncoder()
            setSensorPhase(true)

            setSelectedSensorPosition((pwmPos + ARM_PWM_OFFSET), 0, 10)
            sensorCollection.setQuadraturePosition((pwmPos + ARM_PWM_OFFSET), 10)
            //go {
            //    while (getSelectedSensorPosition(0) < 0) {
            //        setSelectedSensorPosition(pwmPos + ARM_PWM_OFFSET, 0, 10)
            //        sensorCollection.setQuadraturePosition(Math.abs(pwmPos + ARM_PWM_OFFSET), 10)
            //        println("unfucking arm")
            //    }
            //}
            //setSelectedSensorPosition(
            //        pwmPos, 0, 10)

            setPID(TALON_Kp, TALON_Ki, TALON_Kd, TALON_Kf, 0, 400)
            configMotionAcceleration(MM_MAX_ACC, 10)
            configMotionCruiseVelocity(MM_CRUISE_SPEED, 10)
            set(ControlMode.MotionMagic, degreesToTicks(targetRotation.degrees), DemandType.ArbitraryFeedForward, TALON_Kf_ADD * rotation.cos)

            configForwardSoftLimitEnable(true,10)
            configForwardSoftLimitThreshold(1878,10)
            configReverseSoftLimitEnable(true,10)
            configReverseSoftLimitThreshold(-134,10)

        }

        slave1.apply {
            enableCurrentLimit(true)
            configContinuousCurrentLimit(ARM_CONT_MAX_AMPS, 10)
            configPeakCurrentDuration(ARM_PEAK_MAX_AMPS, 10)
            configPeakCurrentLimit(ARM_PEAK_MAX_AMPS_DUR_MS, 10)
        }
    }

    override fun onStart() {
        masterTalon.apply {
            setSelectedSensorPosition(pwmPos + ARM_PWM_OFFSET, 0, 10)
            sensorCollection.setQuadraturePosition(Math.abs(pwmPos + ARM_PWM_OFFSET), 10)
            set(ControlMode.MotionMagic, degreesToTicks(armDegrees), DemandType.ArbitraryFeedForward, TALON_Kf_ADD * rotation.cos)
        }
    }

    override fun onLoop() {
        //val pvaData = profiler.update()
        //val output = armPVA.update(armDegrees, armVel, pvaData.targetPos, pvaData.targetPos, pvaData.targetAcc) // In v
        //val output = armPVA.update(armDegrees, armVel, targetRotation.degrees, 0.0, 0.0) // In v
        //masterTalon.setOpenLoop(-output / 12.0)
        masterTalon.set(ControlMode.MotionMagic, degreesToTicks(targetRotation.degrees), DemandType.ArbitraryFeedForward, TALON_Kf_ADD * rotation.cos)
    }

    override fun onStop() {
        masterTalon.setOpenLoop(0.0)
    }

    private fun stuToRpm(stu: Int): Double = ((stu * 10.0) / 4096.0) * 60.0
    private fun stuToDegS(stu: Int) = (stuToRpm(stu) / 60.0) * 360.0

    private fun degreesToTicks(degrees: Double) = (degrees / 360.0) * 4096.0

    override fun selfCheckup(): Boolean {
        if (masterTalon.getSelectedSensorPosition(10) == 0) return false
        return true
    }

    fun onTarget() = masterTalon.getClosedLoopError(0) < degreesToTicks(15.0)

    fun fixEncoder() = masterTalon.setSelectedSensorPosition(Math.abs(masterTalon.pwmPos + ARM_PWM_OFFSET), 0, 10)

}
