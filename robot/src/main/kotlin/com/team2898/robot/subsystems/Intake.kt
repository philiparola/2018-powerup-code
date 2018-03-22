package com.team2898.robot.subsystems

import com.team2898.engine.controlLoops.StandardPID
import com.team2898.engine.controlLoops.classicControl.PVAPID
import com.team2898.engine.controlLoops.modernControl.BasicStateSpaceController
import com.team2898.engine.kinematics.RigidTransform2d
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.kinematics.Translation2d
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.StateMachine
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.math.clamp
import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.get
import com.team2898.engine.motion.TalonWrapper
import com.team2898.engine.motion.TrapezoidProfile
import com.team2898.robot.config.IntakeConf.*
import com.team2898.robot.config.RobotMap.INTAKE_LEFT_CANID
import com.team2898.robot.config.RobotMap.INTAKE_RIGHT_CANID

/*
Treat each as 775pro w/ 10:1 reduction

 */

object Intake : Subsystem(50.0, "Intake") {
    override val enableTimes = listOf(GamePeriods.AUTO, GamePeriods.TELEOP)

    enum class FunctionModes { LIDAR_ASSIST, DEFAULT }
    enum class ControlModes { STATE_SPACE, PID, OPEN_LOOP }

    var profileSpeed = true
    var functionMode = FunctionModes.DEFAULT
        set(new) {
            field = new
            functionModeSM.changeStateTo(field)
        }
    var controlMode = ControlModes.OPEN_LOOP
        set(new) {
            field = new
            controlModeSM.changeStateTo(field)
        }

    var baselineSurfaceSpeed = 0.0 // in ft/s
    var manualSpeedBias = 0.0 // -1 to 1, operator controlled speed multiplier/override

    val surfaceSpeeds: Pair<Double, Double>
        get() = Pair(
                rpmToSurfaceSpeed(stuToRpm(leftTalon.getSelectedSensorVelocity(0).toDouble())),
                rpmToSurfaceSpeed(stuToRpm(rightTalon.getSelectedSensorVelocity(0).toDouble()))
        )
    val rpms: Pair<Double, Double>
        get() = Pair(
                stuToRpm(leftTalon.getSelectedSensorVelocity(0).toDouble()),
                stuToRpm(rightTalon.getSelectedSensorVelocity(0).toDouble())
        )

    private val leftTalon = TalonWrapper(INTAKE_LEFT_CANID)
    private val rightTalon = TalonWrapper(INTAKE_RIGHT_CANID)

    private fun masters(block: TalonWrapper.() -> Unit) = listOf(leftTalon, rightTalon).forEach { it.block() }

    private val functionModeSM = StateMachine()
    private val controlModeSM = StateMachine()

    private var targetLidarAlignSurfaceSpeed = Pair(0.0, 0.0)
    private var targetSurfaceSpeeds = Pair(0.0, 0.0)

    private val leftProfiler = TrapezoidProfile(maxAcc = 0.0, maxVel = 0.0)
    private val rightProfiler = TrapezoidProfile(maxAcc = 0.0, maxVel = 0.0)

    private val leftPID = StandardPID(Kp = INTAKE_Kp, Ki = INTAKE_Ki, Kd = INTAKE_Kd, Kf = INTAKE_Kf)
    private val rightPID = StandardPID(Kp = INTAKE_Kp, Ki = INTAKE_Ki, Kd = INTAKE_Kd, Kf = INTAKE_Kf)

    private enum class GainEnum { NO_CUBE, CUBE }

    private val leftStateSpace = BasicStateSpaceController<GainEnum>(numInputs = 1, numOutputs = 2, numStates = 2, schedule = GainEnum.NO_CUBE) { gain ->
        when (gain) {
            GainEnum.CUBE -> INTAKE_K_CUBE
            GainEnum.NO_CUBE -> INTAKE_K_NO_CUBE
        }
    }
    private val rightStateSpace = BasicStateSpaceController<GainEnum>(numInputs = 1, numOutputs = 2, numStates = 2, schedule = GainEnum.NO_CUBE) { gain ->
        when (gain) {
            GainEnum.CUBE -> INTAKE_K_CUBE
            GainEnum.NO_CUBE -> INTAKE_K_NO_CUBE
        }
    }


    init {
        masters {
            enableCurrentLimit(true)
            configContinuousCurrentLimit(INTAKE_CONT_MAX_AMPS, 0)
            configPeakCurrentDuration(INTAKE_PEAK_MAX_AMPS, 0)
            configPeakCurrentLimit(INTAKE_PEAK_MAX_AMPS_DUR_MS, 0)

            enableVoltageCompensation(true)
            configVoltageCompSaturation(12.0, 0)
        }
        functionModeSM.changeStateTo(functionMode)
        controlModeSM.changeStateTo(controlMode)

        functionModeSM.registerWhile(FunctionModes.LIDAR_ASSIST) {
            val cube = calcCubeOrientationDistance(VL53l0X.getDistances())
            if (cube.x > INTAKEN_CUBE_THRESHOLD) {
                // todo fix
                targetLidarAlignSurfaceSpeed =
                        Pair(
                                clamp(baselineSurfaceSpeed + cube.sin * (MAX_ORIENTATION_ADJ_PERCENT / 100) * baselineSurfaceSpeed,
                                        magnitude = MAX_SURFACE_SPEED),
                                clamp(baselineSurfaceSpeed - cube.sin * (MAX_ORIENTATION_ADJ_PERCENT / 100) * baselineSurfaceSpeed,
                                        magnitude = MAX_SURFACE_SPEED)
                        )
                targetSurfaceSpeeds = targetLidarAlignSurfaceSpeed
            }
        }

        functionModeSM.registerWhile(FunctionModes.DEFAULT) {
            targetSurfaceSpeeds =
                    Pair(clamp(
                            baselineSurfaceSpeed + (MAX_ORIENTATION_ADJ_PERCENT / 100) * baselineSurfaceSpeed * manualSpeedBias,
                            magnitude = MAX_SURFACE_SPEED
                    ), clamp(baselineSurfaceSpeed - (MAX_ORIENTATION_ADJ_PERCENT / 100) * baselineSurfaceSpeed * manualSpeedBias,
                            magnitude = MAX_SURFACE_SPEED))
        }


        controlModeSM.registerWhile(ControlModes.OPEN_LOOP) {
            leftTalon.setOpenLoop(rpmToVoltage(surfaceSpeedToRPM(if (profileSpeed) leftProfiler.targetPos else targetSurfaceSpeeds.first)) / 12.0)
            rightTalon.setOpenLoop(rpmToVoltage(surfaceSpeedToRPM(if (profileSpeed) rightProfiler.targetPos else targetSurfaceSpeeds.second)) / 12.0)
        }
        controlModeSM.registerWhile(ControlModes.PID) {
            leftPID.setpoint = surfaceSpeedToRPM(if (profileSpeed) leftProfiler.targetPos else targetSurfaceSpeeds.first)
            leftTalon.setOpenLoop(leftPID.update(rpms.first) / 12.0)
            rightPID.setpoint = surfaceSpeedToRPM(if (profileSpeed) rightProfiler.targetPos else targetSurfaceSpeeds.second)
            rightTalon.setOpenLoop(rightPID.update(rpms.first) / 12.0)
        }
        controlModeSM.registerWhile(ControlModes.STATE_SPACE) {
            leftTalon.setOpenLoop(leftStateSpace.update(
                    r = Matrix(arrayOf(doubleArrayOf(surfaceSpeedToRPM(if (profileSpeed) leftProfiler.targetPos else targetSurfaceSpeeds.first)))),
                    x = Matrix(arrayOf(doubleArrayOf(rpms.first)))
            )[0, 0] / 12.0
            )
            rightTalon.setOpenLoop(rightStateSpace.update(
                    r = Matrix(arrayOf(doubleArrayOf(surfaceSpeedToRPM(if (profileSpeed) rightProfiler.targetPos else targetSurfaceSpeeds.second)))),
                    x = Matrix(arrayOf(doubleArrayOf(rpms.first)))
            )[0, 0] / 12.0
            )
        }
    }


    override fun onLoop() {

    }

    // Todo: Actually calculate cube orientation
    private fun calcCubeOrientationDistance(lidarDistances: Triple<Double, Double, Double>): RigidTransform2d {
        return RigidTransform2d(Translation2d(0.0, 0.0), Rotation2d(1.0, 0.0))
    }

    private fun surfaceSpeedToRPM(feetPerSecond: Double): Double =
            (
                    feetPerSecond * 12 // ft/s -> in/s
                            / 4 * Math.PI // in/s -> rotation/s
                            * 60.0 // rotation/s -> rpm
                    )

    private fun rpmToSurfaceSpeed(rpm: Double): Double =
            (
                    rpm / 60.0 // rpm -> rotation/s
                            * 4 * Math.PI // rotation/s -> in/s
                            / 12.0 // in/s -> ft/s
                    )

    private fun stuToRpm(stu: Double): Double = (stu * 10.0) / 4096.0

    private fun rpmToVoltage(rpm: Double): Double =
            (rpm * 12.0) / (18730 * INTAKE_GEARING)

    private fun voltageToRpm(voltage: Double): Double =
            voltage / 12.0 * 18730 * INTAKE_GEARING
}