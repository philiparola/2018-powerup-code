package com.team2898.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.team2898.engine.logging.LogLevel
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.SelfCheckFailException
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.ILooper
import com.team2898.engine.logic.ISelfCheck
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.math.clamp
import com.team2898.engine.motion.TalonWrapper
import com.team2898.robot.config.ElevatorConf.*
import com.team2898.robot.config.RobotMap.ELEV_MASTER_CANID
import com.team2898.robot.config.RobotMap.ELEV_SLAVE1_CANID
import edu.wpi.first.wpilibj.DriverStation
import kotlin.math.PI
import kotlin.math.roundToInt

object Elevator : Subsystem(name = "Elevator", loopHz = 50.0), ISelfCheck, ILooper {
    override val enableTimes: List<GamePeriods> = listOf(GamePeriods.TELEOP, GamePeriods.AUTO)

    val master = TalonWrapper(ELEV_MASTER_CANID)
    val slaves = listOf(
            TalonWrapper(ELEV_SLAVE1_CANID)
//            TalonWrapper(ELEV_SLAVE2_CANID),
//            TalonWrapper(ELEV_SLAVE3_CANID)
    ).apply { forEach { it slaveTo master } }

    var targetPosFt: Double = 0.0
        set(value) {
            field = clamp(value, min= MIN_HEIGHT_FT, max= MAX_HEIGHT_FT)
            master.set(ControlMode.MotionMagic, ftToEncPos(field).toDouble())
        }


    val currentPosFt
        get() = encPosToFt(master.getSelectedSensorPosition(0)) * 2

    fun ftSecToEncVel(vel: Double): Int {
        return (vel * 12.0 *// ft/s -> in/sec
                1 / (SPROCKET_PITCH_DIA * PI) * // in/s -> rot/sec
                4096 * // rot/sec -> tick/sec
                1 / 10.0 // tick/sec -> STU
                ).roundToInt()
    }

    fun encVelToFtSec(vel: Int): Double {
        return vel * 10.0 * // STU -> tick/sec
                1 / 4096.0 * // tick/sec -> rot/sec
                (SPROCKET_PITCH_DIA * PI) *  // rot/sec -> in/sec
                1 / 12.0 // in/sec -> ft/sec
    }

    fun ftToEncPos(ft: Double): Int {
        return (ft / 2 * 12.0 * 1 / (SPROCKET_PITCH_DIA * PI) * 4096.0).roundToInt()
    }

    fun encPosToFt(tick: Int): Double {
        return (tick.toDouble() * (1.0 / 4096.0) * (SPROCKET_PITCH_DIA * PI) / 12.0)
    }

    init {
        master.apply {
            setMagEncoder()
            configPeakCurrentLimit(ELEV_PEAK_MAX_AMPS, 0)
            configContinuousCurrentLimit(ELEV_CONT_MAX_AMPS, 0)
            configPeakCurrentDuration(ELEV_PEAK_MAX_AMPS_DUR_MS, 0)
            enableCurrentLimit(ELEV_CURRENT_LIMIT)

            setPID(ELEV_Kp, ELEV_Ki, ELEV_Kd, ELEV_Kf)

            configMotionCruiseVelocity(ELEV_MAX_VEL.roundToInt(), 0)
            configMotionAcceleration(ELEV_MAX_ACC.roundToInt(), 0)

            setSelectedSensorPosition(0, 0, 0)
        }
    }

    override fun selfCheckup(): Boolean {
        if (master.pwmPos == 0) {
            try {
                throw SelfCheckFailException("Mag encoder not found!", LogLevel.WARNING)
            } catch (e: SelfCheckFailException) {
                DriverStation.reportError(e.reason, true)
            }
            return false
        }
        Logger.logInfo("Elevator selfCheckup", LogLevel.INFO, "selfCheckup Passed!")
        return true
    }

    override fun selfTest(): Boolean {
        return true
    }
}