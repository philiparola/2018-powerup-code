package com.team2898.robot.commands

import com.team2898.engine.logging.LogLevel
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.reflectLocation
import com.team2898.engine.motion.DriveSignal
import com.team2898.robot.motion.pathfinder.ProfileExecutor
import com.team2898.robot.motion.pathfinder.ProfileGenerator
import com.team2898.robot.motion.pathfinder.baselineProfile
import com.team2898.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.Command
import kotlinx.coroutines.experimental.async


class MotionProfileTest : Command() {

    val startTime: Double by lazy { Timer.getFPGATimestamp() }

    val executer: ProfileExecutor

    init {
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Starting profile generation at $startTime")
        executer = ProfileExecutor(
                ProfileGenerator.deferProfile(baselineProfile).apply {
                    invokeOnCompletion {
                        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Finished profile, dt of ${Timer.getFPGATimestamp() - startTime}")
                    }
                }
        )
    }

    override fun start() {
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Running motion profile")
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        executer.execute { left, right ->
            Drivetrain.openLoopPower = DriveSignal(left, right)
        }
    }

    override fun end() {
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Ending motion profile")
    }

    override fun isFinished(): Boolean =
            executer.completed

}