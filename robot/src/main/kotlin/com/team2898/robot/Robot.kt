package com.team2898.robot

import com.team2898.engine.logging.LogLevel
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.reflectLocation
import com.team2898.engine.logic.LoopManager
import com.team2898.robot.commands.Teleop
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.command.Scheduler
import com.team2898.robot.subsystems.*

class Robot : IterativeRobot() {
    companion object {
        val debug = true
    }

    val teleopCommand = Teleop()
    override fun robotInit() {
        //System.setProperty("java.util.concurrent.ForkJoinPool.common.parallelism", "2")
    }

    override fun autonomousInit() {
        LoopManager.onAutonomous()
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Autonomous Init")
        Navx.reset()
        //autoCommand.start()
        Drivetrain.gearMode = Drivetrain.GearModes.LOW
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP

        if (teleopCommand.isRunning) teleopCommand.cancel()
    }


    override fun autonomousPeriodic() {
        Scheduler.getInstance().run()
    }

    override fun teleopInit() {
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Teleop Init")
        Drivetrain.gearMode = Drivetrain.GearModes.LOW
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        LoopManager.onTeleop()

        //if (autoCommand.isRunning) autoCommand.cancel()
        teleopCommand.start()
    }

    override fun teleopPeriodic() {
        Scheduler.getInstance().run()
    }

    override fun disabledInit() {
        LoopManager.onDisable()
    }
}
