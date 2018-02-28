package com.team2898.robot

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.extensions.Vector2D.get
import com.team2898.engine.logging.LogLevel
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.reflectLocation
import com.team2898.engine.logic.LoopManager
import com.team2898.robot.commands.ProfileFollower
import com.team2898.robot.commands.Teleop
import com.team2898.robot.config.RobotConf.SCHEDULER_HZ
import com.team2898.robot.motion.pathfinder.*
import com.team2898.engine.extensions.Vector2D.*
import edu.wpi.first.wpilibj.command.Scheduler
import com.team2898.robot.subsystems.*
import com.team2898.robot.motion.pathfinder.ProfilesSettings.*
import edu.wpi.first.wpilibj.CameraServer
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import openrio.powerup.MatchData

class Robot : TimedRobot() {
    companion object {
        val debug = true
        val start = AutoManager.StartLocations.LEFT
        val target = AutoManager.TargetAutos.SWITCH
    }
    val teleopCommand = Teleop()


    override fun robotInit() {
        Drivetrain.zeroEncoders()

        AsyncLooper(100.0) {
            SmartDashboard.putNumber("NavX yaw", Navx.yaw)
            SmartDashboard.putNumber("intake angle 1", Intake.currentPos.first.degrees)
            SmartDashboard.putNumber("intake angle 2", Intake.currentPos.second.degrees)
            SmartDashboard.putNumber("intake left sin", Intake.currentPos.first.sin)
            SmartDashboard.putNumber("intake left cos", Intake.currentPos.first.cos)
            SmartDashboard.putNumber("intake right sin", Intake.currentPos.second.sin)
            SmartDashboard.putNumber("intake right cos", Intake.currentPos.second.cos)
            SmartDashboard.putNumber("intake left pwm pos", Intake.leftDeployTalon.sensorCollection.pulseWidthPosition.toDouble())
            SmartDashboard.putNumber("intake right pwm pos", Intake.rightDeployTalon.sensorCollection.pulseWidthPosition.toDouble())
            SmartDashboard.putNumber("elev height", Elevator.currentPosFt)
            SmartDashboard.putNumber("manip angle", Manipulator.currentPos().degrees)
            SmartDashboard.putNumber("manip pwm pos", Manipulator.talon.sensorCollection.pulseWidthPosition.toDouble())
            SmartDashboard.putNumber("manip cos", Manipulator.currentPos().cos)
            SmartDashboard.putNumber("manip sin", Manipulator.currentPos().sin)
        }.start()

        SmartDashboard.putString("Session UUID", Logger.uuid)

        Intake.rehome()
        Manipulator.rehome()
        AsyncLooper(SCHEDULER_HZ) {
            Scheduler.getInstance().run()
        }
    }

    override fun autonomousInit() {
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        LoopManager.onAutonomous()
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Autonomous Init")
        Navx.reset()
        AutoManager.produceAuto(start, target).commnad.start()

        if (teleopCommand.isRunning) teleopCommand.cancel()
    }

    override fun teleopInit() {
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Teleop Init")
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        LoopManager.onTeleop()

    }

    override fun disabledInit() {
        LoopManager.onDisable()
    }
}
