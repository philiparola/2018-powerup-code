package com.team2898.robot

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.extensions.Vector2D.get
import com.team2898.engine.logging.LogLevel
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.reflectLocation
import com.team2898.engine.logic.LoopManager
import com.team2898.robot.commands.ProfileFollower
import com.team2898.robot.commands.Teleop
import com.team2898.robot.motion.pathfinder.*
import edu.wpi.first.wpilibj.command.Scheduler
import com.team2898.robot.subsystems.*
import com.team2898.robot.motion.pathfinder.*
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.CameraServer
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import jaci.pathfinder.Trajectory

class Robot : TimedRobot() {
    companion object {
        val debug = true
        val ntInstance = NetworkTableInstance.create()
    }
    val data = DriverStation.getInstance().gameSpecificMessage
    val profile = ProfileGenerator.deferProfile(
            ProfileSettings(
                    hz = 50,
                    maxVel = 3.0,
                    maxAcc = 2.0,
                    maxJerk = 5.0,
                    wheelbaseWidth = 2.2568170930430758,
                    wayPoints = convWaypoint(switchFromLeftToSide),
                    fitMethod = Trajectory.FitMethod.HERMITE_CUBIC,
                    sampleRate = Trajectory.Config.SAMPLES_HIGH
            )
    )
    val autoCommnad = ProfileFollower(Pair(profile.second, profile.first))

    val teleopCommand = Teleop()
    override fun robotInit() {
        Drivetrain.zeroEncoders()
        AsyncLooper(100.0) {
            SmartDashboard.putNumber("dt left vel", Drivetrain.encVelInSec[0] / 12.0)
            SmartDashboard.putNumber("dt right vel", Drivetrain.encVelInSec[1] / 12.0)
            SmartDashboard.putNumber("dt left pos", Drivetrain.encPosIn[0])
            SmartDashboard.putNumber("dt right pos", Drivetrain.encPosIn[1])
            SmartDashboard.putNumber("NavX yaw", Navx.yaw)
        }.start()

        SmartDashboard.putString("Session UUID", Logger.uuid)

        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
    }

    override fun autonomousInit() {
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        LoopManager.onAutonomous()
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Autonomous Init")
        Navx.reset()
        autoCommnad.start()

        if (teleopCommand.isRunning) teleopCommand.cancel()
    }


    override fun autonomousPeriodic() {
        Scheduler.getInstance().run()
    }

    override fun teleopInit() {
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Teleop Init")
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        LoopManager.onTeleop()
        teleopCommand.start()
    }

    override fun teleopPeriodic() {
        Scheduler.getInstance().run()
    }

    override fun disabledInit() {
        LoopManager.onDisable()
    }
}
