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
    }

    val profile = ProfileGenerator.deferProfile(rightSwitchFromCenter)

    val cameraServer = CameraServer.getInstance()
    val cameras = listOf(0, 1, 2)

    var index = 0

    val autoCommnad = ProfileFollower(Pair(profile.second, profile.first))

    val teleopCommand = Teleop()

    override fun robotInit() {
        Drivetrain.zeroEncoders()

        AsyncLooper(100.0) {
            cameraServer.startAutomaticCapture(cameras[index])
            if (OI.operatorController.getRawButton(7)) {
                index++
                if (index == cameras.size) index = 0
            }
            if (OI.operatorController.getRawButton(8)) {
                index--
                if (index == -1) index = cameras.size - 1
            }
            SmartDashboard.putNumber("dt left vel", Drivetrain.encVelInSec[0] / 12.0)
            SmartDashboard.putNumber("dt right vel", Drivetrain.encVelInSec[1] / 12.0)
            SmartDashboard.putNumber("dt left pos", Drivetrain.encPosIn[0])
            SmartDashboard.putNumber("dt right pos", Drivetrain.encPosIn[1])
            SmartDashboard.putNumber("NavX yaw", Navx.yaw)
        }.start()

        SmartDashboard.putString("Session UUID", Logger.uuid)

        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP

        AsyncLooper(SCHEDULER_HZ) {
            Scheduler.getInstance().run()
        }
    }

    override fun autonomousInit() {
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        LoopManager.onAutonomous()
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Autonomous Init")
        Navx.reset()

        autoCommnad.start()

        val switchSide = MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR)
        val scaleSide = MatchData.getOwnedSide(MatchData.GameFeature.SCALE)

        if (teleopCommand.isRunning) teleopCommand.cancel()
    }

    override fun autonomousPeriodic() {
    }


    override fun teleopInit() {
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Teleop Init")
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        LoopManager.onTeleop()
        teleopCommand.start()
    }

    override fun teleopPeriodic() {
    }

    override fun disabledInit() {
        LoopManager.onDisable()
    }
}
