package com.team2898.robot

import com.sun.org.apache.xpath.internal.operations.Bool
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
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.logic.SelfCheckManager
import com.team2898.engine.motion.DriveSignal
import com.team2898.robot.commands.auto.Baseline
import com.team2898.robot.commands.auto.WaitAuto
import edu.wpi.first.wpilibj.command.Scheduler
import com.team2898.robot.subsystems.*
import com.team2898.robot.motion.pathfinder.ProfilesSettings.*
import edu.wpi.first.wpilibj.CameraServer
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import openrio.powerup.MatchData

class Robot : TimedRobot() {
    companion object {
        val debug = true
    }

    val teleopCommand = Teleop()
    val auto = Baseline()

    var found = false
    val startChooser = SendableChooser<AutoManager.StartLocations>()
    val targetChooser = SendableChooser<AutoManager.TargetAutos>()
    val secChooser = SendableChooser<Double>()

    override fun robotInit() {
        SelfCheckManager.checkAll()
        Drivetrain.zeroEncoders()

        CameraServer.getInstance().startAutomaticCapture(0)
        AsyncLooper(25.0) {
            SmartDashboard.putNumber("NavX yaw", Navx.yaw)
            SmartDashboard.putNumber("intake left cos", Intake.currentPos.cos)
            SmartDashboard.putNumber("intake right sin", Intake.currentPos.sin)
            SmartDashboard.putNumber("current intake degrees", Intake.currentPos.degrees)
            SmartDashboard.putNumber("voltage", Intake.leftDeployTalon.motorOutputVoltage)
            SmartDashboard.putNumber("target pos", Intake.talonTargetPos.degrees)
            SmartDashboard.putNumber("tatget enc pos", Intake.rotation2dToEncPos(Intake.talonTargetPos))
        }.start()

        SmartDashboard.putString("Session UUID", Logger.uuid)

        Intake.rehome()
//        Manipulator.rehome()

//        pushChoosers()
    }

    override fun autonomousInit() {
//        var tries = 100
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        LoopManager.onAutonomous()
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Autonomous Init")
        Navx.reset()
        auto.start()

//        val start = startChooser.selected
//        val target = targetChooser.selected
//        val waitSecond = secChooser.selected
//
//        if (teleopCommand.isRunning) teleopCommand.cancel()
//
//        while (!found && tries >= 0) {
//            Logger.logInfo("Auto init", LogLevel.WARNING, "Match data still not found")
//            if (DriverStation.getInstance().gameSpecificMessage != null && DriverStation.getInstance().gameSpecificMessage.length == 3) found = true
//            tries--
//        }
//
//        if (found) {
//            WaitAuto(AutoManager.produceAuto(start, target).commnad, waitSecond).start()
//        } else {
//            DriverStation.reportWarning("Executing baseline command()", false)
//            WaitAuto(Baseline(), waitSecond).start()
//        }
    }

    override fun teleopInit() {
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Teleop Init")
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        LoopManager.onTeleop()
        teleopCommand.start()
    }

    override fun disabledInit() {
        LoopManager.onDisable()
    }
    override fun autonomousPeriodic() {
            Scheduler.getInstance().run()}
    override fun teleopPeriodic() {
            Scheduler.getInstance().run()}

    override fun disabledPeriodic() {
        if (!found) {
            if (DriverStation.getInstance().gameSpecificMessage != null || DriverStation.getInstance().gameSpecificMessage.length == 3) {
                found = true
                Logger.logInfo("Disabled Periodic", LogLevel.INFO, "Match Data found")
            }
        }
            Scheduler.getInstance().run()
    }

    fun pushChoosers() {
        startChooser.addObject("left", AutoManager.StartLocations.LEFT)
        startChooser.addObject("center", AutoManager.StartLocations.CENTER)
        startChooser.addObject("right", AutoManager.StartLocations.RIGHT)
        SmartDashboard.putData("auto starting", startChooser)

        targetChooser.addObject("Switch", AutoManager.TargetAutos.SWITCH)
        targetChooser.addObject("Scale", AutoManager.TargetAutos.SCALE)
        targetChooser.addObject("Base line", AutoManager.TargetAutos.BASE_LINE)
        SmartDashboard.putData("auto target", targetChooser)

        secChooser.addDefault("zero", 0.0)
        secChooser.addObject("one", 1.0)
        secChooser.addObject("two", 2.0)
        secChooser.addObject("three", 3.0)
        secChooser.addObject("four", 4.0)
        secChooser.addObject("five", 5.0)
        secChooser.addObject("six", 6.0)
        secChooser.addObject("seven", 7.0)
        secChooser.addObject("eight", 8.0)
        secChooser.addObject("nine", 9.0)
        secChooser.addObject("ten", 10.0)
        SmartDashboard.putData("wait second", secChooser)
    }
}
