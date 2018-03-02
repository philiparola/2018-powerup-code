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

    var found = false
    val startChooser = SendableChooser<AutoManager.StartLocations>()
    val targetChooser = SendableChooser<AutoManager.TargetAutos>()
    val secChooser = SendableChooser<Double>()

    override fun robotInit() {
        SelfCheckManager.checkAll()
        Drivetrain.zeroEncoders()

        AsyncLooper(100.0) {
            SmartDashboard.putNumber("NavX yaw", Navx.yaw)
            SmartDashboard.putNumber("intake angle 1", Intake.currentPos.first.degrees)
            SmartDashboard.putNumber("intake angle 2", Intake.currentPos.second.degrees)
            SmartDashboard.putNumber("intake left sin", Intake.currentPos.first.sin)
            SmartDashboard.putNumber("intake left cos", Intake.currentPos.first.cos)
            SmartDashboard.putNumber("intake right sin", Intake.currentPos.second.sin)
            SmartDashboard.putNumber("intake right cos", Intake.currentPos.second.cos)
            SmartDashboard.putNumber("intake left pwm pos", (Intake.leftDeployTalon.pwmPos).toDouble())
            SmartDashboard.putNumber("intake right pwm pos", (Intake.rightDeployTalon.pwmPos).toDouble())
            SmartDashboard.putNumber("elev height", Elevator.currentPosFt)
            SmartDashboard.putNumber("manip angle", Manipulator.currentPos.degrees)
            SmartDashboard.putNumber("manip pwm pos", (Manipulator.talon.sensorCollection.pulseWidthPosition and 0xFFF).toDouble())
            SmartDashboard.putNumber("manip cos", Manipulator.currentPos.cos)
            SmartDashboard.putNumber("manip sin", Manipulator.currentPos.sin)
        }.start()

        SmartDashboard.putString("Session UUID", Logger.uuid)

        Intake.rehome()
        Manipulator.rehome()

        AsyncLooper(SCHEDULER_HZ) {
            Scheduler.getInstance().run()
        }
        pushChoosers()
    }

    override fun autonomousInit() {
        var tries = 100
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        LoopManager.onAutonomous()
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Autonomous Init")
        Navx.reset()

        val start = startChooser.selected
        val target = targetChooser.selected
        val waitSecond = secChooser.selected

        if (teleopCommand.isRunning) teleopCommand.cancel()

        while (!found && tries >= 0) {
            Logger.logInfo("Auto init", LogLevel.WARNING, "Match data still not found")
            if (DriverStation.getInstance().gameSpecificMessage != null && DriverStation.getInstance().gameSpecificMessage.length == 3) found = true
            tries--
        }

        if (found) {
            WaitAuto(AutoManager.produceAuto(start, target).commnad, waitSecond).start()
        } else {
            DriverStation.reportWarning("Executing baseline command()", false)
            WaitAuto(Baseline(), waitSecond).start()
        }
    }

    override fun teleopInit() {
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Teleop Init")
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        LoopManager.onTeleop()

    }

    override fun disabledInit() {
        LoopManager.onDisable()
    }

    override fun disabledPeriodic() {
        if (!found) {
            if (DriverStation.getInstance().gameSpecificMessage != null || DriverStation.getInstance().gameSpecificMessage.length == 3) {
                found = true
                Logger.logInfo("Disabled Periodic", LogLevel.INFO, "Match Data found")
            }
        }
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
