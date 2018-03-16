package com.team2898.robot

import com.team2898.engine.logging.LogLevel
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.reflectLocation
import com.team2898.engine.logic.LoopManager
import com.team2898.robot.commands.Teleop
import com.team2898.engine.logic.SelfCheckManager
import com.team2898.robot.commands.Test
import edu.wpi.first.wpilibj.command.Scheduler
import com.team2898.robot.subsystems.*
import edu.wpi.first.wpilibj.CameraServer
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

class Robot : TimedRobot() {
    companion object {
        val debug = true
    }

    val teleopCommand = Teleop()

    val testCom = Test()

    var found = false
    val secChooser = SendableChooser<Double>()

    override fun robotInit() {
        println("run")
        SelfCheckManager.checkAll()
        Drivetrain.zeroEncoders()
//        CameraServer.getInstance().startAutomaticCapture(0)
        SmartDashboard.putString("Session UUID", Logger.uuid)
    }

    override fun autonomousInit() {
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        LoopManager.onAutonomous()
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Autonomous Init")
        Navx.reset()
        testCom.start()
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
    }

    fun pushChoosers() {
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
