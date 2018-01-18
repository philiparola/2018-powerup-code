package com.team2898.robot

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.extensions.Vector2D.get
import com.team2898.engine.logging.LogLevel
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.reflectLocation
import com.team2898.engine.logic.LoopManager
import com.team2898.robot.commands.MotionProfileTest
import com.team2898.robot.commands.PrintProfile
import com.team2898.robot.commands.Teleop
import com.team2898.robot.config.RobotMap.DT_SOLENOID_FORWARD_ID
import com.team2898.robot.config.RobotMap.DT_SOLENOID_REVERSE_ID
import com.team2898.robot.motion.RobotPose
import edu.wpi.first.wpilibj.networktables.NetworkTable as nt
import edu.wpi.first.wpilibj.command.Scheduler
import com.team2898.robot.subsystems.*
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.command.PrintCommand
import edu.wpi.first.wpilibj.hal.FRCNetComm
import edu.wpi.first.wpilibj.hal.HAL
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

class Robot : IterativeRobot() {
    companion object {
        val debug = true
        val ntInstance = NetworkTableInstance.create()
    }

    val data = DriverStation.getInstance().gameSpecificMessage
    val bunnybotPneumatics = DoubleSolenoid(DT_SOLENOID_FORWARD_ID, DT_SOLENOID_REVERSE_ID)
    val autoCommand = MotionProfileTest()
//    val autoCommand = PrintProfile()

    val teleopCommand = Teleop()
    override fun robotInit() {
        bunnybotPneumatics.set(DoubleSolenoid.Value.kReverse)
        //System.setProperty("java.util.concurrent.ForkJoinPool.common.parallelism", "2")

        /*
        ntInstance.setUpdateRate(1 / 100.0) // 100hz

        (0 until 16).forEach {
            HAL.report(FRCNetComm.tResourceType.kResourceType_CANJaguar, it)
            HAL.report(FRCNetComm.tResourceType.kResourceType_NidecBrushless, it)
        }
        (0 until 4).forEach {
            HAL.report(FRCNetComm.tResourceType.kResourceType_PDP, it)
            HAL.report(FRCNetComm.tResourceType.kResourceType_Kinect, it)
        }
        */


        AsyncLooper(100.0) {
            SmartDashboard.putNumber("dt left vel", Drivetrain.encVelInSec[0])
            SmartDashboard.putNumber("dt right vel", Drivetrain.encVelInSec[1])
            SmartDashboard.putNumber("dt left pos", Drivetrain.encPosIn[0])
            SmartDashboard.putNumber("dt right pos", Drivetrain.encPosIn[1])
            SmartDashboard.putNumber("robot x", RobotPose.pose.x)
            SmartDashboard.putNumber("robot y", RobotPose.pose.y)
            SmartDashboard.putNumber("robot theta", RobotPose.pose.theta)
            SmartDashboard.putNumber("NavX yaw", Navx.yaw)
            SmartDashboard.putString("Session UUID", Logger.uuid)
        }.start()


//        AsyncLooper(1.0) {
//            println("dt left vel ${Drivetrain.encVelInSec[0]}")
//            println("dt right vel ${Drivetrain.encVelInSec[1]}")
//            println("dt left pos ${Drivetrain.encPosIn[0]}")
//            println("dt right pos ${Drivetrain.encPosIn[1]}")
//        }.start()
    }

    override fun autonomousInit() {
        bunnybotPneumatics.set(DoubleSolenoid.Value.kReverse)
        LoopManager.onAutonomous()
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Autonomous Init")
        Navx.reset()
        autoCommand.start()
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP

        if (teleopCommand.isRunning) teleopCommand.cancel()

    }


    override fun autonomousPeriodic() {
        Scheduler.getInstance().run()
    }

    override fun teleopInit() {
        bunnybotPneumatics.set(DoubleSolenoid.Value.kReverse)
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Teleop Init")
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
    //override fun robotInit() = println("Robot init!")
    //override fun teleopInit() = println("Teleop init!")
}
