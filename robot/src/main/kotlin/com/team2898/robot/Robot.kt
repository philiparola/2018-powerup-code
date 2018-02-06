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
//import com.team2898.robot.config.RobotMap.DT_SOLENOID_FORWARD_ID
//import com.team2898.robot.config.RobotMap.DT_SOLENOID_REVERSE_ID
import com.team2898.robot.motion.RobotPose
import com.team2898.robot.motion.pathfinder.*
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
import jaci.pathfinder.Trajectory
import kotlinx.serialization.json.JSON

class Robot : IterativeRobot() {
    companion object {
        val debug = true
        val ntInstance = NetworkTableInstance.create()
    }

    val data = DriverStation.getInstance().gameSpecificMessage
//    val bunnybotPneumatics = DoubleSolenoid(DT_SOLENOID_FORWARD_ID, DT_SOLENOID_REVERSE_ID)
    val autoCommand = MotionProfileTest()
//    val autoCommand = PrintProfile()

    val teleopCommand = Teleop()
    override fun robotInit() {
        Drivetrain.zeroEncoders()
//        bunnybotPneumatics.set(DoubleSolenoid.Value.kReverse)
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

//        val testProfileSettings = ProfileSettings(
//                hz = 100, maxVel = 50.0, maxAcc = 100.0, maxJerk = 100.0, wheelbaseWidth = 100.0, wayPoints = convWaypoint(baselineProfile), fitMethod = Trajectory.FitMethod.HERMITE_CUBIC, sampleRate = Trajectory.Config.SAMPLES_HIGH)
//
//        val string = JSON().stringify(testProfileSettings)
//        ProfileGenerator.deferProfile(testProfileSettings)

        AsyncLooper(100.0) {
//            SmartDashboard.putNumber("dt left vel", Drivetrain.encVelInSec[0])
//            SmartDashboard.putNumber("dt right vel", Drivetrain.encVelInSec[1])
//            SmartDashboard.putNumber("dt left pos", Drivetrain.encPosIn[0])
//            SmartDashboard.putNumber("dt right pos", Drivetrain.encPosIn[1])
//            SmartDashboard.putNumber("robot x", RobotPose.pose.x)
//            SmartDashboard.putNumber("robot y", RobotPose.pose.y)
//            SmartDashboard.putNumber("robot theta", RobotPose.pose.theta)
            SmartDashboard.putNumber("NavX yaw", Navx.yaw)

//            println("Left master current: ${Drivetrain.leftMaster.outputCurrent}")
//            println("Right master current: ${Drivetrain.rightMaster.outputCurrent}")
//            println("Left slave current: ${Drivetrain.leftSlave.outputCurrent}")
//            println("Right slave current: ${Drivetrain.leftSlave.outputCurrent}")
        }.start()

        SmartDashboard.putString("Session UUID", Logger.uuid)

//        AsyncLooper(1.0) {
//            println("Left master output voltage: ${Drivetrain.leftMaster.motorOutputVoltage}")
//            println("Right master output voltage: ${Drivetrain.rightMaster.motorOutputVoltage}")
//        }.start()
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
    }

    override fun autonomousInit() {
//        bunnybotPneumatics.set(DoubleSolenoid.Value.kReverse)
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        LoopManager.onAutonomous()
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Autonomous Init")
        Navx.reset()
        autoCommand.start()

        if (teleopCommand.isRunning) teleopCommand.cancel()

    }


    override fun autonomousPeriodic() {
        Scheduler.getInstance().run()
    }

    override fun teleopInit() {
//        bunnybotPneumatics.set(DoubleSolenoid.Value.kReverse)
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
