package com.team2898.robot

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.extensions.get
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.logging.LogLevel
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.reflectLocation
import com.team2898.engine.logic.LoopManager
import com.team2898.robot.commands.Teleop
import com.team2898.engine.logic.SelfCheckManager
import com.team2898.robot.commands.BaselineAuto
import com.team2898.robot.commands.CharDt
import com.team2898.robot.commands.SwitchFromCenter
import com.team2898.robot.motion.RobotPose
import edu.wpi.first.wpilibj.command.Scheduler
import com.team2898.robot.subsystems.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard as sd

class Robot : TimedRobot() {
    companion object {
        val debug = true
    }

    var teleopCommand: Command = Teleop()
    var autoCommand: Command = SwitchFromCenter(true)

    var matchData = ""

    var found = false
    val secChooser = SendableChooser<Double>()

    override fun robotInit() {
        SelfCheckManager.checkAll()
        Drivetrain.zeroEncoders()
        sd.putString("Session UUID", Logger.uuid)

        sd.putNumber("target deg", Arm.targetRotation.degrees)
        sd.putNumber("char volt", 2.0)

        //LoopManager.register(CubeLidar)
        //LoopManager.register(RobotPose)
        //CameraServer.getInstance().startAutomaticCapture(0)

        AsyncLooper(25.0) {
            sd.putNumber("pwm pos", Arm.masterTalon.pwmPos.toDouble())
            //sd.putNumber("pwm pos slave", Arm.slave1.pwmPos.toDouble())
            sd.putNumber("pos", Arm.masterTalon.getSelectedSensorPosition(0).toDouble())
            sd.putString("rot", Arm.rotation.toString())
            sd.putNumber("vout", Arm.masterTalon.motorOutputVoltage)
            //sd.putNumber("iout", Arm.masterTalon.outputCurrent)
            //sd.putNumber("iout slave", Arm.slave1.outputCurrent)
            sd.putString("target", Arm.targetRotation.toString())

            sd.putNumber("actual deg", Arm.armDegrees)

            sd.putNumber("closed loop error", Arm.masterTalon.getClosedLoopError(0).toDouble())
            sd.putNumber("closed loop target", Arm.masterTalon.getClosedLoopTarget(0).toDouble())

            //sd.putNumber("measure 1", CubeLidar.distances.first.toDouble())
            //sd.putNumber("measure 2", CubeLidar.distances.second.toDouble())
            //sd.putNumber("measure 3", CubeLidar.distances.third.toDouble())
            //sd.putNumber("measure 4", CubeLidar.distances.fourth.toDouble())

            sd.putNumber("left enc vel", Drivetrain.encVelInSec[0])
            sd.putNumber("right enc vel", Drivetrain.encVelInSec[1])
            sd.putNumber("gyro", Navx.yaw)
            sd.putString("pose", RobotPose.pose.toString())

            //sd.putNumber("ly", OI.opLY)
            //println("pwm pos ${Arm.masterTalon.pwmPos}")

            //println("pos ${"%.3f".format(Arm.armDegrees)} controller pos ${"%.3f".format(sd.getNumber("arm controller deg", 90.0))}")
            //println("uncor: ${Drivetrain.uncorrectedOpenLoopPower} corr: ${Drivetrain.openLoopPower}")

            //println("cos ${Arm.rotation.cos}")
            //println("sin ${Arm.rotation.sin}")
        }.start()
    }


    override fun autonomousInit() {
        var tries = 100
        while(!found && tries >= 0) {
            tries --
            if (DriverStation.getInstance().gameSpecificMessage.length == 3 && DriverStation.getInstance().gameSpecificMessage != null) found = true
        }
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        LoopManager.onAutonomous()
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Autonomous Init")
        Navx.reset()

        Arm.targetRotation = Rotation2d.createFromDegrees(90.0)
        if (found) {
            autoCommand = SwitchFromCenter(matchData[0] == 'R')
        } else {
            autoCommand = BaselineAuto()
        }
        autoCommand.start()
    }

    override fun teleopInit() {
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Teleop Init")
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        LoopManager.onTeleop()

        autoCommand.cancel()
        teleopCommand = Teleop()
        teleopCommand.start()

        //SetArm(Rotation2d.createFromDegrees(90.0)).start()
        Arm.targetRotation = Rotation2d.createFromDegrees(90.0)
    }

    override fun disabledInit() {
        LoopManager.onDisable()
    }

    override fun autonomousPeriodic() {
        Scheduler.getInstance().run()
    }

    override fun teleopPeriodic() {
        Scheduler.getInstance().run()

        //fucksicle.set(ControlMode.PercentOutput, OI.opLY)
        //str += "${OI.opLY},${fucksicle.getSelectedSensorVelocity(0)}\r\n"
        //println("${OI.opLY},${fucksicle.getSelectedSensorVelocity(0)}")
        //Arm.targetRotation = Rotation2d.createFromDegrees(sd.getNumber("target deg", 90.0))
        //Intake.power = Pair(OI.opLY, OI.opLY)

    }

    override fun disabledPeriodic() {
        if (!found) {
            if (DriverStation.getInstance().gameSpecificMessage != null || DriverStation.getInstance().gameSpecificMessage.length == 3) {
                found = true
                Logger.logInfo("GSM Reporter", LogLevel.INFO, "Match Data found")
                matchData = DriverStation.getInstance().gameSpecificMessage
            }
        }
    }
}
