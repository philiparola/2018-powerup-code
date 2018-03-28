package com.team2898.robot.commands

import com.team2898.engine.OI.ToggleDebounce
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.math.clamp
import com.team2898.engine.motion.CheesyDrive
import edu.wpi.first.wpilibj.command.Command
import com.team2898.robot.OI
import com.team2898.robot.config.ControllerConf.NOENC
import com.team2898.robot.subsystems.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard as sd
import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.team2898.robot.config.ArmConf.SETPOINT_1_DEG
import com.team2898.robot.config.ArmConf.SETPOINT_2_DEG
import com.team2898.robot.config.ArmConf.SETPOINT_3_DEG
import com.team2898.robot.config.ArmConf.SETPOINT_4_DEG
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D

class Teleop : Command() {
    //    val HOME = "/home/lvuser"
//    val files = listOf<File>(
//            File("$HOME/left2V.csv"),
//            File("$HOME/left6V.csv"),
//            File("$HOME/left9V.csv"),
//            File("$HOME/left12V.csv"),
//            File("$HOME/right2V.csv"),
//            File("$HOME/right6V.csv"),
//            File("$HOME/right9V.csv"),
//            File("$HOME/right12V.csv")
//    )

    var startTime = 0.0

    override fun initialize() {
        if (!NOENC) {
        }

        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        Drivetrain.zeroEncoders()
        Navx.reset()
        startTime = Timer.getFPGATimestamp()
        Arm.targetRotation = Rotation2d.createFromDegrees(90.0)
    }

    override fun execute() {
        CheesyDrive.updateQuickTurn(OI.quickTurn)
        Drivetrain.uncorrectedOpenLoopPower = CheesyDrive.updateCheesy(
                (if (!OI.quickTurn) OI.turn else -OI.leftTrigger + OI.rightTrigger),
                OI.throttle,
                OI.quickTurn,
                false
        )
        //if (OI.opA) Manipulator.talon.set(ControlMode.PercentOutput, .3)
        //else if(OI.opB) Manipulator.talon.set(ControlMode.PercentOutput, -.3)
        //else Manipulator.talon.set(ControlMode.PercentOutput, 0.0)

        val intakePower = OI.opLY
        val intakeMod = (OI.rightTrigger - OI.leftTrigger) * 0.5
        Intake.power = Pair(
                clamp(intakePower - intakeMod, 1.0),
                clamp(intakePower + intakeMod, 1.0)
        )

        if (OI.opA) Arm.targetRotation = Rotation2d.createFromDegrees(SETPOINT_3_DEG)
        else if (OI.opB) Arm.targetRotation = Rotation2d.createFromDegrees(SETPOINT_2_DEG)
        else if (OI.opX) Arm.targetRotation = Rotation2d.createFromDegrees(SETPOINT_4_DEG)
        else if (OI.opY) Arm.targetRotation = Rotation2d.createFromDegrees(SETPOINT_1_DEG)

    }

    override fun isFinished(): Boolean = false
}