package com.team2898.robot.commands

import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.motion.CheesyDrive
import edu.wpi.first.wpilibj.command.Command
import com.team2898.robot.OI
import com.team2898.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard as sd

class Teleop : Command() {

    override fun initialize() {
        Drivetrain.controlMode = Drivetrain.ControlModes.VELOCITY_DRIVE
//        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
    }

    override fun execute() {

        CheesyDrive.updateQuickTurn(OI.quickTurn)

        Drivetrain.closedLoopVelTarget =
                CheesyDrive.updateCheesy(
                        (if (!OI.quickTurn) OI.turn else -OI.leftTrigger + OI.rightTrigger),
                        -OI.throttle,
                        OI.quickTurn,
                        true
                ).times(4096)

//        Drivetrain.openLoopPower = CheesyDrive.updateCheesy(
//                        (if (!OI.quickTurn) OI.turn else -OI.leftTrigger + OI.rightTrigger),
//                        -OI.throttle,
//                        OI.quickTurn,
//                        true
//        )
    }

    override fun isFinished(): Boolean = false
}