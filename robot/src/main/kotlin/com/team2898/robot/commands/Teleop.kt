package com.team2898.robot.commands

import com.team2898.engine.extensions.Vector2D.get
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.motion.CheesyDrive
import com.team2898.engine.motion.DriveSignal
import edu.wpi.first.wpilibj.command.Command
import com.team2898.robot.OI
import com.team2898.robot.subsystems.Drivetrain
import com.team2898.robot.subsystems.Navx
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.WaitCommand
import java.io.File
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard as sd

class Teleop : Command() {
    val HOME = "/home/lvuser"
    val files = listOf<File>(
            File("$HOME/left2V.csv"),
            File("$HOME/left6V.csv"),
            File("$HOME/left9V.csv"),
            File("$HOME/left12V.csv"),
            File("$HOME/right2V.csv"),
            File("$HOME/right6V.csv"),
            File("$HOME/right9V.csv"),
            File("$HOME/right12V.csv")
    )
    var startTime = 0.0
    val currentTime
        get() = startTime - Timer.getFPGATimestamp()

    override fun initialize() {
        files.forEach {
            it.writeText("time, vel\n")
        }
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        Drivetrain.zeroEncoders()
        Navx.reset()
        startTime = Timer.getFPGATimestamp()
    }

    override fun execute() {
        if (OI.aButton) {
            Drivetrain.openLoopPower = DriveSignal(2 / 12.0, 2 / 12.0)
        } else if (OI.bButton) {
            Drivetrain.openLoopPower = DriveSignal(6/12.0, 6/12.0)
        } else if (OI.xButton) {
            Drivetrain.openLoopPower = DriveSignal(9/12.0, 9/12.0)
        } else if (OI.yButton) {
            Drivetrain.openLoopPower = DriveSignal(12.0/12.0, 12/12.0)
        }
        else {
            CheesyDrive.updateQuickTurn(OI.quickTurn)
            Drivetrain.openLoopPower = CheesyDrive.updateCheesy(
                    (if (!OI.quickTurn) OI.turn else -OI.leftTrigger + OI.rightTrigger),
                    -OI.throttle,
                    OI.quickTurn,
                    true
            )
        }
    }

    override fun isFinished(): Boolean = false
}