package com.team2898.robot.commands.auto

import com.team2898.engine.motion.DriveSignal
import com.team2898.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj.command.TimedCommand
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.Command


class Baseline: Command(3.0) {
    var start = 0.0

    override fun execute() {
        Drivetrain.openLoopPower = DriveSignal(-0.3, -0.3)
    }

    override fun isFinished(): Boolean {
        return isTimedOut//(Timer.getFPGATimestamp() - start >= 3)
    }

    override fun end() {
        Drivetrain.openLoopPower = DriveSignal(left = 0.0, right = 0.0, brake = true)
    }
}