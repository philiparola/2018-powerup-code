package com.team2898.robot.commands.auto

import com.team2898.engine.motion.DriveSignal
import com.team2898.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj.command.TimedCommand

class Baseline: TimedCommand(5.0) {
    override fun execute() {
        Drivetrain.openLoopPower = DriveSignal(0.5, 0.5)
    }
}