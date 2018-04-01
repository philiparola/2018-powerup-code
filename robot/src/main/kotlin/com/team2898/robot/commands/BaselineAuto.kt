package com.team2898.robot.commands

import com.team2898.engine.motion.DriveSignal
import com.team2898.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj.command.TimedCommand

class BaselineAuto : TimedCommand(3.0) {
    override fun initialize() {
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
    }

    override fun execute() {
        Drivetrain.uncorrectedOpenLoopPower = DriveSignal(0.3, 0.3)
    }

    override fun end() {
        Drivetrain.uncorrectedOpenLoopPower = DriveSignal(0.0, 0.0, true)
    }
}