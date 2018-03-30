package com.team2898.robot.commands

import com.team2898.engine.extensions.get
import com.team2898.engine.motion.DriveSignal
import com.team2898.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj.command.TimedCommand

class DtCompOpen(time: Double,val speed: Pair<Double,Double>) : TimedCommand(time) {
    override fun initialize() {
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
    }

    override fun execute() {
        Drivetrain.uncorrectedOpenLoopPower = DriveSignal(speed[0],speed[1])
    }
    override fun end() {
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        Drivetrain.openLoopPower = DriveSignal.BRAKE
    }
}
