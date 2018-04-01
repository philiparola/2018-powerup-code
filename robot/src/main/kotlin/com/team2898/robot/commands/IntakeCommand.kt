package com.team2898.robot.commands

import com.team2898.robot.subsystems.Intake
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.command.TimedCommand

class IntakeCommand(time: Double, val power: Pair<Double, Double> = Pair(1.0, 1.0),
                    val piston: DoubleSolenoid.Value = DoubleSolenoid.Value.kForward) : TimedCommand(time) {
    override fun initialize() {
        Intake.power = power
    }

    override fun execute() {
        Intake.power = power
    }

    override fun end() {
        Intake.power = Pair(0.0, 0.0)
    }
}
