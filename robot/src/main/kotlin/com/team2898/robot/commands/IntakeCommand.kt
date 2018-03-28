package com.team2898.robot.commands

import com.team2898.robot.subsystems.Intake
import edu.wpi.first.wpilibj.command.TimedCommand

class IntakeCommand(time: Double, val power:Pair<Double, Double> = Pair(1.0, 1.0)) : TimedCommand(time) {
    override fun initialize() {
        Intake.power = power
    }

    override fun execute() {
        Intake.power=power
    }

    override fun end() {
        Intake.power=Pair(0.0,0.0)
    }
}
