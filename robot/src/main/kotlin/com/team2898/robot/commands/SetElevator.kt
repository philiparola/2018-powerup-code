package com.team2898.robot.commands

import com.team2898.engine.math.clamp
import com.team2898.robot.config.ElevatorConf.MAX_HEIGHT_FT
import com.team2898.robot.config.ElevatorConf.MIN_HEIGHT_FT
import com.team2898.robot.subsystems.Elevator
import edu.wpi.first.wpilibj.command.Command

open class SetElevator(var height: Double, val wait: Boolean = true) : Command() {

    override fun initialize() = execute()

    override fun execute() {
        Elevator.targetPosFt = clamp(height, min = MIN_HEIGHT_FT, max = MAX_HEIGHT_FT)
    }

    override fun isFinished(): Boolean {
        if (wait)
            return (Elevator.targetPosFt - .1 < Elevator.currentPosFt &&
                    Elevator.targetPosFt + .1 > Elevator.currentPosFt)
        return true
    }
}