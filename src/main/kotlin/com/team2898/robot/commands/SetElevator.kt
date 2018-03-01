package com.team2898.robot.commands

import com.team2898.robot.config.ElevatorConf.MAX_HEIGHT_FT
import com.team2898.robot.config.ElevatorConf.MIN_HEIGHT_FT
import com.team2898.robot.subsystems.Elevator
import edu.wpi.first.wpilibj.command.Command

open class SetElevator(var height: Double, val wait: Boolean = true) : Command() {

    override fun initialize() = execute()

    override fun execute() {
        if (height > MAX_HEIGHT_FT) height = MAX_HEIGHT_FT
        if (height < MIN_HEIGHT_FT) height = MIN_HEIGHT_FT
        Elevator.targetPosFt = height
    }

    override fun isFinished(): Boolean {
        if (wait) return (Elevator.targetPosFt == Elevator.currentPosFt)
        return true
    }
}