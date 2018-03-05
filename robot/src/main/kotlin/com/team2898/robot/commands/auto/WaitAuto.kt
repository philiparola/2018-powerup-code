package com.team2898.robot.commands.auto

import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.CommandGroup
import edu.wpi.first.wpilibj.command.WaitCommand

class WaitAuto(val auto: Command, val wait: Double): CommandGroup() {
    init {
        addSequential(WaitCommand(wait))
        addSequential(auto)
    }
}