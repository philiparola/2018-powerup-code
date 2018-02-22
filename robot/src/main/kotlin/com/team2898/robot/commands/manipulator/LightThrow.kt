package com.team2898.robot.commands.manipulator

import com.team2898.robot.config.ManipConf.LIGHT_THROW_FINAL_POS
import com.team2898.robot.config.ManipConf.LIGHT_THROW_INIT_POS
import com.team2898.robot.subsystems.Manipulator
import edu.wpi.first.wpilibj.command.InstantCommand
import edu.wpi.first.wpilibj.command.WaitCommand

class LightThrow: InstantCommand() {

    override fun initialize() {
        Manipulator.targetPos = LIGHT_THROW_INIT_POS
    }

    override fun execute() {
        object : WaitCommand(0.5) {}.start()
        Manipulator.targetPos = LIGHT_THROW_FINAL_POS
    }
}