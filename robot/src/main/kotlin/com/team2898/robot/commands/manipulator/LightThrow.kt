package com.team2898.robot.commands.manipulator

import com.team2898.robot.config.ManipConf.LIGHT_THROW_FINAL_POS
import com.team2898.robot.subsystems.Manipulator
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.InstantCommand
import edu.wpi.first.wpilibj.command.WaitCommand

class LightThrow(val wait: Boolean = true): Command() {
    override fun initialize() {
        Manipulator.targetPos = LIGHT_THROW_FINAL_POS
    }

    override fun isFinished(): Boolean {
        if (!wait) return true
        return Manipulator.currentPos == LIGHT_THROW_FINAL_POS
    }
}