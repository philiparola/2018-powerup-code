package com.team2898.robot.commands.manipulator

import com.team2898.robot.config.ManipConf.HOLD_POS
import com.team2898.robot.subsystems.Manipulator
import edu.wpi.first.wpilibj.command.Command

class Hold(val wait: Boolean = false) : Command() {
    override fun initialize() {
        Manipulator.targetPos = HOLD_POS
    }

    override fun isFinished(): Boolean {
        if (!wait) return true
        return Manipulator.currentPos == HOLD_POS
    }
}