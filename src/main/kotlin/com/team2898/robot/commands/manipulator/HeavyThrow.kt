package com.team2898.robot.commands.manipulator

import com.team2898.robot.config.ManipConf.HEAVY_THROW_FINAL_POS
import com.team2898.robot.subsystems.Manipulator
import edu.wpi.first.wpilibj.command.Command

class HeavyThrow(val wait: Boolean = true): Command() {
    override fun initialize() {
        Manipulator.targetPos = HEAVY_THROW_FINAL_POS
    }

    override fun isFinished(): Boolean {
        if (!wait) return true
        return Manipulator.currentPos() == HEAVY_THROW_FINAL_POS
    }

}
