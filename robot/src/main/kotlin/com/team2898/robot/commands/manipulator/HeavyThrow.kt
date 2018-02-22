package com.team2898.robot.commands.manipulator

import com.team2898.robot.config.ManipConf.HEAVY_THROW_FINAL_POS
import com.team2898.robot.config.ManipConf.HEAVY_THROW_INIT_POS
import com.team2898.robot.config.ManipConf.MANIP_PEAK_MAX_AMPS
import com.team2898.robot.subsystems.Manipulator
import edu.wpi.first.wpilibj.command.InstantCommand
import edu.wpi.first.wpilibj.command.WaitCommand

class HeavyThrow : InstantCommand() {
    override fun initialize() {
        Manipulator.targetPos = HEAVY_THROW_INIT_POS
    }

    override fun execute() {
        object : WaitCommand(0.5) {}.start()
        Manipulator.targetPos = HEAVY_THROW_FINAL_POS
    }
}