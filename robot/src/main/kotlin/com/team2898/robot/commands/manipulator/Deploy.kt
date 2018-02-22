package com.team2898.robot.commands.manipulator

import com.team2898.robot.config.ManipConf.DEPLOY_POS
import com.team2898.robot.subsystems.Manipulator
import edu.wpi.first.wpilibj.command.InstantCommand

class Deploy: InstantCommand() {
    override fun execute() {
        Manipulator.targetPos = DEPLOY_POS
    }
}