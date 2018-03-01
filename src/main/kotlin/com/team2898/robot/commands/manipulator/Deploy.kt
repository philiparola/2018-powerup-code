package com.team2898.robot.commands.manipulator

import com.sun.xml.internal.ws.transport.http.DeploymentDescriptorParser
import com.team2898.robot.config.ManipConf.DEPLOY_POS
import com.team2898.robot.subsystems.Manipulator
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.InstantCommand

class Deploy(val wait: Boolean = true): Command() {
    override fun isFinished(): Boolean {
        if (!wait) return true
        return Manipulator.currentPos() == DEPLOY_POS
    }

    override fun execute() {
        Manipulator.targetPos = DEPLOY_POS
    }
}