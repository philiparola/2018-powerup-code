package com.team2898.robot.commands.manipulator

import com.fasterxml.jackson.databind.jsontype.impl.MinimalClassNameIdResolver
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.robot.config.ManipConf.*
import com.team2898.robot.subsystems.Manipulator
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.InstantCommand

class ManipIntake(val wait: Boolean = true): Command() {
    override fun isFinished(): Boolean {
        if (!wait) return true
        return Manipulator.currentPos() == INTAKE_POS
    }

    override fun execute() {
        Manipulator.targetPos = INTAKE_POS
    }
}