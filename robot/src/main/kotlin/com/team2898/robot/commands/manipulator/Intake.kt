package com.team2898.robot.commands.manipulator

import com.team2898.engine.kinematics.Rotation2d
import com.team2898.robot.config.ManipConf.*
import com.team2898.robot.subsystems.Manipulator
import edu.wpi.first.wpilibj.command.InstantCommand

class ManipIntake: InstantCommand() {
    override fun execute() {
        Manipulator.targetPos = INTAKE_POS
    }
}