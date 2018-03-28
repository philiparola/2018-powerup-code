package com.team2898.robot.commands

import com.team2898.engine.kinematics.Rotation2d
import com.team2898.robot.subsystems.Arm
import edu.wpi.first.wpilibj.command.Command

class SetArm(val target: Rotation2d): Command(){
    override fun initialize() {
        Arm.targetRotation = target
    }
    override fun execute() {
        Arm.targetRotation = target
    }
    override fun isFinished(): Boolean = Arm.onTarget()
}