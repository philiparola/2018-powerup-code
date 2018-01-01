package com.team2898.robot.motion

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.kinematics.RigidTransform2d
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.ILooper
import com.team2898.engine.logic.Subsystem
import com.team2898.robot.subsystems.Drivetrain
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D

object RobotState : ILooper {
    override val enableTimes = listOf(GamePeriods.TELEOP, GamePeriods.AUTO)

    var pose = RigidTransform2d()

    var lastEncoderPosition: Vector2D = Vector2D(0.0, 0.0)

    override fun onStart() {
        lastEncoderPosition = Drivetrain.encPos
    }

    override val loop = AsyncLooper(100.0) {
        pose = integrateForwardKinematics(pose,
                forwardKinematics(Drivetrain.encPos)
        )
    }

}