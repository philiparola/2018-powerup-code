package com.team2898.robot.motion

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.extensions.Vector2D.minus
import com.team2898.engine.kinematics.RigidTransform2d
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.ILooper
import com.team2898.engine.logic.RunEvery
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.types.CircularArray
import com.team2898.engine.types.Timestamp
import com.team2898.robot.config.RobotStateConf.BACKLOG_HZ
import com.team2898.robot.config.RobotStateConf.POSE_BACKLOG_SIZE
import com.team2898.robot.config.RobotStateConf.RUN_EVERY_NUMBER
import com.team2898.robot.subsystems.Drivetrain
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D
import kotlin.math.roundToInt
import kotlin.properties.Delegates

object RobotState : ILooper {
    override val enableTimes = listOf(GamePeriods.TELEOP, GamePeriods.AUTO)

    val backlog = CircularArray<Timestamp<RigidTransform2d>>(POSE_BACKLOG_SIZE)
    val runEvery = RunEvery(RUN_EVERY_NUMBER)

    var pose: RigidTransform2d by Delegates.observable(RigidTransform2d()) { prop, old, new ->
        OdometryNTReporter.updateNavMsgs()
    }

    var lastEncoderPosition: Vector2D = Vector2D(0.0, 0.0)

    override fun onStart() {
        lastEncoderPosition = Drivetrain.encPos
    }

    override val loop = AsyncLooper(100.0) {
        val encoderPos = Drivetrain.encPos
        val deltaPos = encoderPos - lastEncoderPosition
        pose = integrateForwardKinematics(pose,
                forwardKinematics(deltaPos)
        )
        lastEncoderPosition = encoderPos


        if (runEvery.shouldRun())
            backlog.add(Timestamp(stamp = pose))
    }

    operator fun get(index: Int) =
            backlog[index].stamp

}