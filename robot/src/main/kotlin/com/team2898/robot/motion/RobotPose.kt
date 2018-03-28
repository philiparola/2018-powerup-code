package com.team2898.robot.motion

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.extensions.minus
import com.team2898.engine.kinematics.RigidTransform2d
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.kinematics.Translation2d
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.ILooper
import com.team2898.engine.logic.RunEvery
import com.team2898.engine.types.CircularArray
import com.team2898.engine.types.Timestamp
import com.team2898.robot.config.RobotStateConf.POSE_BACKLOG_SIZE
import com.team2898.robot.config.RobotStateConf.RUN_EVERY_NUMBER
import com.team2898.robot.subsystems.Drivetrain
import kotlin.properties.Delegates

object RobotPose : ILooper {
    override val enableTimes = listOf(GamePeriods.TELEOP, GamePeriods.AUTO, GamePeriods.DISABLE)

    val backlog = CircularArray<Timestamp<RigidTransform2d>>(POSE_BACKLOG_SIZE)
    val runEvery = RunEvery(RUN_EVERY_NUMBER)
    var pose: RigidTransform2d by Delegates.observable(RigidTransform2d()) { prop, old, new ->
        OdometryNTReporter.updateNavMsgs()
    }

    init {
        backlog.add(Timestamp(stamp = RigidTransform2d(Translation2d(), Rotation2d())))
        backlog.add(Timestamp(stamp = RigidTransform2d(Translation2d(), Rotation2d())))
        backlog.add(Timestamp(stamp = RigidTransform2d(Translation2d(), Rotation2d())))
        println("Pose backlog size is $POSE_BACKLOG_SIZE")
    }

    var lastEncoderPosition: Pair<Double, Double> = Pair(0.0, 0.0)

    override fun onStart() {
        lastEncoderPosition = Drivetrain.encPosIn
    }

    override val loop = AsyncLooper(100.0) {
        //        println("Odometry loop ran! Latest entry is ${backlog[0]}")
        val encoderPos = Drivetrain.encPosIn
        val deltaPos = encoderPos - lastEncoderPosition
        pose = integrateForwardKinematics(pose, forwardKinematics(deltaPos) )
        lastEncoderPosition = encoderPos
        println(pose)


        if (runEvery.shouldRun())
            backlog.add(Timestamp(stamp = pose))
    }

    operator fun get(index: Int) =
            backlog[index].stamp
}