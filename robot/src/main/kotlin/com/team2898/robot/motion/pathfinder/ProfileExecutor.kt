package com.team2898.robot.motion.pathfinder

import com.team2898.engine.async.util.go
import com.team2898.engine.controlLoops.classicControl.PVAPID
import com.team2898.engine.extensions.Vector2D.get
import com.team2898.engine.logging.*
import com.team2898.engine.logic.RunEvery
import com.team2898.engine.motion.DriveSignal
import com.team2898.robot.config.MotionProfileConfig.*
import com.team2898.robot.subsystems.Drivetrain
import jaci.pathfinder.Trajectory
import jaci.pathfinder.Trajectory.Segment
import kotlinx.coroutines.experimental.Deferred
import kotlinx.coroutines.experimental.Job
import kotlinx.coroutines.experimental.cancelAndJoin
import kotlinx.coroutines.experimental.delay

class ProfileExecutor(val profile: Deferred<Pair<Trajectory, Trajectory>>) {

    var completed = false

    private lateinit var job: Job
    private var started = false

    private var logEvery = RunEvery(10)

    fun execute(motorCommand: (Double, Double) -> Unit) {
        println(reflectLocation())
        job = go {
            println(reflectLocation())
            started = true
            if (!profile.isCompleted) {
                Logger.logInfo(reflectLocation(), LogLevel.WARNING, "Profile not done generating! Waiting until completion to execute")
            }
            profile.await()

            println(reflectLocation())

            val completedProfile = profile.getCompleted()
            println(completedProfile)

            if (completedProfile.first.segments.size != completedProfile.second.segments.size)
                Logger.logInfo(reflectLocation(), LogLevel.WARNING, "Left and right trajectories different lengths!")

            println(reflectLocation())

            val leftController = PVAPID(
                    ProfileKp,
                    ProfileKi,
                    ProfileKvp,
                    ProfileKvf,
                    ProfileKaf,
                    -12.0,
                    12.0
            )
            val rightController = PVAPID(
                    ProfileKp,
                    ProfileKi,
                    ProfileKvp,
                    ProfileKvf,
                    ProfileKaf,
                    -12.0,
                    12.0
            )

            for (i in 0..completedProfile.first.segments.size) {
                val leftSegment = completedProfile.first.segments[0]
                val rightSegment = completedProfile.first.segments[0]

                val left = leftController.update(position = Drivetrain.encPosIn[0],
                        velocity = Drivetrain.encVelInSec[0],
                        targetPos = leftSegment.position,
                        targetVel = leftSegment.velocity,
                        targetAcc = leftSegment.acceleration,
                        dt = leftSegment.dt)
                val right = rightController.update(position = Drivetrain.encPosIn[1],
                        velocity = Drivetrain.encVelInSec[1],
                        targetPos = rightSegment.position,
                        targetVel = rightSegment.velocity,
                        targetAcc = rightSegment.acceleration,
                        dt = rightSegment.dt)


                motorCommand(left, right)

                if (logEvery.shouldRun()) {
                    Logger.logData("profile executor", "left reported pos", Drivetrain.encPosIn[0])
                    Logger.logData("profile executor", "left reported vel", Drivetrain.encVelInSec[0])
                    Logger.logData("profile executor", "right reported pos", Drivetrain.encPosIn[1])
                    Logger.logData("profile executor", "right reported vel", Drivetrain.encVelInSec[1])

                    Logger.logData("profile executor", "left target pos", leftSegment.position)
                    Logger.logData("profile executor", "left target vel", leftSegment.velocity)
                    Logger.logData("profile executor", "left target acc", leftSegment.acceleration)
                    Logger.logData("profile executor", "right target pos", rightSegment.position)
                    Logger.logData("profile executor", "right target vel", rightSegment.velocity)
                    Logger.logData("profile executor", "right target acc", rightSegment.acceleration)
                }
            }


            completed = true
            Drivetrain.closedLoopVelTarget = DriveSignal.NEUTRAL
        }
    }

    fun cancel() {
        if (!started || completed) return
        TimeBombAsync(0.1) {
            job.cancel()
            job.cancelChildren()
            Drivetrain.closedLoopVelTarget = DriveSignal.NEUTRAL
            job.join()
        }
    }


}