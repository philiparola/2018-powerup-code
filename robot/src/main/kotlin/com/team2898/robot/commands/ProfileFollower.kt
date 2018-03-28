package com.team2898.robot.commands

import com.team2898.engine.controlLoops.classicControl.PVAPID
import com.team2898.engine.extensions.minus
import com.team2898.engine.motion.DriveSignal
import com.team2898.engine.extensions.get
import com.team2898.robot.config.DrivetrainConf.*
import com.team2898.robot.subsystems.Drivetrain
import com.team2898.robot.subsystems.Navx
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.Command
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import kotlin.math.abs
import kotlin.math.sign

class ProfileFollower(profile: Pair<Trajectory, Trajectory>) : Command() {
    val leftTraj = profile.first
    val rightTraj = profile.second

    var encoderOffset = Pair(0.0, 0.0)
    var currentSegment = 0
    var startTime = 0.0
    var initElapseTime = 0.0
    val elapsedTime // current time
        get() = Timer.getFPGATimestamp() - startTime
    val encoderDistance
        get() = Drivetrain.encPosFt - encoderOffset
    var t = 0

    val leftPVA = PVAPID(
            Kpp = 0.005,
            Kpi = 0.001,
            Kvp = 0.0,
            Kvf = { speed: Double ->
                // speed = f/s
                ((if (abs(speed) > 0.1) abs(speed) - LEFT_B else 0.0) / LEFT_M) / 12.0 * sign(speed) // motor setpoint
            },
            Kaf = 0.05,
            Kpf = { 0.0 }
    )

    val rightPVA = PVAPID(
            Kpp = 0.005,
            Kpi = 0.001,
            Kvp = 0.0,
            Kvf = { speed: Double ->
                // speed = f/s
                ((if (abs(speed) > 0.1) abs(speed) - RIGHT_B else 0.0) / RIGHT_M) / 12.0 * sign(speed) // motor setpoint
            },
            Kaf = 0.05,
            Kpf = { 0.0 }
    )

    override fun initialize() {
        println("init")
        encoderOffset = Drivetrain.encPosFt
        assert(leftTraj.length() == rightTraj.length())
        currentSegment = 0
        startTime = Timer.getFPGATimestamp()
        println("start time: $startTime")
        println("the segment size ${leftTraj.segments.size}")
        initElapseTime = elapsedTime
        t = 0
    }

    override fun execute() {
        if (isFinished) return
        val currentDistance = Drivetrain.encPosFt - encoderOffset
        currentSegment = (elapsedTime / leftTraj.segments[0].dt).toInt()
        if (currentSegment == leftTraj.segments.size) return
        val leftSeg = leftTraj.segments.get(currentSegment)
        val rightSeg = rightTraj.segments.get(currentSegment)

        val desiredHeading = Pathfinder.r2d(leftSeg.heading)
        val angleDifference = Pathfinder.boundHalfDegrees(desiredHeading + Navx.yaw)
        val kp = 0.8
        val turn = kp * (-1.0 / 80.0) * angleDifference

        val left = leftPVA.update(
                currentDistance[0],
                Drivetrain.encVelInSec[0] / 12.0,
                leftSeg.position,
                leftSeg.velocity,
                leftSeg.acceleration
        )

        val right = rightPVA.update(
                currentDistance[1],
                Drivetrain.encVelInSec[1] / 12.0,
                rightSeg.position,
                rightSeg.velocity,
                rightSeg.acceleration
        )
        Drivetrain.openLoopPower = DriveSignal(left + turn, right - turn)
        t++
    }

    override fun end() {
        println("Done")
    }

    override fun isFinished(): Boolean {
        return leftTraj.segments[0].dt * leftTraj.segments.size <= elapsedTime // i guess?
    }
}