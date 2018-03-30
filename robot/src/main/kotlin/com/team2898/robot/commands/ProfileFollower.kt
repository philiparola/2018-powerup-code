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

class ProfileFollower(profile: Pair<Trajectory, Trajectory>, val reverse: Boolean = false) : Command() {
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

    // ft, ft/s, ft/s^2 -> volts
    val leftPVA = PVAPID(
            Kpp = 0.006,
            Kpi = 0.002,
            Kvp = 0.01,
            Kvf = { speed: Double ->
                // speed = f/s
                ((if (abs(speed) > 0.1) abs(speed) - LEFT_B else 0.0) / LEFT_M) / 12.0 * sign(speed) // motor setpoint
            },
            Kaf = 0.04,
            Kpf = { 0.0 }
    )

    val rightPVA = PVAPID(
            Kpp = 0.006,
            Kpi = 0.002,
            Kvp = 0.01,
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

        val sign = if (reverse) -1 else 1

        val desiredHeading = Pathfinder.r2d(leftSeg.heading*sign)
        val angleDifference = Pathfinder.boundHalfDegrees(desiredHeading + Navx.yaw)
        val kp = 1.0
        val turn = kp * (-1.0 / 80.0) * angleDifference


        val left = leftPVA.update(
                currentDistance[0],
                Drivetrain.encVelInSec[0] / 12.0,
                leftSeg.position*sign,
                leftSeg.velocity*sign,
                leftSeg.acceleration*sign
        )

        val right = rightPVA.update(
                currentDistance[1],
                Drivetrain.encVelInSec[1] / 12.0,
                rightSeg.position*sign,
                rightSeg.velocity*sign,
                rightSeg.acceleration*sign
        )
        if (!reverse)
            Drivetrain.openLoopPower = DriveSignal(left + turn, right - turn)
        else
            Drivetrain.openLoopPower = DriveSignal(right - turn, left + turn)
        t++
    }

    override fun end() {
        println("Done")
    }

    override fun isFinished(): Boolean {
        return leftTraj.segments[0].dt * leftTraj.segments.size <= elapsedTime // i guess?
    }
}