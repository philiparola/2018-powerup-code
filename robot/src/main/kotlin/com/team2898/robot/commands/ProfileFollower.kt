package com.team2898.robot.commands

import com.team2898.engine.controlLoops.classicControl.PVAPID
import com.team2898.engine.extensions.Vector2D.get
import com.team2898.engine.extensions.Vector2D.minus
import com.team2898.engine.motion.DriveSignal
import com.team2898.robot.config.DrivetrainConf.*
import com.team2898.robot.subsystems.Drivetrain
import com.team2898.robot.subsystems.Navx
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.Command
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D
import kotlin.math.abs
import kotlin.math.sign

class ProfileFollower(profile: Pair<Trajectory, Trajectory>) : Command() {
    val leftTraj = profile.first
    val rightTraj = profile.second

    var encoderOffset = Vector2D(0.0, 0.0)
    var currentSegment = 0
    var startTime = 0.0

    val elapsedTime // current time
        get() = Timer.getFPGATimestamp() - startTime
    val encoderDistance
        get() = Drivetrain.encPosFt - encoderOffset

    val leftPVA = PVAPID(
            Kp = 0.0,
            Ki = 0.0,
            Kvp = 0.0,
            Kvf = { speed: Double ->
                // speed = f/s
                ((if (abs(speed) > 0.1) abs(speed) - LEFT_B else 0.0) / LEFT_M) / 12.0 * sign(speed) // motor setpoint
            },
            Kaf = 0.0,
            Kpf = { 0.0 }
    )

    val rightPVA = PVAPID(
            Kp = 0.0,
            Ki = 0.0,
            Kvp = 0.0,
            Kvf = { speed: Double ->
                // speed = f/s
                ((if (abs(speed) > 0.1) abs(speed) - RIGHT_B else 0.0) / RIGHT_M) / 12.0 * sign(speed) // motor setpoint
            },
            Kaf = 0.0,
            Kpf = { 0.0 }
    )


    override fun initialize() {
        encoderOffset = Drivetrain.encPosFt
        assert(leftTraj.length() == rightTraj.length())
        currentSegment = 0
        startTime = Timer.getFPGATimestamp()
    }

    override fun execute() {
        val currentDistance = Drivetrain.encPosFt - encoderOffset
        currentSegment = (elapsedTime / leftTraj.segments[0].dt).toInt()// should get the current segment based on timestamp...

        val leftSeg = leftTraj.segments.get(currentSegment)
        val rightSeg = rightTraj.segments.get(currentSegment)

        val desiredHeading = Pathfinder.r2d(leftSeg.heading)
        val angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - Navx.yaw)
        val kp = 0.6
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
        Drivetrain.openLoopPower = DriveSignal(left - turn, right + turn)
    }

    override fun end() {

    }

    override fun isFinished(): Boolean {
        return leftTraj.segments[0].dt * leftTraj.segments.size >= elapsedTime // i guess?
    }
}