package com.team2898.robot.commands

import com.team2898.engine.extensions.Vector2D.get
import com.team2898.engine.math.avg
import com.team2898.engine.motion.DriveSignal
import com.team2898.robot.motion.pathfinder.*
import com.team2898.robot.subsystems.Drivetrain
import com.team2898.robot.subsystems.Navx
import edu.wpi.first.wpilibj.command.Command
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import jaci.pathfinder.followers.EncoderFollower
import java.io.File


class MotionProfileTest : Command() {
    val testProfile = ProfileSettings(
            hz = 100,
            maxVel = 10.0,
            maxAcc = 5.0,
            maxJerk = 15.0,
            wheelbaseWidth = 2.1,
            wayPoints = convWaypoint(baselineProfile),
            fitMethod = Trajectory.FitMethod.HERMITE_CUBIC,
            sampleRate = Trajectory.Config.SAMPLES_HIGH
    )
    val profile = ProfileGenerator.deferProfile(testProfile)
    val left = EncoderFollower(profile.first)
    val right = EncoderFollower(profile.second)
    val encs = listOf(left, right)


    init {
        println("init")

        left.configureEncoder(Drivetrain.encPosRaw[0].toInt(), 4096, 0.5)
        right.configureEncoder(Drivetrain.encPosRaw[1].toInt(), 4096, 0.5)

        left.configurePIDVA(0.0, 0.0, 0.0, 1 / 15.0, 0.0)
        right.configurePIDVA(0.0, 0.0, 0.0, 1 / 15.0, 0.0)
    }

    override fun initialize() {
        println("starting mp")
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        Navx.reset()
    }

    val sb = StringBuilder().append("t, left, right, r vel, l vel, angle diff, left dis, right dis\n")
    var t = 0

    var headings = 0.0
    var desiredHeading = 0.0
    var angleDifference = 0.0
    var turn = 0.0

    override fun execute() {
        if (isFinished) return
        headings = Navx.yaw
        desiredHeading = Pathfinder.r2d(left.heading)
        angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - headings)

        val kp = 0.6
        val kf = 0.0
        turn = kp * (-1.0 / 80.0) * angleDifference + avg(left.segment.velocity, right.segment.velocity) * kf

        val l = left.calculate(Drivetrain.encPosRaw[0].toInt())
        val r = right.calculate(Drivetrain.encPosRaw[1].toInt())
        sb.append("$t, $r, $l, ${Drivetrain.encVelRaw[0]}, ${Drivetrain.encVelRaw[1]}, $angleDifference, ${Drivetrain.encPosFt[0]}, ${Drivetrain.encPosFt[1]}\n")
        Drivetrain.openLoopPower = DriveSignal(l - turn, r + turn)
        t++
    }

    override fun end() {
        File("/home/lvuser/output.csv").writeText(sb.toString())
        Drivetrain.openLoopPower = DriveSignal(brake = true)
        println("end")
    }

    override fun isFinished(): Boolean {
        return left.isFinished or right.isFinished
    }
}

