package com.team2898.robot.commands

import com.team2898.engine.extensions.Vector2D.get
import com.team2898.engine.math.avg
import com.team2898.engine.motion.DriveSignal
import com.team2898.robot.motion.pathfinder.*
import com.team2898.robot.subsystems.Drivetrain
import com.team2898.robot.subsystems.Navx
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.WaitCommand
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.followers.EncoderFollower
import java.io.File
import com.team2898.robot.motion.pathfinder.ProfilesSettings.testProfile


class MotionProfileTest : Command() {
    val profile = ProfileGenerator.deferProfile(testProfile)
    val left = EncoderFollower(profile.first)
    val right = EncoderFollower(profile.second)

    init {
        println("init")

        left.configureEncoder(Drivetrain.encPosRaw[0].toInt(), 4096, 0.5)
        right.configureEncoder(Drivetrain.encPosRaw[1].toInt(), 4096, 0.5)

        left.configurePIDVA(0.1, 0.01, 0.01, 1 / 14.6, 0.01)
        right.configurePIDVA(0.1, 0.01, 0.01, 1 / 14.6, 0.01)
    }

    override fun initialize() {
        println("starting mp")
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        Navx.reset()
        Drivetrain.zeroEncoders()
        object: WaitCommand(2.0) {} .start()
    }

    val sb = StringBuilder().append("t, left, right, r vel, l vel, angle diff, left dis, right dis\n")
    var t = 0

    var headings = 0.0
    var desiredHeading = 0.0
    var angleDifference = 0.0
    var turn = 0.0

    val leftSB = StringBuilder().append("left motor, left vel\n")
    val rightSB = StringBuilder().append("right motor, right vel\n")

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
        val leftMotor = l - turn
        val rightMotor = r + turn
        leftSB.append("${l - turn}, ${Drivetrain.encVelInSec[0]/12}\n")
        rightSB.append("${r + turn}, ${Drivetrain.encVelInSec[1]/12}\n")
        t++
    }

    override fun end() {
        File("/home/lvuser/output.csv").writeText(sb.toString())
        File("/home/lvuser/leftMotor.csv").writeText(leftSB.toString())
        File("/home/lvuser/rightMotor.csv").writeText(rightSB.toString())
        Drivetrain.openLoopPower = DriveSignal(brake = true)
        println("end")
    }

    override fun isFinished(): Boolean {
        return left.isFinished or right.isFinished
    }
}