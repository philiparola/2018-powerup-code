package com.team2898.robot.commands

import com.team2898.engine.extensions.Vector2D.get
import com.team2898.engine.motion.DriveSignal
import com.team2898.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.Command
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import jaci.pathfinder.Trajectory.Config.SAMPLES_HIGH
import jaci.pathfinder.Waypoint
import jaci.pathfinder.followers.EncoderFollower
import jaci.pathfinder.modifiers.TankModifier
import java.io.File


class MotionProfileTest : Command() {

    val startTime: Double by lazy { Timer.getFPGATimestamp() }

    lateinit var leftCntl: EncoderFollower
    lateinit var rightCntl: EncoderFollower

    init {

        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP

        println("Starting MP gen")

        fun d2r(t: Double): Double = Pathfinder.d2r(t)

        val path = arrayOf(
                Waypoint(0.0, 0.0, d2r(0.0)),
                Waypoint(2.0, 2.0, d2r(0.0)),
                Waypoint(4.0, 2.0, d2r(0.0))
        )

        val config = Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, SAMPLES_HIGH,
                0.05, 3.0, 2.0, 60.0)

        val traj = Pathfinder.generate(path, config)

        val modified = TankModifier(traj).modify(0.63)

        val leftTraj = modified.leftTrajectory
        val rightTraj = modified.rightTrajectory

        leftCntl = EncoderFollower(modified.leftTrajectory)
        rightCntl = EncoderFollower(modified.rightTrajectory)
        val encFollowers = listOf(leftCntl, rightCntl)

        Drivetrain.masters {
            sensorCollection.setQuadraturePosition(0, 0)
        }


        encFollowers.forEach {
            it.apply {
                configureEncoder(0, 4096, 0.1524)
                configurePIDVA(0.0, 0.0, 0.0, 1 / 3.0, 0.0)
            }
        }

        val left = modified.leftTrajectory
        val right = modified.rightTrajectory

        println("Ending MP gen")
        val sb = StringBuilder()

        sb.append("time,left x,left y,right x,right y,left vel,right vel,left acc,right acc\n")
        for (i in 0 until left.length() - 1) {
            sb.append(
                    "${i * 0.05}",
                    ", ${left[i].x}",
                    ", ${left[i].y}",
                    ", ${right[i].x}",
                    ", ${right[i].y}",
                    ", ${left[i].velocity}",
                    ", ${right[i].velocity}",
                    ", ${left[i].acceleration}",
                    ", ${right[i].acceleration}",
                    "\n"
            )
        }
        File("/home/lvuser/pathfinder.csv").writeText(sb.toString())
    }
    val sb = StringBuilder().append("time, left vel, right vel\n")
    var time = 0
    override fun execute() {
        if (isFinished()) return
        val l = leftCntl.calculate(Drivetrain.encPosRaw[0].toInt())
        val r = rightCntl.calculate(Drivetrain.encPosRaw[1].toInt())
//        leftCntl.segment.apply {
//            val sb = StringBuilder()
//            sb.append(", ${this.x}", ", ${this.y}", ", ${this.velocity}", ", ${this.acceleration}")
//            println("Left segment: ${sb.toString()}")
//        }
//
//        rightCntl.segment.apply {
//            val sb = StringBuilder()
//            sb.append(", ${this.x}", ", ${this.y}", ", ${this.velocity}", ", ${this.acceleration}")
//            println("Right segment: ${sb.toString()}")
//        }
        Drivetrain.openLoopPower = DriveSignal(l, r)
        println("${Drivetrain.openLoopPower}")
        print("left enc vel raw = ${Drivetrain.encVelRaw[0]} ")
        println("right enc vel raw = ${Drivetrain.encVelRaw[1]}")
        println("left enc vel m/s = ${Drivetrain.encVelRaw[0] * 0.04601806640625 * 0.0254}")
        println("right enc vel m/s = ${Drivetrain.encVelRaw[1] * 0.04601806640625 * 0.0254}")
        sb.append("$time" +
                ", ${Drivetrain.encVelRaw[0] * 0.04601806640625 * 0.0254 * -1}" +
                ", ${Drivetrain.encVelRaw[1] * 0.04601806640625 * 0.0254}" +
                "\n")
        time ++
    }

    override fun end() {
        File("/home/lvuser/motorVel.csv").writeText(sb.toString())
        println("ending mp")
    }

    override fun isFinished(): Boolean = leftCntl.isFinished && rightCntl.isFinished


}

