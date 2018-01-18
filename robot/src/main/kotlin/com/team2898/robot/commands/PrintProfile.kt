package com.team2898.robot.commands

import com.team2898.engine.logging.LogLevel
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.reflectLocation
import edu.wpi.first.wpilibj.command.Command
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import jaci.pathfinder.Waypoint
import jaci.pathfinder.modifiers.TankModifier
import java.io.File
import kotlin.system.measureTimeMillis

class PrintProfile : Command() {
    override fun start() {
        val time = measureTimeMillis {
            val hz = 100.0
            val maxVel = 168.0
            val maxAcc = 168.0 / 2
            val maxJerk = 300.0

            val points = arrayOf(
                    Waypoint(0.0, 0.0, Pathfinder.d2r(0.0)),
                    Waypoint(72.0, 72.0, Pathfinder.d2r(0.0))
            )

            val config = Trajectory.Config(
                    Trajectory.FitMethod.HERMITE_CUBIC,
                    Trajectory.Config.SAMPLES_FAST,
                    1 / hz,
                    maxVel,
                    maxAcc,
                    maxJerk
            )
            val trajectory = Pathfinder.generate(points, config)
            val modifier = TankModifier(trajectory).modify(22.0)

            val left = modifier.leftTrajectory
            val right = modifier.rightTrajectory

            val sb2 = StringBuilder()
            sb2.append("t, left x, right x, left y, right y\n")
            for (i in 0 until left.length() - 1) {
                sb2.append(
                        "$i" +
                                ", ${left[i].x}" +
                                ", ${right[i].x}" +
                                ", ${left[i].y}" +
                                ", ${right[i].y}" +
                                "\n"
                )
            }

            val sb = StringBuilder()
            sb.append("t, position, vel, acc, x, y, heading\n")
            for (i in 0 until trajectory.length() - 1) {
                sb.append(
                        "$i," +
                                "${trajectory[i].position}" +
                                ",${trajectory[i].velocity}" +
                                ",${trajectory[i].acceleration}" +
                                ",${trajectory[i].x}" +
                                ",${trajectory[i].y}" +
                                ",${trajectory[i].heading}" +
                                "\n"
                )
            }
            try {
                File("/home/lvuser/data1.csv").writeText(sb.toString())
                File("/home/lvuser/data2.csv").writeText(sb2.toString())
            } catch (ex: Exception) {
                println(ex.message)
            }
        }
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Time taken to do test profile: $time")
    }

    override fun isFinished(): Boolean {
        return false
    }
}