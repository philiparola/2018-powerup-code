package com.team2898.robot.motion.pathfinder

import com.team2898.engine.async.pools.ComputePool
import com.team2898.engine.async.util.go
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import jaci.pathfinder.Waypoint
import jaci.pathfinder.modifiers.TankModifier
import kotlinx.coroutines.experimental.Deferred
import kotlinx.coroutines.experimental.async
import java.io.File
import java.util.concurrent.Future

object ProfileGenerator {
    val hz = 100
    val maxVel = 168.0 // in/s
    val maxAcc = 168.0 / 2 // in/s^2
    val maxJerk = 300.0 // in/s^3

    val wheelbaseWidth = 0.75 // m

    val config = Trajectory.Config(
            Trajectory.FitMethod.HERMITE_CUBIC,
            Trajectory.Config.SAMPLES_HIGH,
            1.0 / hz, maxVel, maxAcc, maxJerk)

    fun deferProfile(profile: List<Triple<Double, Double, Double>>): Deferred<Pair<Trajectory, Trajectory>> = async(ComputePool) {
        val waypoints: Array<Waypoint> = profile.map { it ->
            Waypoint(it.first, it.second, Pathfinder.d2r(it.third))
        }.toTypedArray()

        val trajectory = Pathfinder.generate(waypoints, config)
        val modifier = TankModifier(trajectory).modify(wheelbaseWidth)

        val sb = StringBuilder()

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

        File("/home/lvuser/testNoTank.csv").writeText(sb.toString())

        Pair<Trajectory, Trajectory>(modifier.leftTrajectory, modifier.rightTrajectory)
    }
}
