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
    val maxVel = 4.2672 // mt/s

//    val maxAcc = 168.0 * 2 // in/s^2
    val maxAcc = 4.2672 / 2 // m/s^2
    val maxJerk = 7.62 // m/s^3

    val wheelbaseWidth = 0.5588 // m

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

        Pair<Trajectory, Trajectory>(modifier.leftTrajectory, modifier.rightTrajectory)
    }
}
