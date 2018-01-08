package com.team2898.robot.motion.pathfinder

import com.team2898.engine.async.pools.ComputePool
import com.team2898.engine.async.util.go
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import jaci.pathfinder.Waypoint
import jaci.pathfinder.modifiers.TankModifier
import kotlinx.coroutines.experimental.CompletableDeferred
import kotlinx.coroutines.experimental.Deferred
import kotlinx.coroutines.experimental.async
import java.util.concurrent.Future

object ProfileGenerator {
    val profile = baselineProfile

    val generatedProfile: Deferred<Pair<Trajectory, Trajectory>>

    val hz = 100
    val maxVel = 2.0 // m/s
    val maxAcc = 2.0 // m/s^2
    val maxJerk = 30.0 // m/s^3

    val wheelbaseWidth = 0.75 // m

    val config = Trajectory.Config(
            Trajectory.FitMethod.HERMITE_CUBIC,
            Trajectory.Config.SAMPLES_HIGH,
            1.0 / hz,
            maxVel,
            maxAcc,
            maxJerk
    )


    init {
        generatedProfile = async(ComputePool) {
            val waypoints = profile.map { it ->
                Waypoint(it.first, it.second, Pathfinder.d2r(it.third))
            }.toTypedArray()

            val trajectory = Pathfinder.generate(waypoints, config)

            val modifier = TankModifier(trajectory).modify(wheelbaseWidth)

            Pair<Trajectory, Trajectory>(modifier.leftTrajectory, modifier.rightTrajectory)
        }
    }
}