package com.team2898.robot.motion.pathfinder

import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import jaci.pathfinder.Waypoint
import kotlinx.serialization.*

val convWaypoint: (prof: Array<Waypoint>) -> List<Triple<Double, Double, Double>> = { prof ->
    prof.map { point ->
        Triple<Double, Double, Double>(point.x, point.y, point.angle)
    }
}

val tripleToWaypoint: (prof: List<Triple<Double, Double, Double>>) -> Array<Waypoint> = {
    it.map {
        Waypoint(it.first, it.second, Pathfinder.d2r(it.third))
    }.toTypedArray()
}

@Serializable
data class ProfileSettings(val hz: Int = 100,
                           val maxVel: Double,
                           val maxAcc: Double,
                           val maxJerk: Double,
                           val wheelbaseWidth: Double,
                           val wayPoints: List<Triple<Double, Double, Double>>,
                           val fitMethod: Trajectory.FitMethod,
                           val sampleRate: Int
)
