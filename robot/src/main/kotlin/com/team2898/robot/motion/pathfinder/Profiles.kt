package com.team2898.robot.motion.pathfinder

import jaci.pathfinder.Waypoint
import kotlin.math.PI

val baselineProfile = arrayOf( //
        Waypoint(0.0, 0.0, 0.0),
        Waypoint(6.0, 0.0, 0.0)
)

val switchFromCenterProfile = arrayOf( // working, starting from center
        Waypoint(1.5, 13.0, 0.0),
        Waypoint(2.5, 13.0, 0.0),
        Waypoint(9.5, 9.0, 0.0),
        Waypoint(10.5,9.0, 0.0)
)

val testSwichProfile = arrayOf(
        Waypoint(0.0, 13.0, 0.0),
        Waypoint(3.0, 11.0, 0.0),
        Waypoint(8.0, 10.0, 0.0),
        Waypoint(9.0, 10.0, 0.0),
        Waypoint(11.5, 10.0, 0.0)
)

val switchFromLeftToSide = arrayOf(
        Waypoint(1.5, 23.0, 0.0),
        Waypoint(9.0, 23.0, 0.0),
        Waypoint(13.0, 20.0, -90.0)
)

