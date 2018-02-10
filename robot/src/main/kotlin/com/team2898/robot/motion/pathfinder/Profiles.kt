package com.team2898.robot.motion.pathfinder

import jaci.pathfinder.Waypoint

val baselineProfile = arrayOf(
        Waypoint(0.0, 0.0, 0.0),
        Waypoint(6.0, 0.0, 0.0)
)

val switchProfile = arrayOf(
        Waypoint(1.5, 13.0, 0.0),
        Waypoint(2.5, 13.0, 0.0),
        Waypoint(9.5, 7.0, 0.0),
        Waypoint(10.5, 7.0, 0.0)

)

val testSwichProfile = arrayOf(
        Waypoint(0.0, 13.0, 0.0),
        Waypoint(3.0, 11.0, 0.0),
        Waypoint(8.0, 10.0, 0.0),
        Waypoint(9.0, 10.0, 0.0),
        Waypoint(11.5, 10.0, 0.0)
)
