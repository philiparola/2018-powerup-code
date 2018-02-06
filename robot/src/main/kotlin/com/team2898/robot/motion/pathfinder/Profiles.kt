package com.team2898.robot.motion.pathfinder

import jaci.pathfinder.Waypoint

val baselineProfile = arrayOf(
        // x, y, degrees
        Waypoint(0.0, 0.0, 0.0),
        Waypoint(9.0, 0.0, 0.0)
)

val switchProfile = arrayOf(
        Waypoint(0.0, 13.0, 0.0),
        Waypoint(1.0, 13.0, 0.0),
        Waypoint(10.0, 8.0, 0.0),
        Waypoint(11.0, 8.0, 0.0)
)
