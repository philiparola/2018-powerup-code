package com.team2898.robot.motion.pathfinder

import jaci.pathfinder.Waypoint

val baselineProfile = listOf<Triple<Double, Double, Double>>(
        // x, y, degrees
        Triple(0.0, 0.0, 0.0),
        Triple(3.0*12, 0.0, 0.0),
        Triple(12.0*12, 12.0*12, 45.0),
        Triple(12.0*12, 24.0*12, 90.0)
)
