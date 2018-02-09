package com.team2898.robot.motion.pathfinder.ProfilesSettings

import com.team2898.robot.motion.pathfinder.ProfileSettings
import com.team2898.robot.motion.pathfinder.baselineProfile
import com.team2898.robot.motion.pathfinder.convWaypoint
import jaci.pathfinder.Trajectory

val testProfile = ProfileSettings(
        hz = 100,
        maxVel = 10.0,
        maxAcc = 5.0,
        maxJerk = 15.0,
        wheelbaseWidth = 2.1,
        wayPoints = convWaypoint(baselineProfile),
        fitMethod = Trajectory.FitMethod.HERMITE_CUBIC,
        sampleRate = Trajectory.Config.SAMPLES_HIGH
)
