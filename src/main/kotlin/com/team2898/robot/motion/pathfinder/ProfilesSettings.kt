package com.team2898.robot.motion.pathfinder.ProfilesSettings

import com.team2898.robot.motion.pathfinder.*
import jaci.pathfinder.Trajectory

val rightSwitchFromCenter = ProfileSettings(
        hz = 50,
        maxVel = 3.0,
        maxAcc = 2.0,
        maxJerk = 5.0,
        wheelbaseWidth = 2.2568170930430758,
        wayPoints = convWaypoint(rightSwitchFromCenterProfile),
        fitMethod = Trajectory.FitMethod.HERMITE_CUBIC,
        sampleRate = Trajectory.Config.SAMPLES_HIGH
)

val leftSwitchFromCenter = ProfileSettings(
        hz = 50,
        maxVel = 3.0,
        maxAcc = 2.0,
        maxJerk = 5.0,
        wheelbaseWidth = 2.2568170930430758,
        wayPoints = convWaypoint(leftSwitchFromCenterProfile),
        fitMethod = Trajectory.FitMethod.HERMITE_CUBIC,
        sampleRate = Trajectory.Config.SAMPLES_HIGH
)

val rightSwitchFromLeft = ProfileSettings(
        hz = 50,
        maxVel = 3.0,
        maxAcc = 2.0,
        maxJerk = 5.0,
        wheelbaseWidth = 2.2568170930430758,
        wayPoints = convWaypoint(rightSwitchFromLeftProfile),
        fitMethod = Trajectory.FitMethod.HERMITE_CUBIC,
        sampleRate = Trajectory.Config.SAMPLES_HIGH
)

val rightSwitchFromRight = ProfileSettings(
        hz = 50,
        maxVel = 3.0,
        maxAcc = 2.0,
        maxJerk = 5.0,
        wheelbaseWidth = 2.2568170930430758,
        wayPoints = convWaypoint(rightSwitchFromRightProfile),
        fitMethod = Trajectory.FitMethod.HERMITE_CUBIC,
        sampleRate = Trajectory.Config.SAMPLES_HIGH
)

val leftSwitchFromRight = ProfileSettings(
        hz = 50,
        maxVel = 3.0,
        maxAcc = 2.0,
        maxJerk = 5.0,
        wheelbaseWidth = 2.2568170930430758,
        wayPoints = convWaypoint(leftSwitchFromRightProfile),
        fitMethod = Trajectory.FitMethod.HERMITE_CUBIC,
        sampleRate = Trajectory.Config.SAMPLES_HIGH
)

val leftSwitchFromLeft = ProfileSettings(
        hz = 50,
        maxVel = 3.0,
        maxAcc = 2.0,
        maxJerk = 5.0,
        wheelbaseWidth = 2.2568170930430758,
        wayPoints = convWaypoint(leftSwitchFromLeftProfile),
        fitMethod = Trajectory.FitMethod.HERMITE_CUBIC,
        sampleRate = Trajectory.Config.SAMPLES_HIGH
)
