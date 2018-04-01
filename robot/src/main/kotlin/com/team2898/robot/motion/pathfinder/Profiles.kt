package com.team2898.robot.motion.pathfinder

import jaci.pathfinder.Waypoint

val baselineProfile = arrayOf( //
        Waypoint(0.0, 0.0, 0.0),
        Waypoint(6.0, 0.0, 0.0)
)

val rightSwitchFromCenterProfile = arrayOf( // working, starting from center
        Waypoint(1.5, 13.0, 0.0),
        Waypoint(2.5, 13.0, 0.0),
        Waypoint(9.0, 9.0, 0.0),
        Waypoint(10.0,9.0, 0.0)
)

val leftSwitchFromCenterProfile = arrayOf( // works
        Waypoint(1.5, 13.0, 0.0),
        Waypoint(2.5, 13.0, 0.0),
        Waypoint(9.0, 17.0, 0.0),
        Waypoint(10.0,17.0, 0.0)
)
val leftSwitchFromLeftProfile = arrayOf( // testing, should not work // TODO
        Waypoint(1.5, 23.0, 0.0),
        Waypoint(10.0, 23.0, 0.0),
        Waypoint(14.0, 20.5, 90.0)
)

val rightSwitchFromLeftProfile = arrayOf( // testing, should NOT WORK!!!!! // TODO
        Waypoint(1.5, 23.0, 0.0),
        Waypoint(14.0, 23.0, 0.0),
        Waypoint(19.0, 19.0, 90.0),
        Waypoint(19.0, 6.0, 90.0),
        Waypoint(17.0, 2.0, 180.0),
        Waypoint(14.0, 6.0, -90.0)
)

val rightSwitchFromRightProfile = arrayOf( // testing, not work // TODO
        Waypoint(1.5, 4.0, 0.0),
        Waypoint(10.0, 4.0, 0.0),
        Waypoint(14.0, 6.5, -90.0)
)

val leftSwitchFromRightProfile = arrayOf( // testing, not work // TODO
        Waypoint(1.5, 4.0, 0.0),
        Waypoint(14.0, 4.0, 0.0),
        Waypoint(19.0, 8.0, -90.0),
        Waypoint(19.0, 20.0, -90.0),
        Waypoint(17.0, 24.0, 180.0),
        Waypoint(14.0, 21.0, 90.0)
)