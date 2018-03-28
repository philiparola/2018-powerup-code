package com.team2898.robot.commands

import com.team2898.engine.kinematics.Rotation2d
import com.team2898.robot.motion.pathfinder.*
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.CommandGroup
import edu.wpi.first.wpilibj.command.WaitForChildren
import jaci.pathfinder.Trajectory

class SwitchFromCenter(val ourColorOnRight: Boolean) : CommandGroup() {
    init {
        //addParallel(SetArm(Rotation2d.createFromDegrees(45.0)))
        addSequential(ProfileFollower(
                ProfileGenerator.genProfile(
                        ProfileSettings(
                                hz = 100,
                                maxVel = 3.5,
                                maxAcc = 2.0,
                                maxJerk = 10.0,
                                wheelbaseWidth = 2.1,
                                wayPoints = convWaypoint(if (ourColorOnRight) rightSwitchFromCenterProfile else leftSwitchFromCenterProfile),
                                fitMethod = Trajectory.FitMethod.HERMITE_CUBIC,
                                sampleRate = Trajectory.Config.SAMPLES_HIGH
                        )
                )
        ))
        addSequential(WaitForChildren())
        //addSequential(IntakeCommand(time = 0.5, power = Pair(1.0, 1.0)))
    }
}