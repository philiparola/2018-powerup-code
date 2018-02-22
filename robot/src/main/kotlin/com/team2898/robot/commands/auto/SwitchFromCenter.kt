package com.team2898.robot.commands.auto

import com.team2898.robot.commands.ProfileFollower
import com.team2898.robot.commands.SetElevator
import com.team2898.robot.commands.manipulator.LightThrow
import com.team2898.robot.motion.pathfinder.ProfileGenerator
import com.team2898.robot.motion.pathfinder.ProfilesSettings.leftSwitchFromCenter
import com.team2898.robot.motion.pathfinder.ProfilesSettings.rightSwitchFromCenter
import edu.wpi.first.wpilibj.command.CommandGroup
import edu.wpi.first.wpilibj.command.WaitForChildren
import openrio.powerup.MatchData

class SwitchFromCenter(ownedSide: MatchData.OwnedSide): CommandGroup() {
    init {
        addParallel(SetElevator(2.0))
        if (switchSide == MatchData.OwnedSide.LEFT)
            addParallel(ProfileFollower(ProfileGenerator.deferProfile(leftSwitchFromCenter)))
        if (switchSide == MatchData.OwnedSide.RIGHT)
            addParallel(ProfileFollower(ProfileGenerator.deferProfile(rightSwitchFromCenter)))
        addSequential(WaitForChildren())
        addSequential(LightThrow())
    }
}