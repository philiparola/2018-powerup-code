package com.team2898.robot.commands.auto

import com.team2898.robot.commands.ProfileFollower
import com.team2898.robot.commands.SetElevator
import com.team2898.robot.commands.manipulator.LightThrow
import com.team2898.robot.motion.pathfinder.ProfileGenerator
import com.team2898.robot.motion.pathfinder.ProfilesSettings.leftSwitchFromRight
import com.team2898.robot.motion.pathfinder.ProfilesSettings.rightSwitchFromRight
import edu.wpi.first.wpilibj.command.CommandGroup
import edu.wpi.first.wpilibj.command.WaitForChildren
import openrio.powerup.MatchData

class SwitchFromRight: CommandGroup() {
    init {
        addParallel(SetElevator(2.0, false))
        if (MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR) == MatchData.OwnedSide.RIGHT)
            addParallel(ProfileFollower(ProfileGenerator.deferProfile(rightSwitchFromRight)))
        if (MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR) == MatchData.OwnedSide.LEFT)
            addParallel(ProfileFollower(ProfileGenerator.deferProfile(leftSwitchFromRight)))
        addSequential(WaitForChildren())
        addSequential(LightThrow())
    }
}