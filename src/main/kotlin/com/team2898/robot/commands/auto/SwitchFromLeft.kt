package com.team2898.robot.commands.auto

import com.team2898.robot.commands.ProfileFollower
import com.team2898.robot.commands.SetElevator
import com.team2898.robot.commands.manipulator.LightThrow
import com.team2898.robot.motion.pathfinder.ProfileGenerator
import com.team2898.robot.motion.pathfinder.ProfilesSettings.leftSwitchFromLeft
import com.team2898.robot.motion.pathfinder.ProfilesSettings.rightSwitchFromLeft
import com.team2898.robot.motion.pathfinder.ProfilesSettings.rightSwitchFromRight
import edu.wpi.first.wpilibj.command.CommandGroup
import edu.wpi.first.wpilibj.command.WaitForChildren
import openrio.powerup.MatchData

class SwitchFromLeft: CommandGroup() {
    val switchSide = MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR)
    init {
        addParallel(SetElevator(2.0, false))
        if (switchSide == MatchData.OwnedSide.RIGHT)
            addParallel(ProfileFollower(ProfileGenerator.deferProfile(rightSwitchFromLeft)))
        if (switchSide == MatchData.OwnedSide.LEFT)
            addParallel(ProfileFollower(ProfileGenerator.deferProfile(leftSwitchFromLeft)))
        addSequential(WaitForChildren())
        addSequential(LightThrow())
    }
}