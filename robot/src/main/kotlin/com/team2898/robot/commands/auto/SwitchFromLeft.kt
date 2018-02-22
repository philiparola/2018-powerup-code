package com.team2898.robot.commands.auto

import com.team2898.robot.commands.SetElevator
import edu.wpi.first.wpilibj.command.CommandGroup
import openrio.powerup.MatchData

class SwitchFromLeft: CommandGroup() {
    val switchSide = MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR)
    init {
        addParallel(SetElevator(2.0, false))
//        if (switchSide == MatchData.OwnedSide.RIGHT)
//            addParallel(ProfileFollower(ProfileGenerator.deferProfile()))
    }
}