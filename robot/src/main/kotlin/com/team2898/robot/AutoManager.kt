package com.team2898.robot

import com.team2898.robot.commands.auto.Baseline
import com.team2898.robot.commands.auto.SwitchFromCenter
import com.team2898.robot.commands.auto.SwitchFromLeft
import com.team2898.robot.commands.auto.SwitchFromRight
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.CommandGroup
import openrio.powerup.MatchData
import java.util.*


object AutoManager {
    enum class StartLocations {
        LEFT, CENTER, RIGHT
    }

    enum class TargetAutos {
        SWITCH, SCALE, TWO_CUBE_SWITCH, TWO_CUBE_SCALE, BASE_LINE
    }

    val invalidAutos = listOf(
            Pair(StartLocations.CENTER, TargetAutos.SCALE),
            Pair(StartLocations.CENTER, TargetAutos.TWO_CUBE_SCALE),
            Pair(StartLocations.LEFT, TargetAutos.SCALE),
            Pair(StartLocations.LEFT, TargetAutos.TWO_CUBE_SCALE)
    )

    data class AutoPair(val start: StartLocations, val target: TargetAutos, val commnad: Command) {
        fun create(): MutableMap<Pair<StartLocations, TargetAutos>, Command> {
            return mutableMapOf<Pair<StartLocations, TargetAutos>, Command> (
                    Pair(Pair(start, target), commnad)
            )
        }
    }

    val leftFallback = TargetAutos.SWITCH
    val rightFallback = TargetAutos.SWITCH
    val centerFallback = TargetAutos.SWITCH

    val switchSide = MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR)
    val scaleSide = MatchData.getOwnedSide(MatchData.GameFeature.SCALE)


    // left, left = left switch
    // left, right = baseline
    // center, left = left switch
    // center, right = right switch
    // right, left = baseline
    // right, right = right switch

    fun produceAuto(start: StartLocations, target: TargetAutos): AutoPair {
        if (target == TargetAutos.SWITCH) {
            return when (start) {
                StartLocations.RIGHT -> AutoPair(start, target, SwitchFromRight())
                StartLocations.CENTER -> AutoPair(start, target, SwitchFromCenter())
                StartLocations.LEFT -> AutoPair(start, target, SwitchFromLeft())
            }
        }
        if (target == TargetAutos.BASE_LINE)
            return AutoPair(start, target, Baseline())
        return AutoPair(start, target, Baseline())
    }
}