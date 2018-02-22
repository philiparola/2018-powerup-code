package com.team2898.robot

import edu.wpi.first.wpilibj.command.CommandGroup


object AutoManager {
    enum class StartLocations {
        LEFT, CENTER, RIGHT
    }

    enum class TargetAutos {
        SWITCH, SCALE, TWO_CUBE_SWITCH, TWO_CUBE_SCALE
    }

    val invalidAutos = listOf(
            Pair(StartLocations.CENTER, TargetAutos.SCALE),
            Pair(StartLocations.CENTER, TargetAutos.TWO_CUBE_SCALE)
    )

    val leftFallback = TargetAutos.SWITCH
    val rightFallback = TargetAutos.SWITCH
    val centerFallback = TargetAutos.SWITCH


    internal val StartTargetCmdPairing =
            mutableMapOf<Pair<StartLocations, TargetAutos>, CommandGroup>(
            )


    fun produceAuto(start: StartLocations, target: TargetAutos) {

    }
}