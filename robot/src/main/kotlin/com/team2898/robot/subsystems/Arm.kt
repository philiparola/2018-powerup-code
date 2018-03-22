package com.team2898.robot.subsystems

import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.Subsystem

object Arm : Subsystem(100.0, "Arm") {
    override val enableTimes = listOf(GamePeriods.AUTO, GamePeriods.TELEOP)
}
