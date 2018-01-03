package com.team2898.robot

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.logic.GamePeriods
import edu.wpi.first.wpilibj.networktables.NetworkTable as nt

object NTHandler {

    val miscTable = nt.getTable("misc")
    val navTable = nt.getTable("nav")

    fun setGamePeriod(mode: GamePeriods) {
        miscTable.putString("GamePeriod", mode.toString())
    }

    init {
        nt.setUpdateRate(0.01)
    }
}