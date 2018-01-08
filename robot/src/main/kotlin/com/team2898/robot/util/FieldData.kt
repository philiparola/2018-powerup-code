package com.team2898.robot.util

import edu.wpi.first.wpilibj.DriverStation

object FieldSides {
    val str: String by lazy { DriverStation.getInstance().gameSpecificMessage }

    enum class Sides { LEFT, RIGHT }

    operator fun get(index: Int) =
            when (str) {
                "L" -> Sides.LEFT
                "R" -> Sides.RIGHT
                else -> Sides.LEFT
            }
}