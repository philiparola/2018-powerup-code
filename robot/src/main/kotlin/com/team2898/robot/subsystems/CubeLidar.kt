package com.team2898.robot.subsystems

import com.google.gson.JsonElement
import com.google.gson.JsonParser
import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.ILooper
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.types.Quadruple
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard as sd
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.JSON


object CubeLidar {

    val distances: Quadruple<Double, Double, Double, Double>
        get() =
            Quadruple(
                    sd.getNumber("LIDAR1", 0.0),
                    sd.getNumber("LIDAR2", 0.0),
                    sd.getNumber("LIDAR3", 0.0),
                    sd.getNumber("LIDAR4", 0.0)
            )

    val cubeOrientation: Rotation2d
        get() = Rotation2d.createFromDegrees(
                sd.getNumber("cube degrees", 0.0)
        )

    init {
    }

}
