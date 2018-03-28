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
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.JSON


object CubeLidar : ILooper {
    override val loop = AsyncLooper(25.0) {
        val result: String? = arduino.readString()
        //println(result)
        if (result != null && result.length > 10) {
            try {
                val json = result
                //val res = JSON().parse<Response>(json)
                val res = json.split(',')
                distances = Quadruple(res[0].toInt(), res[1].toInt(), res[2].toInt(), res[3].toInt())
            } catch (e: Exception) {
            }
        }

    }
    override val enableTimes = listOf(GamePeriods.AUTO, GamePeriods.TELEOP, GamePeriods.DISABLE)

    @Serializable
    data class Response(val MEASURE_1: Double, val MEASURE_2: Double, val MEASURE_3: Double, val MEASURE_4: Double)

    var distances = Quadruple(0, 0, 0, 0)

    val arduino = SerialPort(9600, SerialPort.Port.kUSB1, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne)
    val cubeOrientation: Rotation2d
        get() = Rotation2d()

    init {
        arduino.enableTermination('\n')
    }

}
