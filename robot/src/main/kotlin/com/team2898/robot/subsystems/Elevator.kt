package com.team2898.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlFrame
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced
import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.ILooper
import com.team2898.engine.logic.ISelfCheck
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.motion.DriveSignal
import com.team2898.engine.motion.TalonWrapper
import com.team2898.robot.config.ELEV_TALON_ID
import com.team2898.robot.config.SLAVE_SRX

object Elevator: Subsystem(name ="Elevator", loopHz = 100.0), ISelfCheck, ILooper {

    val motor = TalonWrapper(ELEV_TALON_ID)
    val slaveMotor = TalonWrapper(SLAVE_SRX)


    init {
        motor.apply {
            slaveMotor slaveTo motor

        }
    }

    val encRawVel: Int
        get() = motor.sensorCollection.quadratureVelocity

    override val loop: AsyncLooper
        get() = super.loop

    override val enableTimes: List<GamePeriods> = listOf(GamePeriods.TELEOP, GamePeriods.AUTO)

    override fun onStart() {
        motor.sensorCollection.setAnalogPosition(0, 0)
    }
    override fun onLoop() {
    }

    override fun onStop() {
    }

    override fun selfCheckup(): Boolean {
        return true
    }

    override fun selfTest(): Boolean {
        return true
    }

}

