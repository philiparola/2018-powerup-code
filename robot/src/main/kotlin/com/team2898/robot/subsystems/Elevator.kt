package com.team2898.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlFrame
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.ISelfCheck
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.motion.DriveSignal
import com.team2898.engine.motion.TalonWrapper
import com.team2898.robot.config.ELEV_TALON_ID


object Elevator: Subsystem(name ="Elevator", loopHz = 100.0), ISelfCheck {

    val motor = TalonWrapper(ELEV_TALON_ID)

    init {
        motor.apply {
            setControlFrameHz(mapOf(Pair(ControlFrame.Control_3_General, 1)))
            setFeedbackFrameHz(mapOf(Pair(StatusFrameEnhanced.Status_2_Feedback0, 100)))
        }
    }



    override val enableTimes: List<GamePeriods> = listOf(GamePeriods.TELEOP, GamePeriods.AUTO)

    override fun onStart() {
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

