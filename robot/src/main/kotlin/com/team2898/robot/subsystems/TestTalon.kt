package com.team2898.robot.subsystems

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.motion.TalonWrapper
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

class TestTalon {
    val talon = TalonWrapper(4)
    init {
        talon.setMagEncoder()
    }

    fun start() {
        AsyncLooper(100.0) {
            SmartDashboard.putNumber("quad pos", talon.sensorCollection.quadraturePosition.toDouble())
            SmartDashboard.putNumber("quad vel", talon.sensorCollection.quadratureVelocity.toDouble())
            SmartDashboard.putNumber("pwm pos", talon.pwmPos.toDouble())
            SmartDashboard.putNumber("test", talon.getSelectedSensorPosition(0).toDouble())
        }.start()
    }

    fun zero() {
        talon.sensorCollection.setQuadraturePosition(0, 0)
    }
}