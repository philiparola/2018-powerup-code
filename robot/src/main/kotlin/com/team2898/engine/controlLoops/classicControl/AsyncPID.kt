package com.team2898.engine.controlLoops

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.async.IAsyncLoop
import edu.wpi.first.wpilibj.Timer

/* import edu.wpi.first.wpilibj.util.BoundaryException */

/**
 * Asynchronous PID control loop wrapper
 * Inherits a standard synchronous PID loop as well as Kotlin's concurrent Job system to provide "true" concurrency (none of that notifier trash)
 * @author Solomon
 */

open class AsyncPID(
        Kp: Double=0.0,
        Ki: Double = 0.0,
        Kd: Double = 0.0,
        Kf: Double = 0.0,
        integratorMax: Double = Double.POSITIVE_INFINITY,
        currentTime: Double = Timer.getFPGATimestamp(),
        execHz: Double
): StandardPID(Kp, Ki, Kd, Kf, integratorMax, currentTime), IAsyncLoop {

    var m_sensorValue: Double = 0.0 // Current sensor value
    var m_sensorOutput: Double = 0.0

    var m_asyncLooper: AsyncLooper = AsyncLooper(execHz) {
        m_sensorValue = getSensorInput()
        m_sensorOutput = update(m_sensorOutput, currentTime = Timer.getFPGATimestamp())
    }

    override fun getLoop(): AsyncLooper = m_asyncLooper

    @Synchronized
    override fun start() = m_asyncLooper.start()

    @Synchronized
    override fun stop() = m_asyncLooper.stop()

    @Synchronized
    override fun setTargetHz(hz: Double) = m_asyncLooper.setTargetHz(hz)


    // Override this and make it return the input from the relavent sensor
    val getSensorInput: () -> Double = {0.0}

    // Override this and use it to utilize the output from the PID controller
    val useSensorOutput: (Double) -> Unit = {}


}
