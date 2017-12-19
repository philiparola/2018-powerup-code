package com.team2898.engine.async

import com.team2898.engine.logging.TimeBombAsync
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.experimental.*


class AsyncLooper(var hz: Double, val logOnCantKeepUp: Boolean = true, val startStopTimeout: Double = 1.0, val func: () -> Unit) {
    var m_job: Job = init()
    var m_targetHz = hz

    private fun init(): Job {
        return launch(context=CommonPool, start = CoroutineStart.LAZY) {
            while (isActive) {
                val startTime: Long = Timer.getFPGATimestamp().toLong()
                func()
                val deltaTime: Long = Timer.getFPGATimestamp().toLong() - startTime
                if (deltaTime < 1000/hz) delay((1000/m_targetHz - deltaTime).toLong())
                else if (logOnCantKeepUp) logCantKeepUp()
            }
        }
    }

    fun start() {
        m_job = init()
        TimeBombAsync(startStopTimeout) {
            m_job.start()
        }
    }

    fun stop() {
        TimeBombAsync(startStopTimeout) {
            m_job.cancel()
            m_job.join()
        }
    }

    fun logCantKeepUp() {
        throw NotImplementedError("make this func dummy")
    }

    fun setTargetHz(hz: Double) {
        m_targetHz = hz
    }
}
