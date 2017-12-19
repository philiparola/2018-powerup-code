package com.team2898.engine.signalProcessing

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.async.IAsyncLoop
import com.team2898.engine.logging.LogLevel
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.reflectLocation

class MovingAverageFilterAsync(averagePeriod: Int, updateHz: Double): MovingAverageFilter(averagePeriod), IAsyncLoop {

    val m_looper: AsyncLooper = AsyncLooper(updateHz) {
        getAverage(input())
    }

    var input: () -> Double = {
        Logger.logInfo(reflectLocation(), LogLevel.ERROR, "Input source not defined")
        0.0
    }

    override fun setTargetHz(hz: Double) = m_looper.setTargetHz(hz)
    override fun stop() = m_looper.stop()
    override fun start() = m_looper.start()
    override fun getLoop() = m_looper

    override fun getAverage(): Double {
        return average()
    }
}