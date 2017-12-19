package com.team2898.engine.logging

import com.team2898.engine.async.util.go

class TimeBombAsync(time: Double, throwException: Boolean = true, printLogger: Boolean = true, runOnEnd: () -> Unit = {}, block: suspend () -> Unit) {
    val tb: TimeBomb = TimeBomb(time=time, startArmed=false, throwException=throwException, printLogger=printLogger).apply {
        setOnBlow(runOnEnd)
    }

    init {
        go {
            tb.fuse()
            block()
            if (!tb.blown) tb.defuse()
        }
    }
}
