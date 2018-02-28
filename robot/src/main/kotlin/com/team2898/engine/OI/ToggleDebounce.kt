package com.team2898.engine.OI

import edu.wpi.first.wpilibj.Timer

class ToggleDebounce {
    var state = false
    var lastTime = Timer.getFPGATimestamp()

    fun buttonPressed(newState: Boolean) {
        if (Timer.getFPGATimestamp() - lastTime > 50) {
            state = newState
            lastTime = Timer.getFPGATimestamp()
        }
    }
}