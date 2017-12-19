package com.team2898.engine.logic

object LoopManager {
    val loops= mutableListOf<ILooper>()

    @Synchronized fun register(loop: ILooper) {
        loops.add(loop)
    }

    fun onDisable() {
        loops.forEach { iloop ->
            if (iloop.enableTimes.contains(GamePeriods.DISABLE)) iloop.loop.start()
            else iloop.loop.stop()
        }
    }

    fun onAutonomous() {
        loops.forEach { iloop ->
            if (iloop.enableTimes.contains(GamePeriods.AUTO)) iloop.loop.start()
            else iloop.loop.stop()
        }
    }

    fun onTeleop() {
        loops.forEach { iloop ->
            if (iloop.enableTimes.contains(GamePeriods.TELEOP)) iloop.loop.start()
            else iloop.loop.stop()
        }
    }
}