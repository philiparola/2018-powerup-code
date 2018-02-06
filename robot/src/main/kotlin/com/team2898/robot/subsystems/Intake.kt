package com.team2898.robot.subsystems

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.logic.GamePeriods
import com.team2898.engine.logic.ILooper
import com.team2898.engine.logic.LoopManager
import com.team2898.engine.logic.Subsystem
import com.team2898.engine.motion.TalonWrapper
import com.team2898.robot.config.*
import edu.wpi.first.wpilibj.Spark
import edu.wpi.first.wpilibj.SpeedControllerGroup

object Intake: ILooper, Subsystem(100.0, "Intake") {
    override val enableTimes: List<GamePeriods> = listOf(GamePeriods.AUTO, GamePeriods.TELEOP)

    override val loop: AsyncLooper = AsyncLooper(100.0) {
        sparks.set(sparkTargetSpeed)
        masterTalon.set(talonTargetSpeed)
    }


    val masterTalon = TalonWrapper(INTAKE_MASTER)
    val slaveTalon = TalonWrapper(INTAKE_SLAVE)
    val leftSpark = Spark(LEFT_SPARK)
    val rightSpark = Spark(RIGHT_SPARK)

    val sparks = SpeedControllerGroup(leftSpark, rightSpark)

    var sparkCurrentSpeed = 0.0
    var talonCurrentSpeed = 0.0

    var sparkTargetSpeed = 0.0
    var talonTargetSpeed = 0.0

    init {
        slaveTalon slaveTo masterTalon
        LoopManager.register(this)
        masterTalon.apply {
            configPeakCurrentLimit(INTAKE_PEAK_MAX_AMPS, 0)
            configContinuousCurrentLimit(INTAKE_CONT_MAX_AMPS, 0)
            configPeakCurrentDuration(INTAKE_PEAK_MAX_AMPS_DUR_MS, 0)
            enableCurrentLimit(INTAKE_CURRENT_LIMIT)
        }
    }


    override fun onStart() {
        sparks.set(0.0)
    }

    override fun onLoop() {
    }

    override fun onStop() {
        sparks.set(0.0)
    }

    override fun selfCheckup(): Boolean {
        return true
    }

    override fun selfTest(): Boolean {
        return true
    }

}