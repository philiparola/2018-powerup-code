package com.team2898.robot.motion.pathfinder

import com.team2898.engine.async.util.go
import com.team2898.engine.logging.LogLevel
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.reflectLocation
import com.team2898.engine.motion.DriveSignal
import com.team2898.robot.subsystems.Drivetrain
import jaci.pathfinder.Trajectory
import kotlinx.coroutines.experimental.delay

object ProfileExecutor {

    fun execute(motorCommand: (Double, Double) -> Unit) =
            go {
                if (!ProfileGenerator.generatedProfile.isCompleted) {
                    Logger.logInfo(reflectLocation(), LogLevel.WARNING, "Profile not done generating! Waiting until completion to execute")
                }
                ProfileGenerator.generatedProfile.await()

                // TODO: Make cascading or 3DOF pid loop
                ProfileGenerator.generatedProfile.getCompleted().apply {
                    for (i in 0..first.segments.size) {
                        Drivetrain.closedLoopVelTarget = DriveSignal(
                                first.segments[i].velocity,
                                second.segments[i].velocity
                        )
                        delay(
                                second.segments[i].dt.toLong()
                        )
                    }

                }

            }

}