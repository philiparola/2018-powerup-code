package com.team2898.robot.commands

import com.team2898.engine.logging.LogLevel
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.reflectLocation
import com.team2898.engine.motion.DriveSignal
import com.team2898.robot.motion.pathfinder.ProfileExecutor
import com.team2898.robot.motion.pathfinder.ProfileGenerator
import com.team2898.robot.motion.pathfinder.baselineProfile
import com.team2898.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.Command
import kotlinx.coroutines.experimental.async
import kotlinx.coroutines.experimental.defer
import java.io.File
import java.sql.DriverAction


class MotionProfileTest : Command() {

    val startTime: Double by lazy { Timer.getFPGATimestamp() }

    val executer: ProfileExecutor

    init {
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Starting profile generation at $startTime")
        val deferredProfile = ProfileGenerator.deferProfile(baselineProfile)

        deferredProfile.invokeOnCompletion {
            Logger.logInfo(reflectLocation(), LogLevel.INFO, "Finished profile, dt of ${Timer.getFPGATimestamp() - startTime}")
            val prof = deferredProfile.getCompleted()
            val left = prof.first
            val right = prof.second
            val length = left.segments.size
            val sb = StringBuilder()
            sb.append(
                    "t, leftPos, rightPos, leftVel, rightVel, leftAcc, rightAcc, leftX, rightX, leftY, rightY, leftHeading, rightHeading\n"
            )
            for (i in 0 until length - 1) {
                sb.append(
                        "$i," +
                                "${left[i].position}" +
                                ",${right[i].position}" +
                                ",${left[i].velocity}" +
                                ",${right[i].velocity}" +
                                ",${left[i].acceleration}" +
                                ",${right[i].acceleration}" +
                                ",${left[i].x}" +
                                ",${right[i].x}" +
                                ",${left[i].y}" +
                                ",${right[i].y}" +
                                ",${left[i].heading}" +
                                ",${right[i].heading}" +
                                "\n"
                )
            }

            File("/home/lvuser/test.csv").writeText(sb.toString())

        }
        executer = ProfileExecutor(deferredProfile)
    }

    override fun start() {
        val file = File("/home/lvuser/motorOutput2.csv")
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Running motion profile")
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        var i = 0
        val sb = StringBuilder()
        sb.append("time, left, right\n")
        file.writeText("time, left, right\n")
        executer.execute { left, right ->
            sb.append("$i, $left, $right\n")
            file.appendText("$i, $left, $right\n")
            println("left is $left")
            println("right is $right")
            Drivetrain.openLoopPower = DriveSignal(left, right)
            println("i is $i")
            i++
        }
        println("done")
        File("/home/lvuser/motorOutput.csv").writeText(sb.toString())
    }

    override fun end() {
        Logger.logInfo(reflectLocation(), LogLevel.INFO, "Ending motion profile")
    }

    override fun isFinished(): Boolean =
            executer.completed

}