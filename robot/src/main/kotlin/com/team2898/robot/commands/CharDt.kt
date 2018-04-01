package com.team2898.robot.commands

import com.team2898.engine.extensions.get
import com.team2898.engine.motion.DriveSignal
import com.team2898.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.TimedCommand
import java.io.File

class CharDt(val step: Double) : TimedCommand(2.0) {

    var csv = ""
    var last = Pair(0.0, 0.0)
    var lastTime = Timer.getFPGATimestamp()
    override fun initialize() {
        Drivetrain.openLoopPower = DriveSignal.NEUTRAL
        last = Drivetrain.encVelInSec
        lastTime = Timer.getFPGATimestamp()
    }

    override fun execute() {
        val dt = Timer.getFPGATimestamp() - lastTime
        val vels = Drivetrain.encVelInSec
        csv += "${vels[0]},${vels[1]},${(vels[0] - last[0]) * dt},${(vels[1] - last[1]) * dt}\n"

        last = vels
        lastTime += dt // so that it's consistent or whatever

        Drivetrain.openLoopPower = DriveSignal(step/12.0,step/12.0)
    }

    override fun end() {
        Drivetrain.openLoopPower = DriveSignal.NEUTRAL
        File("/home/lvuser/${step}v_char.csv").writeText(csv)
    }
    override fun cancel() = end()
}