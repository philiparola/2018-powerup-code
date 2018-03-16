package com.team2898.robot.commands

import com.ctre.phoenix.motorcontrol.ControlMode
import com.team2898.engine.controlLoops.classicControl.PVAPID
import com.team2898.engine.motion.TrapezoidProfile
import com.team2898.robot.subsystems.TestTalon
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.Command
import java.io.File

class Test: Command() {
    val talon = TestTalon()
    val currentPos = {
        talon.talon.sensorCollection.quadraturePosition.toDouble()
    }
    val currentVel = {
        talon.talon.sensorCollection.quadratureVelocity.toDouble()
    }

    val pva = PVAPID(0.0, 0.0, 0.0, {0.0}, 0.0, {0.0})

    val tester = TrapezoidProfile(400.0, 500.0, currentVel, currentPos)

    override fun initialize() {
        println("Test com")
        tester.updateTarget(2000.0)
    }

    val sb = StringBuilder().append("time, vel, pos, acc\n")
    override fun execute() {
        if (tester.isinished()) return
        val prof = tester.update()
//        talon.talon.set(ControlMode.PercentOutput, pva.update(
//                position = prof.currentPos,
//                velocity = prof.currentVel,
//                targetVel = prof.targetVel,
//                targetPos = prof.targetPos,
//                targetAcc = prof.targetAcc
//        ))
        sb.append("${prof.time}, ${prof.targetVel}, ${prof.targetPos}, ${prof.targetAcc}\n")
    }

    override fun isFinished(): Boolean {
        if (tester.isinished()) {
            File("/home/lvuser/test.csv").writeText(sb.toString())
            print("test com done")
            return true
        }
        return tester.isinished()
    }
}