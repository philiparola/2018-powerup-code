package com.team2898.robot.commands

import com.ctre.phoenix.motorcontrol.ControlMode
import com.team2898.engine.extensions.get
import com.team2898.engine.motion.DriveSignal
import com.team2898.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj.command.Command
import kotlin.math.abs

class DtMM(val distances: Pair<Int, Int>) : Command() {
    override fun isFinished(): Boolean =
            abs(Drivetrain.encPosRaw[0] - distances[0]) < 512 &&
                    abs(Drivetrain.encPosRaw[1] - distances[1]) < 512


    override fun initialize() {
        Drivetrain.zeroEncoders()
        Drivetrain.controlMode = Drivetrain.ControlModes.MOTION_MAGIC
    }

    override fun execute() {
        Drivetrain.mmDistance = DriveSignal(distances[0].toDouble(), distances[1].toDouble())
        Drivetrain.leftMaster.set(ControlMode.MotionMagic, distances[0].toDouble())
        Drivetrain.rightMaster.set(ControlMode.MotionMagic, distances[1].toDouble())
    }

    override fun end() {
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        Drivetrain.openLoopPower = DriveSignal.BRAKE
    }
}
