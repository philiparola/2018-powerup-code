package com.team2898.robot.commands

import com.team2898.engine.extensions.Vector2D.get
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.motion.CheesyDrive
import com.team2898.engine.motion.DriveSignal
import edu.wpi.first.wpilibj.command.Command
import com.team2898.robot.OI
import com.team2898.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj.command.WaitCommand
import java.io.File
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard as sd

class Teleop : Command() {

    val files = listOf<File>(
            File("/home/lvuser/oneVleft.csv"),
            File("/home/lvuser/oneVright.csv"),
            File("/home/lvuser/threeVleft.csv"),
            File("/home/lvuser/threeVright.csv"),
            File("/home/lvuser/sixVleft.csv"),
            File("/home/lvuser/sixVright.csv"),
            File("/home/lvuser/maxVleft.csv"),
            File("/home/lvuser/maxVright.csv")
    )

    override fun initialize() {
//        Drivetrain.controlMode = Drivetrain.ControlModes.VELOCITY_DRIVE
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        File("/home/lvuser/leftMotor.csv").writeText("voltage, velocity\n")
        File("/home/lvuser/rightMotor.csv").writeText("voltage, velocity\n")

        files.forEach {
            it.writeText("voltage, velocity\n")
        }
    }

    override fun execute() {
        Drivetrain.zeroEncoders()

        if (OI.aButton) {
            Drivetrain.openLoopPower = DriveSignal(left = 1.5 / 12.0, right = 1.5 / 12.0)
            File("/home/lvuser/oneVleft.csv").appendText("${Drivetrain.leftMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[0] / 12.0}\n")
            File("/home/lvuser/oneVright.csv").appendText("${Drivetrain.rightMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[1] / 12.0}\n")
            println("${Drivetrain.leftMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[0] / 12.0}\n")
            println("${Drivetrain.rightMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[1] / 12.0}\n")
        } else if (OI.bButton) {
            Drivetrain.openLoopPower = DriveSignal(left = 3 / 12.0, right = 3 / 12.0)
            File("/home/lvuser/threeVleft.csv").appendText("${Drivetrain.leftMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[0] / 12.0}\n")
            File("/home/lvuser/threeVright.csv").appendText("${Drivetrain.rightMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[1] / 12.0}\n")
            println("${Drivetrain.leftMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[0] / 12.0}\n")
            println("${Drivetrain.rightMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[1] / 12.0}\n")
        } else if (OI.xButton) {
            Drivetrain.openLoopPower = DriveSignal(left = 6 / 12.0, right = 6 / 12.0)
            File("/home/lvuser/sixVleft.csv").appendText("${Drivetrain.leftMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[0] / 12.0}\n")
            File("/home/lvuser/sixVright.csv").appendText("${Drivetrain.rightMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[1] / 12.0}\n")
            println("${Drivetrain.leftMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[0] / 12.0}\n")
            println("${Drivetrain.rightMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[1] / 12.0}\n")
        } else if (OI.yButton) {
            Drivetrain.openLoopPower = DriveSignal(left = 9 / 12.0, right = 9 / 12.0)
            File("/home/lvuser/nineVleft.csv").appendText("${Drivetrain.leftMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[0] / 12.0}\n")
            File("/home/lvuser/nineVright.csv").appendText("${Drivetrain.rightMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[1] / 12.0}\n")
            println("${Drivetrain.leftMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[0] / 12.0}\n")
            println("${Drivetrain.rightMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[1] / 12.0}\n")
        } else if (OI.highGear) {
            Drivetrain.openLoopPower = DriveSignal(left = 1.25 / 12.0, right = 1.25 / 12.0)
            File("/home/lvuser/maxVleft.csv").appendText("${Drivetrain.leftMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[0] / 12.0}\n")
            File("/home/lvuser/maxVright.csv").appendText("${Drivetrain.rightMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[1] / 12.0}\n")
            println("${Drivetrain.leftMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[0] / 12.0}\n")
            println("${Drivetrain.rightMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[1] / 12.0}\n")
        } else {
            CheesyDrive.updateQuickTurn(OI.quickTurn)
            Drivetrain.openLoopPower = CheesyDrive.updateCheesy(
                    (if (!OI.quickTurn) OI.turn else -OI.leftTrigger + OI.rightTrigger),
                    -OI.throttle,
                    OI.quickTurn,
                    true
            )
        }

    }

    override fun isFinished(): Boolean = false
}

//            File("/home/lvuser/leftMotor.csv").appendText("${Drivetrain.leftMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[0]/12}\n")
//            File("/home/lvuser/rightMotor.csv").appendText("${Drivetrain.rightMaster.motorOutputVoltage}, ${Drivetrain.encVelInSec[1]/12}\n")

