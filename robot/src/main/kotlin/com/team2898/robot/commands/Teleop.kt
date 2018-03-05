package com.team2898.robot.commands

import com.team2898.engine.OI.ToggleDebounce
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.math.clamp
import com.team2898.engine.motion.CheesyDrive
import edu.wpi.first.wpilibj.command.Command
import com.team2898.robot.OI
import com.team2898.robot.commands.manipulator.*
import com.team2898.robot.config.ControllerConf.NOENC
import com.team2898.robot.config.ElevatorConf.ELEV_SCALE_HEIGHT
import com.team2898.robot.config.ElevatorConf.ELEV_SWICH_HEIGHT
import com.team2898.robot.config.ElevatorConf.MAX_HEIGHT_FT
import com.team2898.robot.config.ElevatorConf.MIN_HEIGHT_FT
import com.team2898.robot.subsystems.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard as sd
import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D

class Teleop : Command() {
    //    val HOME = "/home/lvuser"
//    val files = listOf<File>(
//            File("$HOME/left2V.csv"),
//            File("$HOME/left6V.csv"),
//            File("$HOME/left9V.csv"),
//            File("$HOME/left12V.csv"),
//            File("$HOME/right2V.csv"),
//            File("$HOME/right6V.csv"),
//            File("$HOME/right9V.csv"),
//            File("$HOME/right12V.csv")
//    )
    var startTime = 0.0

    val elevatorSetpoints = listOf(MIN_HEIGHT_FT, ELEV_SWICH_HEIGHT, ELEV_SCALE_HEIGHT, MAX_HEIGHT_FT)
    var index = 0

    val raiseElevator = ToggleDebounce(onFall = {
        index = clamp(index++, 0, elevatorSetpoints.size - 1)
    })
    val lowerElevator = ToggleDebounce(onFall = {
        index = clamp(index--, 0, elevatorSetpoints.size - 1)
    })
    val raiseIntake = ToggleDebounce(onFall = {
        Intake.talonTargetPos = Rotation2d.createFromDegrees(Intake.currentPos.degrees + 10)
    })
    val lowerIntake = ToggleDebounce(onFall = {
        Intake.talonTargetPos = Rotation2d.createFromDegrees(Intake.currentPos.degrees - 10)

    })

    val switchState = ToggleDebounce(onFall = {
        Intake.switchPistonState()
    })

    override fun initialize() {
        if (!NOENC) {
        }

        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        Drivetrain.zeroEncoders()
        Navx.reset()
        startTime = Timer.getFPGATimestamp()
        Intake.reStart()
    }

    override fun execute() {
        CheesyDrive.updateQuickTurn(OI.quickTurn)
        Drivetrain.uncorrectedOpenLoopPower = CheesyDrive.updateCheesy(
                (if (!OI.quickTurn) OI.turn else -OI.leftTrigger + OI.rightTrigger),
                OI.throttle,
                OI.quickTurn,
                true
        )
        //if (OI.opA) Manipulator.talon.set(ControlMode.PercentOutput, .3)
        //else if(OI.opB) Manipulator.talon.set(ControlMode.PercentOutput, -.3)
        //else Manipulator.talon.set(ControlMode.PercentOutput, 0.0)


        Intake.sparkTargetSpeed = OI.calcIntakeSpeed()
        if (OI.opLShoulder) Intake.sparkTargetSpeed = Vector2D(-0.3, 0.0)
        if (OI.opRShoulder) Intake.sparkTargetSpeed = Vector2D(0.0, -0.3)

        if (OI.openPiston) {
            Intake.pistonState = Intake.PistonState.OPEN
        } else {
            Intake.pistonState = Intake.PistonState.CLOSED
        }

//        if (OI.opA) {
//            Intake.talonTargetPos = Rotation2d.createFromDegrees(80.0)
//        }

        //Elevator.master.set(ControlMode.PercentOutput, OI.opRTrig - OI.opLTrig)


        Intake.leftDeployTalon.set(ControlMode.PercentOutput, OI.opRY/2)

//        Intake.rightDeployTalon.set(ControlMode.PercentOutput, OI.leftTrigger/3)
//        Intake.leftDeployTalon.set(ControlMode.PercentOutput, OI.leftTrigger/3)
//
//        Intake.leftDeployTalon.set(ControlMode.PercentOutput, OI.rightTrigger/3)
//        Intake.rightDeployTalon.set(ControlMode.PercentOutput, OI.rightTrigger/3)


    }

    override fun isFinished(): Boolean = false
}