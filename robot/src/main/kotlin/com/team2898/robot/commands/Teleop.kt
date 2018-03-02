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
        Intake.talonTargetPos = Rotation2d.createFromDegrees(Intake.currentPos.first.degrees + 10)
    })
    val lowerIntake = ToggleDebounce(onFall = {
        Intake.talonTargetPos = Rotation2d.createFromDegrees(Intake.currentPos.first.degrees - 10)
    })

    override fun initialize() {
        if (!NOENC) {
        }

        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        Drivetrain.zeroEncoders()
        Navx.reset()
        startTime = Timer.getFPGATimestamp()
    }

    override fun execute() {
        CheesyDrive.updateQuickTurn(OI.quickTurn)
        Drivetrain.uncorrectedOpenLoopPower = CheesyDrive.updateCheesy(
                (if (!OI.quickTurn) OI.turn else -OI.leftTrigger + OI.rightTrigger),
                -OI.throttle,
                OI.quickTurn,
                true
        )
        if (NOENC) {
            if (OI.aButton) Manipulator.talon.set(0.3)
            else if (OI.bButton) Manipulator.talon.set(-0.3)
            else Manipulator.talon.set(0.0)
            if (OI.raiseElev) Elevator.master.set(0.4)
            else if (OI.lowerElev) Elevator.master.set(-0.4)
            else Elevator.master.set(0.0)
        } else {
            if (OI.aButton) Hold().start()
            if (OI.bButton) ManipIntake().start()
            if (OI.xButton) LightThrow().start()
            if (OI.yButton) HeavyThrow().start()

            ////// intake piston ////
            if (OI.closeIntake) Intake.pistonState = Intake.PistonState.CLOSED
            else Intake.pistonState = Intake.PistonState.OPEN

            ////// Elevator /////
            Elevator.targetPosFt = elevatorSetpoints[index]

            raiseIntake.buttonPressed(OI.raiseIntake)
            lowerIntake.buttonPressed(OI.lowerIntake)
            raiseElevator.buttonPressed(OI.raiseElev)
            lowerElevator.buttonPressed(OI.lowerElev)
        }

        ////// Intake speed ////
        Intake.sparkTargetSpeed = OI.calcIntakeSpeed()
    }

    override fun isFinished(): Boolean = false
}