package com.team2898.robot.commands

import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.motion.CheesyDrive
import edu.wpi.first.wpilibj.command.Command
import com.team2898.robot.OI
import com.team2898.robot.commands.manipulator.Deploy
import com.team2898.robot.commands.manipulator.HeavyThrow
import com.team2898.robot.commands.manipulator.LightThrow
import com.team2898.robot.commands.manipulator.ManipIntake
import com.team2898.robot.subsystems.Drivetrain
import com.team2898.robot.subsystems.Elevator
import com.team2898.robot.subsystems.Intake
import com.team2898.robot.subsystems.Navx
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
    val currentTime
        get() = startTime - Timer.getFPGATimestamp()

    override fun initialize() {
//        files.forEach {
//            it.writeText("time, vel\n")
//        }
        Drivetrain.controlMode = Drivetrain.ControlModes.OPEN_LOOP
        Drivetrain.zeroEncoders()
        Navx.reset()
        startTime = Timer.getFPGATimestamp()
    }

    override fun execute() {
        CheesyDrive.updateQuickTurn(OI.quickTurn)
        Drivetrain.openLoopPower = CheesyDrive.updateCheesy(
                (if (!OI.quickTurn) OI.turn else -OI.leftTrigger + OI.rightTrigger),
                -OI.throttle,
                OI.quickTurn,
                true
        )
        if (OI.aButton) Deploy().start()
        if (OI.bButton) ManipIntake().start()
        if (OI.xButton) LightThrow().start()
        if (OI.yButton) HeavyThrow().start()

        ////// intake piston ////
        if (OI.closeIntake) Intake.intakePistonState = Intake.PistonState.CLOSED
        else Intake.intakePistonState = Intake.PistonState.OPEN

        ////// Elevator /////
        if (OI.raiseElev) SetElevator(Elevator.currentPosFt + 1).start()
        if (OI.lowerIntake) SetElevator(Elevator.currentPosFt - 1).start()

        ////// Intake /////
        if (OI.lowerIntake) Intake.talonTargetPos = Rotation2d.createFromDegrees(Intake.currentPos.degrees - 10)
        if (OI.raiseIntake) Intake.talonTargetPos = Rotation2d.createFromDegrees(Intake.currentPos.degrees + 10)

        ////// Intake speed ////
        Intake.sparkTargetSpeed =  OI.calcIntakeSpeed()
    }

    override fun isFinished(): Boolean = false
}