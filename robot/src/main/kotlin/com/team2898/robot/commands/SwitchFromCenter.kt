package com.team2898.robot.commands

import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.motion.DriveSignal
import com.team2898.robot.motion.pathfinder.*
import com.team2898.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.command.*
import jaci.pathfinder.Trajectory
import kotlin.math.roundToInt

class SwitchFromCenter(val ourColorOnRight: Boolean) : CommandGroup() {
    val PROFILE_1 = if (ourColorOnRight) rightSwitchFromCenterProfile else leftSwitchFromCenterProfile
    val PROFILE_2 = if (ourColorOnRight) rightSwitchFromCenterProfile else leftSwitchFromCenterProfile

    init {
        addSequential(IntakeCommand(time = .0, power = Pair(.5, .5), piston = DoubleSolenoid.Value.kReverse))
        addSequential(ProfileFollower(
                ProfileGenerator.genProfile(
                        ProfileSettings(
                                hz = 100,
                                maxVel = 3.5,
                                maxAcc = 2.0,
                                maxJerk = 10.0,
                                wheelbaseWidth = 2.175,
                                wayPoints = convWaypoint(PROFILE_1),
                                fitMethod = Trajectory.FitMethod.HERMITE_CUBIC,
                                sampleRate = Trajectory.Config.SAMPLES_HIGH
                        )
                )
        ))
        addSequential(WaitForChildren())
        addSequential(object : InstantCommand() {
            override fun initialize() {
                Drivetrain.openLoopPower = DriveSignal.BRAKE
            }
        })
        addSequential(SetArm(Rotation2d.createFromDegrees(60.0)))
        addSequential(IntakeCommand(time = 1.0, power = Pair(.5, .5), piston = DoubleSolenoid.Value.kForward))
        addParallel(SetArm(Rotation2d.createFromDegrees(15.0)))
        addSequential(DtCompOpen(2.0, Pair(-0.4, -0.4)))
        addSequential(WaitCommand(1.0))
    }
}