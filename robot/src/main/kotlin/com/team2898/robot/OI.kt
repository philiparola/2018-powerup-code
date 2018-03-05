package com.team2898.robot

import com.team2898.engine.extensions.Vector2D.l2
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.robot.config.ControllerConf.TESTING
import com.team2898.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj.Joystick
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D
import kotlin.math.abs
import kotlin.math.sign

object OI {

    fun deadzone(value: Double): Double {
        if (Math.abs(value) < 0.15) return 0.0
        return value
    }

    fun cube(value: Double): Double = Math.pow(value, 3.0)
    fun square(value: Double): Double = Math.pow(value, 2.0) * if (value > 0) 1 else -1

    fun process(value: Double, deadzone: Boolean = true, cube: Boolean = false, square: Boolean = false): Double {
        var localValue = value
        if (deadzone)
            localValue = deadzone(value)
        if (cube) {
            localValue = cube(localValue)
        } else if (square) {
            localValue = square(localValue)
        }

        return localValue
    }

    val driverController: Joystick = Joystick(0)
    val operatorController: Joystick = Joystick(1)

    val throttle
        get() = process(driverController.getRawAxis(1))
    val turn
        get() = turn()
    val quickTurn: Boolean
        get() = process(Math.max(driverController.getRawAxis(2), driverController.getRawAxis(3))) != 0.0
    val leftTrigger
        get() = process(driverController.getRawAxis(2) * 0.8, square = true)
    val rightTrigger
        get() = process(driverController.getRawAxis(3) * 0.8, square = true)
    val brake
        get() = driverController.getRawButton(8) || driverController.getRawButton(9)
    val lowGear
        get() = driverController.getRawButton(5)
    val highGear
        get() = driverController.getRawButton(6)


    // elev up: op right trig
    // elev down: op left trig

    // manip forward: op A
    // manip backwards: op B

    // intake up/down: left stick Y
    // intake spin: right stick Yp

    val opA
        get() = operatorController.getRawButton(1)
    val opB
        get() = operatorController.getRawButton(2)
    val opX
        get() = operatorController.getRawButton(3)
    val opY
        get() = operatorController.getRawButton(4)

    val opLTrig
        get() = operatorController.getRawAxis(2)
    val opRTrig
        get() = operatorController.getRawAxis(3)

    val opLY
        get() = operatorController.getRawAxis(1)
    val opRY
        get() = operatorController.getRawAxis(5)
    val opLShoulder
        get()= operatorController.getRawButton(5)
    val opRShoulder
        get()= operatorController.getRawButton(6)

    val openPiston
        get() = operatorController.getRawButton(9)


    // intake spark -> joystick left Y
    // deploy talons -> button D pad up and down ish


    fun turn(): Double {
        if (!TESTING) return process(driverController.getRawAxis(4), square = true) * 0.8
        val y = driverController.getRawAxis(5)
        val x = driverController.getRawAxis(4)

        val theta = Math.atan2(y, x)
        val thetaDeg = Math.toDegrees(theta)

        val thetaTurn = ((thetaDeg - 90) * -1) / 90
        val magnitude = Vector2D(y, x).l2

        val turn = magnitude * thetaTurn
        return turn
    }

    fun calcIntakeSpeed(): Vector2D {
        val x = operatorController.getRawAxis(1) // power
        val y = operatorController.getRawAxis(2) // offset ish
        // left, right
        return Vector2D(x - sign(y) * abs(1 - y), (x + sign(y) * abs(1 - y)))
    }
}