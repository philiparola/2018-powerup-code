package com.team2898.robot.motion

import com.team2898.engine.extensions.avg
import com.team2898.engine.kinematics.RigidTransform2d
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.kinematics.Translation2d
import com.team2898.engine.kinematics.Twist2d
import com.team2898.robot.config.RobotPhysicalConf.drivebaseWidthInches

fun forwardKinematics(deltas: Pair<Double, Double>): Twist2d {
    val w = deltas.avg() * 2 / drivebaseWidthInches
    return forwardKinematics(deltas, w)
}

fun forwardKinematics(deltas: Pair<Double, Double>, dtheta: Double): Twist2d =
        Twist2d(dx = deltas.avg(), dy = 0.0, dtheta = dtheta)

fun integrateForwardKinematics(currentPose: RigidTransform2d, delta: Twist2d): RigidTransform2d {
    var currentX = currentPose.translation.x
    var currentY = currentPose.translation.y
    var currentHeading = currentPose.rotation.radians

    currentHeading += delta.dtheta
    currentX += delta.dx * Math.cos(currentHeading)
    currentY += delta.dy * Math.sin(currentHeading)
    return RigidTransform2d(Translation2d(currentX, currentY), Rotation2d.createFromRadians(currentHeading))
}

//fun forwardKinematicsGyro(currentPose: RigidTransform2d, wheelDeltas: Pair<Double, Double>, heading: Double) {
//    val forwardDistance = wheelDeltas.avg()
//    val cosPlus =
//}

