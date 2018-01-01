package com.team2898.robot.motion

import com.team2898.engine.extensions.Vector2D.avg
import com.team2898.engine.kinematics.RigidTransform2d
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.kinematics.Translation2d
import com.team2898.engine.kinematics.Twist2d
import com.team2898.engine.math.*
import com.team2898.robot.config.RobotPhysicalConf.drivebaseWidthInches
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D

fun forwardKinematics(deltas: Vector2D): Twist2d {
    val w = deltas.avg * 2 / drivebaseWidthInches
    return forwardKinematics(deltas, w)
}

fun forwardKinematics(deltas: Vector2D, dtheta: Double): Twist2d =
        Twist2d(dx = deltas.avg, dy = 0.0, dtheta = dtheta)

fun integrateForwardKinematics(currentPose: RigidTransform2d, delta: Twist2d): RigidTransform2d {
    var currentX = currentPose.translation.x
    var currentY = currentPose.translation.y
    var currentHeading = currentPose.rotation.radians

    currentHeading += delta.dtheta
    currentX += delta.dx * Math.cos(currentHeading)
    currentY += delta.dy * Math.sin(currentHeading)
    return RigidTransform2d(Translation2d(currentX, currentY), Rotation2d.createFromRadians(currentHeading))
}
