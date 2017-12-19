package com.team2898.engine.kinematics

import com.team2898.engine.math.linear.rotateVector2D
import com.team2898.engine.types.Interpolable

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D
import java.text.DecimalFormat

// Represents 2D rotation on the x, y plane

class Rotation2d: Interpolable<Rotation2d> {
    companion object { // Allows us to have static traits
        @JvmStatic
        fun createFromRadians(angleRadians: Double): Rotation2d {
            return Rotation2d(Math.cos(angleRadians), Math.sin(angleRadians))
        }
        @JvmStatic
        fun createFromDegrees(angleDegrees: Double): Rotation2d {
            return createFromRadians(Math.toRadians(angleDegrees))
        }
    }

    val fmt: DecimalFormat = DecimalFormat("#0.000")

    // represents as [cos(θ), sin(θ)]. default is 0° (pointing right)
    // for instance, 45° -> [cos(45°), sin(45°)] -> [(√2)/2, (√2)/2]
    private var rotation: Vector2D = Vector2D(1.0, 0.0)

    val kEpsilon = 1.0E-8 // "Accuracy," so to speak. We assume values smaller than this are negligible/zero

    constructor() {
    }

    constructor(cos: Double, sin: Double) {
        rotation = Vector2D(cos, sin).normalize()
    }

    constructor(toSet: Rotation2d) {
        rotation = toSet.rotation.normalize()
    }

    constructor(toSetVector: Vector2D) {
        rotation = toSetVector.normalize()
    }

    fun vector(): Vector2D = rotation

    fun cos(): Double = rotation.x
    fun sin(): Double = rotation.y

    fun tan(): Double {
        if (rotation.x > kEpsilon) return rotation.y/rotation.x
        return if (rotation.y >= 0.0) Double.POSITIVE_INFINITY else Double.NEGATIVE_INFINITY
    }

    fun radians(): Double = Math.atan2(rotation.y, rotation.x)

    fun degrees(): Double = Math.toDegrees(radians())

    /*
    * Rotation matrix operation to add two Rotation2ds
    */
    fun rotateBy(toRotateBy: Rotation2d): Rotation2d {
        val rotated = rotateVector2D(this.rotation, toRotateBy.vector())
        return Rotation2d(rotated)
    }

    // Returns inverse of rotation, can undo the effects of the above matrix rotation
    fun inverse(): Rotation2d {
        return Rotation2d(this.rotation.x, -this.rotation.y)
    }

    override fun interpolate(upperVal: Rotation2d, interpolatePoint: Double): Rotation2d {
        when {
            interpolatePoint<=0 -> return Rotation2d(this)
            interpolatePoint>=1 -> return Rotation2d(upperVal)
            else -> return this.rotateBy(
                    Rotation2d.createFromRadians(inverse().rotateBy(upperVal).radians() * interpolatePoint)
            )
        }
    }

    override fun toString(): String {
        return "${fmt.format(degrees())} deg"
    }
}
