package com.team2898.engine.kinematics

import com.team2898.engine.types.Interpolable
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D


// Represents 2D pose (position, orientation) on the x, y plane
class RigidTransform2d: Interpolable<RigidTransform2d> {
    companion object {
        @JvmStatic
        val kEpsilon: Double = 1.0E-8

        @JvmStatic
        fun fromTranslation(translation: Translation2d): RigidTransform2d {
            return RigidTransform2d(translation, Rotation2d())
        }

        @JvmStatic
        fun fromRotation(rotation: Rotation2d): RigidTransform2d {
            return RigidTransform2d(Translation2d(), rotation)
        }

        /**
         * Creates a new RigidTransform2d given a velocity + curvature
         */

        @JvmStatic
        fun fromDelta(twist: Twist2d): RigidTransform2d {
            val sinTheta: Double = Math.sin(twist.dtheta)
            val cosTheta: Double = Math.cos(twist.dtheta)
            var s: Double = 0.0
            var c: Double = 0.0

            if (Math.abs(twist.dtheta) < kEpsilon) {
                s = 1.0 - (1.0/6.0) * Math.pow(twist.dtheta, 2.0)
                c = 0.5 * twist.dtheta
            } else {
                s = sinTheta/twist.dtheta
                c = (1.0 - cosTheta)/twist.dtheta
            }
            return RigidTransform2d(
                    Translation2d(
                            twist.dx * s - twist.dy * c,
                            twist.dx * c + twist.dy * s
                    ),
                    Rotation2d(cosTheta, sinTheta)
            )
        }

    }


    private var m_translation: Translation2d = Translation2d()
    private var m_rotation: Rotation2d = Rotation2d()

    constructor()

    constructor(translation: Translation2d, rotation: Rotation2d) {
        m_translation = translation
        m_rotation = rotation
    }

    constructor(toCopy: RigidTransform2d) {
        m_translation = Translation2d(toCopy.m_translation)
        m_rotation = Rotation2d(toCopy.m_rotation)
    }

    fun getTranslation(): Translation2d {return m_translation}
    fun getRotation(): Rotation2d {return m_rotation}

    fun setTranslation(translation: Translation2d) {m_translation = translation}
    fun setRotation(rotation: Rotation2d) {m_rotation = rotation}
    fun setTranslation(translation: Vector2D) {m_translation = Translation2d(translation)}
    fun setRotation(rotation: Vector2D) {m_rotation = Rotation2d(rotation)}

    /** To transform a 2D pose by another, we first translate by another translation then rotate by the other rotation
     */
    fun transformBy(other: RigidTransform2d): RigidTransform2d {
        return RigidTransform2d(
                m_translation.translateBy(other.getTranslation().rotateByOrigin(m_rotation)),
                m_rotation.rotateBy(other.getRotation())
        )
    }

    /** Inverse of the transformation undoes effect of translating by the own transform
     * (if that makes any sense)
     */

    fun inverse(): RigidTransform2d {
        val rotationInverted: Rotation2d = m_rotation.inverse()
        return RigidTransform2d(
                m_translation.inverse().rotateByOrigin(rotationInverted),
                rotationInverted
        )
    }

    /** Linear interpolation (sucks)
     * TODO: Update to constant curvature interpolation
     */

    override fun interpolate(upperVal: RigidTransform2d, interpolatePoint: Double): RigidTransform2d {
        when {
            (interpolatePoint <= 0) -> return RigidTransform2d(this)
            (interpolatePoint >= 1) -> return RigidTransform2d(upperVal)
            else -> return RigidTransform2d(
                    m_translation.interpolate(upperVal.getTranslation(), interpolatePoint),
                    m_rotation.interpolate(upperVal.getRotation(), interpolatePoint)
            )
        }
    }

    override fun toString(): String {
        return "T: ${m_translation.toString()}, R: ${m_rotation.toString()}"
    }

}
