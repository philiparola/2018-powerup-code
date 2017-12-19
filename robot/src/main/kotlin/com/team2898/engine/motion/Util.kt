package com.team2898.engine.motion

/** Stores left + right motor speed + brake value
 * @param left left power/speed
 * @param right right power/speed
 * @param brake brake mode or not
 */
data class DriveSignal(val left: Double = 0.0, val right: Double = 0.0, val brake: Boolean = false)

/** Generated motion profile for built in profiling, Talon profiling compatible
 * @param position position in rotations
 * @param velocity velocity in rpm
 * @param duration duration in ms
 */
data class TalonPoint(val position: Double, val velocity: Double, val duration: Double)
