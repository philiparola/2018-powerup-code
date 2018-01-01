package com.team2898.engine.util

import edu.wpi.first.wpilibj.Timer

fun clamp(value: Double, min: Double, max: Double) = Math.max(min, Math.min(max, value))

fun millis(): Double = Timer.getFPGATimestamp()*1000
fun seconds(): Double = Timer.getFPGATimestamp()
