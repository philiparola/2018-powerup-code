package com.team2898.engine.math

fun clamp(value: Double, magnitude: Double) = clamp(value, -magnitude, magnitude)
fun clamp(value: Double, min: Double, max: Double) = Math.min(max, Math.max(min, value))

