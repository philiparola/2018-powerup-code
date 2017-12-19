package com.team2898.engine.types

/** Stores a time along with a generic stamp
 * @param time time
 * @param stamp value associated with the time
 */
data class Timestamp<T>(val time: Double, val stamp: T)


/** Simply stores a minimum value, maximum value, and a default nominal value
 * @param min minimum value
 * @param max maximum value
 * @param norm default value
 */
data class MinMax(val min: Double, val max: Double, val norm: Double = (max+min)/2)


