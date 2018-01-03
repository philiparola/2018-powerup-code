package com.team2898.robot.config.RobotStateConf

import kotlin.math.roundToInt

const val BACKLOG_HZ = 25.0
const val POSE_BACKLOG_SIZE_SECONDS = 0.5


val POSE_BACKLOG_SIZE = (BACKLOG_HZ * POSE_BACKLOG_SIZE_SECONDS).roundToInt()
val RUN_EVERY_NUMBER = (100.0 / BACKLOG_HZ).roundToInt()
