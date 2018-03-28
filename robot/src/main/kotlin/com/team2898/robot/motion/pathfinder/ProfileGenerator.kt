package com.team2898.robot.motion.pathfinder

import com.team2898.robot.config.MotionProfileConfig.FILE_EXTENSION
import com.team2898.robot.config.MotionProfileConfig.FILE_PATH
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import jaci.pathfinder.modifiers.TankModifier
import kotlinx.serialization.json.JSON
import java.io.File
import java.security.MessageDigest

object ProfileGenerator {
    fun genProfile(profile: ProfileSettings): Pair<Trajectory, Trajectory> {
        val mprofile: Trajectory
        val serialized = JSON().stringify(profile)
        val file = File("$FILE_PATH/${serialized.md5()}.$FILE_EXTENSION")
        println("$file")


        if (file.exists()) {
            mprofile = Pathfinder.readFromCSV(file)
        } else {
            val path = tripleToWaypoint(profile.wayPoints)
            val config = Trajectory.Config(
                    profile.fitMethod,
                    profile.sampleRate,
                    1.0 / profile.hz,
                    profile.maxVel,
                    profile.maxAcc,
                    profile.maxJerk
            )
            mprofile = Pathfinder.generate(path, config)
            Pathfinder.writeToCSV(file, mprofile)
        }
        val modifier = TankModifier(mprofile).modify(profile.wheelbaseWidth)
        println("${modifier.leftTrajectory.length()}")
        return Pair<Trajectory, Trajectory>(modifier.leftTrajectory, modifier.rightTrajectory)
    }

    fun String.md5(): String {
        val md = MessageDigest.getInstance("MD5")
        val digested = md.digest(toByteArray())
        return digested.joinToString("") {
            String.format("%02x", it)
        }
    }
}
