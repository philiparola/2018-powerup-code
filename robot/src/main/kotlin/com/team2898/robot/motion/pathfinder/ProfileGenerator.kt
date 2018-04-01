package com.team2898.robot.motion.pathfinder

import com.team2898.engine.logging.LogLevel
import com.team2898.engine.logging.Logger
import com.team2898.engine.logging.reflectLocation
import com.team2898.robot.config.MotionProfileConfig.FILE_EXTENSION
import com.team2898.robot.config.MotionProfileConfig.FILE_PATH
import edu.wpi.first.wpilibj.Timer
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import jaci.pathfinder.modifiers.TankModifier
import kotlinx.serialization.json.JSON
import java.io.File
import java.security.MessageDigest
import kotlin.system.measureTimeMillis

object ProfileGenerator {
    fun genProfile(profile: ProfileSettings): Pair<Trajectory, Trajectory> {
        val serialized = JSON().stringify(profile)
        val file = File("$FILE_PATH/${serialized.md5()}.$FILE_EXTENSION")
        val logInfo = Logger.logInfo(reflectLocation(), LogLevel.INFO, "Starting profile generation, hash of ${serialized.md5()}")

        val mprofile: Trajectory
        if (file.exists()) {
            Logger.logInfo(reflectLocation(), LogLevel.INFO, "Pregenerated profile found!")
            mprofile = Pathfinder.readFromCSV(file)
        } else {
            Logger.logInfo(reflectLocation(), LogLevel.INFO, "Pregenerated profile not found, generating!")
            val start = Timer.getFPGATimestamp()
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
            val dt = Timer.getFPGATimestamp() - start
            Logger.logInfo(reflectLocation(), LogLevel.INFO, "Generated profile in ${"%.3f".format(dt)} seconds")
        }

        val modifier = TankModifier(mprofile).modify(profile.wheelbaseWidth)

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
