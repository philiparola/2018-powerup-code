package com.team2898.robot.motion.pathfinder

import com.beust.klaxon.convert
import com.team2898.engine.async.pools.ComputePool
import com.team2898.engine.async.util.go
import com.team2898.robot.config.MotionProfileConfig.FILE_EXTENSION
import com.team2898.robot.config.MotionProfileConfig.FILE_PATH
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import jaci.pathfinder.Waypoint
import jaci.pathfinder.modifiers.TankModifier
import kotlinx.coroutines.experimental.Deferred
import kotlinx.coroutines.experimental.async
import kotlinx.serialization.json.JSON
import sun.security.provider.MD5
import java.io.File
import java.math.BigInteger
import java.nio.file.Path
import java.util.*
import java.util.concurrent.Future
import kotlin.experimental.and

object ProfileGenerator {
    fun deferProfile(profile: ProfileSettings): Deferred<Pair<Trajectory, Trajectory>> = async(ComputePool) {
        val mprofile: Trajectory
        val serialized = JSON().stringify(profile)
        val mdEnc = java.security.MessageDigest.getInstance("MD5")
        // Encryption algorithmy
        val md5Base16 = BigInteger(1, mdEnc.digest(serialized.toByteArray()))     // calculate md5 hash
        val sum = Base64.getEncoder().encodeToString(md5Base16.toByteArray()).trim()     // convert from base16 to base64 and remove the new line character
        val file =File("$FILE_PATH/$sum.$FILE_EXTENSION")


        if (file.exists()) { // if the file exists
            mprofile = Pathfinder.readFromCSV(file)
        } else { // if not
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

        Pair<Trajectory, Trajectory>(modifier.leftTrajectory, modifier.rightTrajectory)
    }

}
