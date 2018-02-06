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
import java.security.MessageDigest
import java.util.*
import java.util.concurrent.Future
import kotlin.experimental.and
import kotlin.experimental.or

object ProfileGenerator {
    fun deferProfile(profile: ProfileSettings): Pair<Trajectory, Trajectory> {
        println("deferProfile()")
        val mprofile: Trajectory
        val serialized = JSON().stringify(profile)
//        val mdEnc = java.security.MessageDigest.getInstance("MD5")
//        val md5Base16 = BigInteger(1, mdEnc.digest(serialized.toByteArray()))     // calculate md5 hash
//        val sum = Base64.getEncoder().encodeToString(md5Base16.toByteArray()).trim()     // convert from base16 to base64 and remove the new line character
        val file = File("$FILE_PATH/${serialized.md5()}.$FILE_EXTENSION")
        println("$file")


        if (file.exists()) { // if the file exists
            mprofile = Pathfinder.readFromCSV(file)
            println("reading")
        } else { // if not
            println("saving")
            val path = tripleToWaypoint(profile.wayPoints)
            println("path reading")
            val config = Trajectory.Config(
                    profile.fitMethod,
                    profile.sampleRate,
                    1.0 / profile.hz,
                    profile.maxVel,
                    profile.maxAcc,
                    profile.maxJerk
            )
            println("config done")
            mprofile = Pathfinder.generate(path, config)
            println("gen done")
            Pathfinder.writeToCSV(file, mprofile)
            println("written")
        }
        val modifier = TankModifier(mprofile).modify(profile.wheelbaseWidth)
        println("${modifier.leftTrajectory.length()}")

        return Pair<Trajectory, Trajectory>(modifier.leftTrajectory, modifier.rightTrajectory)
    }


    private val hexArray = "0123456789ABCDEF".toCharArray()
    fun bytesToHex(bytes: ByteArray): String {
        val hexChars = CharArray(bytes.size * 2)
        for (j in bytes.indices) {
            val v = (bytes[j] and 0xFF.toByte()).toInt()

            hexChars[j * 2] = hexArray[v ushr 4]
            hexChars[j * 2 + 1] = hexArray[v and 0x0F]
        }
        return String(hexChars)
    }


    fun String.md5(): String {
        val md = MessageDigest.getInstance("MD5")
        val digested = md.digest(toByteArray())
        return digested.joinToString("") {
            String.format("%02x", it)
        }
    }
}
